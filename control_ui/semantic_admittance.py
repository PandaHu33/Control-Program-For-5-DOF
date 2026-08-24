"""Central canonical-space semantic admittance.

The module is deliberately independent from the UI prompt state machine and
from the WA100 low-level servo.  It accepts immutable nominal/measured
canonical snapshots and produces a bounded corrected snapshot plus an audit
record.  OFF is a strict pass-through and SHADOW never authorizes its candidate
for dispatch.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np


SEMANTIC_MODES = ("OFF", "SHADOW", "HAND_ONLY", "ARM_HAND")
HAND_ALL_VALID_MASK = (1 << 6) - 1
WRIST_ALL_VALID_MASK = (1 << 6) - 1


def normalize_semantic_mode(value: Any) -> str | None:
    if isinstance(value, bool):
        return None
    text = str(value or "").strip().upper().replace("-", "_")
    aliases = {"HANDONLY": "HAND_ONLY", "ARMHAND": "ARM_HAND"}
    text = aliases.get(text, text)
    return text if text in SEMANTIC_MODES else None


def _finite(value: Any, size: int) -> np.ndarray | None:
    try:
        result = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    return result if result.size == size and np.all(np.isfinite(result)) else None


def arm_forward_kinematics(q_rad: Sequence[float], link_3_m: float = 0.424,
                           link_5_m: float = 0.424) -> dict[str, list[float]]:
    """Existing UI arm FK, expressed as a canonical wrist pose.

    q1 is base yaw, q2/q3 are the planar shoulder/elbow convention already
    used by index.html, and q4 is wrist body roll.
    """
    q = _finite(q_rad, 5)
    if q is None:
        raise ValueError("q_a_measured must contain five finite joint angles")
    radial = link_3_m * math.cos(q[1]) + link_5_m * math.cos(q[1] - q[2])
    position = [radial * math.cos(q[0]), radial * math.sin(q[0]),
                link_3_m * math.sin(q[1]) + link_5_m * math.sin(q[1] - q[2])]
    half = float(q[3]) * 0.5
    return {"position_m": position, "orientation_xyzw": [math.sin(half), 0.0, 0.0, math.cos(half)]}


def arm_inverse_kinematics(position_m: Sequence[float], wrist_roll_rad: float = 0.0,
                           link_3_m: float = 0.424, link_5_m: float = 0.424) -> list[float]:
    """Inverse of the registered three-joint UI FK, with bounded acos."""
    p = _finite(position_m, 3)
    if p is None:
        raise ValueError("position_m must contain three finite values")
    x, y, z = map(float, p)
    radius = math.hypot(x, y)
    reach = min(link_3_m + link_5_m, max(abs(link_3_m - link_5_m) + 1e-6,
                                         math.sqrt(x*x + y*y + z*z)))
    elbow_inner = math.acos(np.clip(
        (link_3_m*link_3_m + link_5_m*link_5_m - reach*reach) /
        (2.0 * link_3_m * link_5_m), -1.0, 1.0))
    q3 = math.pi - elbow_inner
    shoulder_offset = math.acos(np.clip(
        (link_3_m*link_3_m + reach*reach - link_5_m*link_5_m) /
        (2.0 * link_3_m * reach), -1.0, 1.0))
    return [math.atan2(y, x), math.atan2(z, radius) + shoulder_offset, q3,
            float(wrist_roll_rad)]


def canonical_snapshot(kind: str, *, timestamp_ns: int, decision_seq: int,
                       wrist_pose: Mapping[str, Any] | None,
                       hand_position_units: Any, hand_skeleton: Any,
                       wrist_mask: int, hand_mask: int,
                       source: str, calibrations: Mapping[str, Any] | None = None,
                       invalid_reasons: Sequence[str] = ()) -> dict[str, Any]:
    return {
        "type": f"canonical_{kind}", "schema_version": 1,
        "timestamp_ns": int(timestamp_ns), "decision_seq": int(decision_seq),
        "wrist_pose_C": copy.deepcopy(wrist_pose) if wrist_pose else None,
        "hand_position_units": copy.deepcopy(hand_position_units),
        "hand_skeleton_C": copy.deepcopy(hand_skeleton),
        "validity": {"wrist_dof_mask": int(wrist_mask), "hand_dof_mask": int(hand_mask)},
        "source": str(source), "calibrations": dict(calibrations or {}),
        "invalid_reasons": sorted(set(str(item) for item in invalid_reasons if item)),
    }


@dataclass(frozen=True)
class SemanticLimits:
    measured_timeout_sec: float = 0.25
    current_timeout_sec: float = 0.25
    trigger_ma: float = 80.0
    wrist_max_displacement_m: float = 0.025
    wrist_max_speed_m_s: float = 0.02
    hand_max_unload_ratio: float = 0.25
    registered_retreat_direction_C: tuple[float, float, float] = (-1.0, 0.0, 0.0)


class SemanticAdmittance:
    """Thread-safe bounded semantic correction state."""

    def __init__(self, limits: SemanticLimits | None = None, mode: str = "OFF",
                 hand_kinematics: Any = None):
        self.limits = limits or SemanticLimits()
        self.lock = threading.RLock()
        self.mode = normalize_semantic_mode(mode) or "OFF"
        self.hand_kinematics = hand_kinematics
        self._wrist_offset_m = 0.0
        self._last_update_ns = 0

    def set_mode(self, mode: Any) -> str:
        normalized = normalize_semantic_mode(mode)
        if normalized is None:
            raise ValueError("mode must be OFF, SHADOW, HAND_ONLY, or ARM_HAND")
        with self.lock:
            self.mode = normalized
            if normalized == "OFF":
                self._wrist_offset_m = 0.0
        return normalized

    def update(self, nominal: Mapping[str, Any], measured: Mapping[str, Any] | None,
               current_residual: Any, *, current_timestamp_ns: int = 0,
               now_ns: int | None = None, open_hand_skeleton: Any = None) -> dict[str, Any]:
        now_ns = int(now_ns or time.time_ns())
        measured = dict(measured or {})
        with self.lock:
            mode = self.mode
            dt = min(0.1, max(0.0, (now_ns - self._last_update_ns) / 1e9)) if self._last_update_ns else 0.0
            self._last_update_ns = now_ns
            reasons: list[str] = []
            measured_age = (now_ns - int(measured.get("timestamp_ns") or 0)) / 1e9
            current_age = (now_ns - int(current_timestamp_ns or 0)) / 1e9
            measured_validity = measured.get("validity") or {}
            measured_ok = (
                0 <= measured_age <= self.limits.measured_timeout_sec
                and int(measured_validity.get("wrist_dof_mask", 0)) != 0
                and int(measured_validity.get("hand_dof_mask", 0)) != 0
            )
            residual = _finite(current_residual, 6)
            current_ok = residual is not None and 0 <= current_age <= self.limits.current_timeout_sec
            if not measured_ok:
                reasons.append("measured_canonical_stale_or_invalid")
            if not current_ok:
                reasons.append("current_residual_stale_or_invalid")
            safe = measured_ok and current_ok
            load = float(max(0.0, np.max(residual) - self.limits.trigger_ma)) if current_ok else 0.0
            confidence = float(np.clip(load / max(self.limits.trigger_ma, 1.0), 0.0, 1.0))
            triggered = safe and confidence > 0.0

            # No old correction may survive a fault or OFF transition.
            if not safe or mode == "OFF":
                self._wrist_offset_m = 0.0
            elif triggered and mode in {"SHADOW", "ARM_HAND"}:
                self._wrist_offset_m = min(
                    self.limits.wrist_max_displacement_m,
                    self._wrist_offset_m + self.limits.wrist_max_speed_m_s * confidence * dt,
                )
            else:
                self._wrist_offset_m = max(0.0, self._wrist_offset_m - self.limits.wrist_max_speed_m_s * dt)

            direction = _finite(self.limits.registered_retreat_direction_C, 3)
            direction = direction / np.linalg.norm(direction)
            delta_wrist = direction * self._wrist_offset_m
            nominal_pose = copy.deepcopy(nominal.get("wrist_pose_C"))
            corrected_pose = copy.deepcopy(nominal_pose)
            if corrected_pose and mode in {"SHADOW", "ARM_HAND"} and safe:
                corrected_pose["position_m"] = (
                    np.asarray(corrected_pose["position_m"], dtype=float) + delta_wrist
                ).tolist()
            else:
                delta_wrist[:] = 0.0

            nominal_units = _finite(nominal.get("hand_position_units"), 6)
            if nominal_units is None:
                nominal_units = np.full(6, 2000.0, dtype=float)
                reasons.append("nominal_hand_position_invalid")
                safe = False
            corrected_units = nominal_units.copy()
            unload_ratio = self.limits.hand_max_unload_ratio * confidence if triggered else 0.0
            if mode in {"SHADOW", "HAND_ONLY", "ARM_HAND"} and safe:
                corrected_units += unload_ratio * (2000.0 - nominal_units)
            else:
                unload_ratio = 0.0
            if self.hand_kinematics is None:
                corrected_hand = np.asarray(nominal.get("hand_skeleton_C"), dtype=float).reshape(21, 3)
                if not np.allclose(corrected_units, nominal_units):
                    reasons.append("wa100_kinematics_unavailable")
                    corrected_units = nominal_units.copy()
                    unload_ratio = 0.0
            else:
                corrected_hand = self.hand_kinematics.canonical_skeleton_21(corrected_units)
            nominal_hand = np.asarray(nominal.get("hand_skeleton_C"), dtype=float).reshape(21, 3)
            delta_hand = corrected_hand - nominal_hand
            delta_units = corrected_units - nominal_units

            corrected = canonical_snapshot(
                "corrected", timestamp_ns=int(nominal.get("timestamp_ns") or now_ns),
                decision_seq=int(nominal.get("decision_seq") or 0),
                wrist_pose=corrected_pose, hand_position_units=corrected_units.tolist(),
                hand_skeleton=corrected_hand.tolist(),
                wrist_mask=int((nominal.get("validity") or {}).get("wrist_dof_mask", 0)),
                hand_mask=int((nominal.get("validity") or {}).get("hand_dof_mask", 0)),
                source="semantic_admittance", calibrations=nominal.get("calibrations"),
                invalid_reasons=reasons,
            )
            dispatch_corrected = safe and mode in {"HAND_ONLY", "ARM_HAND"}
            audit = {
                "type": "semantic_admittance", "schema_version": 1,
                "timestamp_ns": now_ns, "decision_seq": corrected["decision_seq"],
                "mode": mode, "triggered": bool(triggered),
                "direction_confidence": confidence,
                "delta_wrist_C": [*delta_wrist.tolist(), 0.0, 0.0, 0.0],
                "delta_hand_position_units": delta_units.tolist(),
                "delta_hand_skeleton_C": delta_hand.tolist(),
                "hand_unload_ratio": unload_ratio,
                "degradation_reasons": reasons,
                "dispatch_corrected": bool(dispatch_corrected),
                "local_wa100_admittance_allowed": mode == "OFF",
            }
            return {"corrected": corrected, "audit": audit}
