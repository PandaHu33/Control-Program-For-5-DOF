"""Canonical master-side arm/hand semantics used by the E1 experiment.

The module deliberately stops at the nominal command boundary.  It does not
contain contact inference, admittance, or any E2 closed-loop behaviour.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import numpy as np

try:
    from .wa100_kinematics import (
        WA100Kinematics,
        WA100_CANONICAL_PARENTS,
        WA100_CANONICAL_POINT_NAMES,
        WA100_HAND_CHANNEL_NAMES,
    )
except ImportError:
    from wa100_kinematics import (
        WA100Kinematics,
        WA100_CANONICAL_PARENTS,
        WA100_CANONICAL_POINT_NAMES,
        WA100_HAND_CHANNEL_NAMES,
    )

try:
    from .canonical_planning import CanonicalPlannerConfig, CanonicalWristPlanner
except ImportError:
    from canonical_planning import CanonicalPlannerConfig, CanonicalWristPlanner

CANONICAL_SCHEMA_VERSION = 4
WRIST_DOF_POSITION_X = 1 << 0
WRIST_DOF_POSITION_Y = 1 << 1
WRIST_DOF_POSITION_Z = 1 << 2
WRIST_DOF_ROLL = 1 << 3
WRIST_SUPPORTED_MASK = (
    WRIST_DOF_POSITION_X | WRIST_DOF_POSITION_Y | WRIST_DOF_POSITION_Z | WRIST_DOF_ROLL
)
HAND_ALL_VALID_MASK = (1 << 6) - 1
CANONICAL_JOINT_NAMES = WA100_CANONICAL_POINT_NAMES
CANONICAL_PARENTS = WA100_CANONICAL_PARENTS
E1_CONDITIONS = {
    ("hand_vision", "vr"): "M1-PICO",
    ("controller_delta", "glove"): "M2-VR-GLOVE",
    ("gamepad", "glove"): "M3-GAMEPAD-GLOVE",
    ("keyboard", "glove"): "M4-KEYBOARD-GLOVE",
}


def _finite_vector(value: Any, size: int) -> Optional[np.ndarray]:
    try:
        result = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if result.size != size or not np.all(np.isfinite(result)):
        return None
    return result


def normalize_quaternion_xyzw(value: Any) -> Optional[np.ndarray]:
    result = _finite_vector(value, 4)
    if result is None:
        return None
    norm = float(np.linalg.norm(result))
    if norm <= 1e-12:
        return None
    result = result / norm
    # q and -q represent the same orientation.  Canonicalize the sign so
    # replay output is deterministic and adjacent sign flips never jump.
    if result[3] < 0.0:
        result = -result
    return result


def quaternion_multiply_xyzw(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    lx, ly, lz, lw = np.asarray(left, dtype=float)
    rx, ry, rz, rw = np.asarray(right, dtype=float)
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ], dtype=float)


def quaternion_conjugate_xyzw(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = np.asarray(value, dtype=float)
    return np.array([-x, -y, -z, w], dtype=float)


def quaternion_rotate_xyzw(quaternion: Sequence[float], vector: Sequence[float]) -> np.ndarray:
    q = normalize_quaternion_xyzw(quaternion)
    if q is None:
        raise ValueError("invalid quaternion")
    v = np.r_[np.asarray(vector, dtype=float), 0.0]
    return quaternion_multiply_xyzw(quaternion_multiply_xyzw(q, v), quaternion_conjugate_xyzw(q))[:3]


def rotvec_to_quaternion_xyzw(value: Sequence[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis = vector / angle
    return np.r_[axis * math.sin(angle * 0.5), math.cos(angle * 0.5)]


def euler_deg_to_quaternion_xyzw(value: Sequence[float], order: str = "zyx") -> np.ndarray:
    angles = dict(zip("xyz", np.asarray(value, dtype=float)))
    result = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    for axis in str(order).lower():
        half = math.radians(float(angles[axis])) * 0.5
        item = np.array([0.0, 0.0, 0.0, math.cos(half)], dtype=float)
        item["xyz".index(axis)] = math.sin(half)
        result = quaternion_multiply_xyzw(result, item)
    normalized = normalize_quaternion_xyzw(result)
    if normalized is None:
        raise ValueError("invalid Euler rotation")
    return normalized


def integrate_wrist_pose(
    pose: Dict[str, Any], adapter_wrist_delta: Sequence[float]
) -> Dict[str, list[float]]:
    """Apply world translation on the left and body rotation on the right."""
    delta = _finite_vector(adapter_wrist_delta, 6)
    position = _finite_vector((pose or {}).get("position_m"), 3)
    orientation = normalize_quaternion_xyzw((pose or {}).get("orientation_xyzw"))
    if delta is None or position is None or orientation is None:
        raise ValueError("invalid wrist pose or delta")
    next_orientation = normalize_quaternion_xyzw(
        quaternion_multiply_xyzw(orientation, rotvec_to_quaternion_xyzw(delta[3:6]))
    )
    return {
        "position_m": (position + delta[:3]).tolist(),
        "orientation_xyzw": next_orientation.tolist(),
    }


def validate_canonical_goal(payload: Any) -> tuple[bool, str]:
    if not isinstance(payload, dict) or payload.get("type") != "canonical_goal":
        return False, "invalid_type"
    schema_version = int(payload.get("schema_version", 0))
    if schema_version not in {1, 2, 3, CANONICAL_SCHEMA_VERSION}:
        return False, "invalid_schema"
    if _finite_vector(payload.get("adapter_wrist_delta"), 6) is None:
        return False, "invalid_adapter_wrist_delta"
    pose = payload.get("wrist_pose_C") or {}
    if _finite_vector(pose.get("position_m"), 3) is None or normalize_quaternion_xyzw(pose.get("orientation_xyzw")) is None:
        return False, "invalid_wrist_pose"
    validity = payload.get("validity") or {}
    hand_mask_key = "hand_dof_mask" if schema_version >= 4 else "hand_node_mask"
    if not isinstance(validity.get("wrist_dof_mask"), int) or not isinstance(validity.get(hand_mask_key), int):
        return False, "invalid_validity_masks"
    if schema_version >= 2:
        raw_intent = payload.get("raw_intent") or {}
        raw_pose = raw_intent.get("wrist_pose_C") or {}
        if (
            _finite_vector(raw_intent.get("adapter_wrist_delta"), 6) is None
            or _finite_vector(raw_pose.get("position_m"), 3) is None
            or normalize_quaternion_xyzw(raw_pose.get("orientation_xyzw")) is None
            or _finite_vector(raw_intent.get("arm_target_q_rad"), 4) is None
        ):
            return False, "invalid_raw_intent"
        planning = payload.get("planning") or {}
        if (
            planning.get("name") not in {"off", "jerk_limited", "one_euro"}
            or planning.get("profile") != planning.get("name")
            or not isinstance(planning.get("valid_for_dispatch"), bool)
            or _finite_vector([planning.get("processing_time_ms")], 1) is None
        ):
            return False, "invalid_planning"
    if schema_version < 4:
        try:
            skeleton = np.asarray(payload.get("hand_skeleton_w"), dtype=float)
        except (TypeError, ValueError):
            return False, "invalid_hand_skeleton"
        return (True, "") if skeleton.shape == (21, 3) and np.all(np.isfinite(skeleton)) else (False, "invalid_hand_skeleton")

    if payload.get("hand_source") not in {"pico_hand", "data_glove", "preset_hand"}:
        return False, "invalid_hand_source"
    if _finite_vector(payload.get("hand_position_units"), 6) is None:
        return False, "invalid_hand_position_units"
    try:
        skeleton_c = np.asarray(payload.get("hand_skeleton_C"), dtype=float)
    except (TypeError, ValueError):
        return False, "invalid_hand_skeleton"
    model_sha = str((payload.get("calibrations") or {}).get("wa100_kinematics_sha256") or "")
    if (
        skeleton_c.shape != (21, 3)
        or not np.all(np.isfinite(skeleton_c))
        or not 0 <= int(validity.get("hand_dof_mask", -1)) <= HAND_ALL_VALID_MASK
        or len(model_sha) != 64
        or any(character not in "0123456789abcdef" for character in model_sha)
        or "hand_pose_C" in payload
        or "hand_skeleton_w" in payload
        or "hand" in (payload.get("raw_intent") or {})
    ):
        return False, "invalid_wa100_hand_state"
    gestures = payload.get("gestures") or {}
    for key in ("thumb_index_pinch", "thumb_middle_pinch"):
        state = gestures.get(key) or {}
        if (
            not isinstance(state.get("active"), bool)
            or _finite_vector([state.get("normalized_distance"), state.get("confidence")], 2) is None
            or float(state.get("normalized_distance")) < 0.0
            or not 0.0 <= float(state.get("confidence")) <= 1.0
        ):
            return False, "invalid_canonical_gestures"
    return True, ""


class WA100PinchDetector:
    """Hysteretic pinch states derived only from WA100 FK feature points."""

    def __init__(self, enter_threshold: float = 0.55, release_threshold: float = 0.70,
                 confirm_enter_sec: float = 0.08, confirm_release_sec: float = 0.04):
        if not all(math.isfinite(v) and v > 0 for v in (confirm_enter_sec, confirm_release_sec)):
            raise ValueError("gesture confirmation durations must be positive finite values")
        self.confirm_enter_sec = confirm_enter_sec
        self.confirm_release_sec = confirm_release_sec
        self.enter_threshold = float(enter_threshold)
        self.release_threshold = float(release_threshold)
        self.states = {"thumb_index_pinch": False, "thumb_middle_pinch": False}
        self.pending = {}
        self.last_time = None
        self.active_anchor = "none"

    def reset(self) -> None:
        self.states = {"thumb_index_pinch": False, "thumb_middle_pinch": False}
        self.pending.clear()
        self.last_time = None
        self.active_anchor = "none"

    def update(self, skeleton: Any, now: Optional[float] = None) -> Dict[str, Any]:
        if now is not None:
            if self.last_time is not None and (now < self.last_time or now - self.last_time > 0.25):
                self.reset()
            self.last_time = now
        points = np.asarray(skeleton, dtype=float).reshape(21, 3)
        palm_width = max(float(np.linalg.norm(points[5] - points[17])), 1e-9)
        result: Dict[str, Any] = {
            "active_anchor": "none",
            "enter_threshold": self.enter_threshold,
            "release_threshold": self.release_threshold,
        }
        for key, tip in (("thumb_index_pinch", 8), ("thumb_middle_pinch", 12)):
            distance = float(np.linalg.norm(points[4] - points[tip])) / palm_width
            active = self.states[key]
            desired = distance <= (self.release_threshold if active else self.enter_threshold)
            if now is None:
                active = desired
            elif desired == active:
                self.pending.pop(key, None)
            else:
                since = self.pending.setdefault(key, now)
                if now - since + 1e-9 >= (self.confirm_enter_sec if desired else self.confirm_release_sec):
                    active = desired
                    self.pending.pop(key, None)
            self.states[key] = active
            result[key] = {
                "active": active,
                "normalized_distance": distance,
                "confidence": float(np.clip(1.0 - distance / self.release_threshold, 0.0, 1.0)),
            }
        if result["thumb_index_pinch"]["active"]:
            result["active_anchor"] = "thumb_index"
        elif result["thumb_middle_pinch"]["active"]:
            result["active_anchor"] = "thumb_middle"
        if now is not None and self.active_anchor != "none":
            if result[self.active_anchor + "_pinch"]["active"]:
                result["active_anchor"] = self.active_anchor
        self.active_anchor = result["active_anchor"]
        return result


class CanonicalGoalBuilder:
    """Thread-safe aggregation of one wrist adapter and one hand adapter."""

    def __init__(self, kinematics_model_path: Path | str, stale_timeout_sec: float = 0.5,
                 planner_config: Optional[CanonicalPlannerConfig | Dict[str, Any]] = None):
        self.lock = threading.RLock()
        self.hand_kinematics = WA100Kinematics.load(kinematics_model_path)
        self.schema_version = CANONICAL_SCHEMA_VERSION
        gesture_config = planner_config if isinstance(planner_config, dict) else {}
        self.pinch_detector = WA100PinchDetector(
            confirm_enter_sec=float(gesture_config.get("vr_gesture_enter_sec", 0.08)),
            confirm_release_sec=float(gesture_config.get("vr_gesture_release_sec", 0.04)))
        if isinstance(planner_config, CanonicalPlannerConfig):
            resolved_planner_config = planner_config
        else:
            resolved_planner_config = CanonicalPlannerConfig.from_mapping(planner_config)
        self.wrist_planner = CanonicalWristPlanner(resolved_planner_config)
        self.stale_timeout_ns = max(1, int(float(stale_timeout_sec) * 1e9))
        self.output_seq = 0
        self.wrist: Optional[Dict[str, Any]] = None
        self.hand: Optional[Dict[str, Any]] = None
        self.last_goal: Optional[Dict[str, Any]] = None
        self.last_emitted_wrist_key: Optional[tuple[str, int]] = None
        self.last_error = "awaiting wrist and hand adapters"

    def observe_wrist_adapter(self, payload: Dict[str, Any], receive_utc_ns: Optional[int] = None) -> tuple[bool, str]:
        if not isinstance(payload, dict) or payload.get("type") != "canonical_wrist_adapter":
            return False, "invalid_type"
        if int(payload.get("schema_version", 0)) != 1:
            return False, "invalid_schema"
        delta = _finite_vector(payload.get("adapter_wrist_delta"), 6)
        pose = payload.get("wrist_pose_C") or {}
        position = _finite_vector(pose.get("position_m"), 3)
        orientation = normalize_quaternion_xyzw(pose.get("orientation_xyzw"))
        target_q = _finite_vector(payload.get("arm_target_q_rad"), 4)
        source = str(payload.get("wrist_source") or "")
        if source not in {"keyboard", "gamepad", "pico_wrist", "vr_controller"}:
            return False, "unsupported_wrist_source"
        if delta is None or position is None or orientation is None or target_q is None:
            return False, "invalid_wrist_adapter"
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        with self.lock:
            session = int(payload.get("tracking_session_id", 0))
            if (self.wrist and source == "pico_wrist" and
                    session != self.wrist.get("tracking_session_id", 0)):
                self.wrist_planner.reset_tracking_reference()
            self.wrist = {
                "tracking_session_id": session,
                "source": source,
                "source_seq": int(payload.get("source_seq", -1)),
                "source_time_ns": int(payload.get("source_time_ns") or receive_utc_ns),
                "receive_utc_ns": receive_utc_ns,
                "delta": delta.tolist(),
                "pose": {"position_m": position.tolist(), "orientation_xyzw": orientation.tolist()},
                "target_q": target_q.tolist(),
                "mapping_id": str(payload.get("mapping_id") or "ui-arm-mapping-v1"),
                "wrist_dof_mask": int(payload.get("wrist_dof_mask", WRIST_SUPPORTED_MASK)) & 0x3F,
                "no_send": bool(payload.get("no_send", False)),
            }
        return True, ""

    def observe_hand_input(
        self, source: str, value: Dict[str, Any], receive_utc_ns: Optional[int] = None
    ) -> tuple[bool, str]:
        source = str(source)
        if source not in {"pico_hand", "data_glove", "preset_hand"}:
            return False, "unsupported_hand_source"
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        started_ns = time.perf_counter_ns()
        targets = value.get("mapped_target_units")
        if source == "preset_hand" and targets is None:
            targets = value.get("positions")
        target_values = _finite_vector(targets, 6)
        if target_values is None or not np.all((target_values >= 0.0) & (target_values <= 2000.0)):
            return False, "hand_position_units_must_be_six_values_in_0_2000"
        try:
            skeleton = self.hand_kinematics.canonical_skeleton_21(target_values)
        except (TypeError, ValueError) as exc:
            return False, str(exc)
        calibration_id = str(value.get("glove_calibration_id") or value.get("calibration_id") or "")
        with self.lock:
            frame_id = int(value.get("frameId", value.get("frame_id", -1)))
            previous = self.hand
            generation = (value.get("xrSkeletonFilter") or {}).get("right_generation", 0)
            same_stream = (previous and previous["source"] == source and previous["calibration_id"] == calibration_id
                           and previous.get("tracking_generation", 0) == generation)
            if not same_stream or (previous and receive_utc_ns - previous["receive_utc_ns"] > 250_000_000):
                self.pinch_detector.reset()
            elif source == "pico_hand" and frame_id >= 0 and frame_id <= previous["source_seq"]:
                return False, "duplicate_or_out_of_order_hand_frame"
            gestures = self.pinch_detector.update(skeleton, receive_utc_ns / 1e9 if source == "pico_hand" else None)
            self.hand = {
                "tracking_generation": generation,
                "vr_anchor_stability": dict(value.get("vr_anchor_stability") or {}) if source == "pico_hand" else {},
                "source": source,
                "source_seq": int(value.get("frameId", value.get("frame_id", -1))),
                "source_time_ns": int(value.get("source_timestamp_ns") or value.get("receiver_wall_time_ns") or receive_utc_ns),
                "receive_utc_ns": receive_utc_ns,
                "skeleton": skeleton.tolist(),
                "target_units": target_values.tolist(),
                "calibration_id": calibration_id,
                "invalid_reasons": [],
                "gestures": gestures,
                "processing_time_ms": (time.perf_counter_ns() - started_ns) / 1e6,
            }
        return True, ""

    def build(self, arm_mode: str, hand_mode: str, now_ns: Optional[int] = None) -> Optional[Dict[str, Any]]:
        now_ns = int(now_ns or time.time_ns())
        with self.lock:
            wrist = dict(self.wrist or {})
            hand = dict(self.hand or {})
            if not wrist or not hand:
                self.last_error = "awaiting wrist and hand adapters"
                return None
            wrist_age = max(0, now_ns - int(wrist["receive_utc_ns"]))
            hand_age = max(0, now_ns - int(hand["receive_utc_ns"]))
            invalid_reasons = list(hand.get("invalid_reasons") or [])
            wrist_mask = int(wrist.get("wrist_dof_mask", 0))
            hand_mask = HAND_ALL_VALID_MASK
            if wrist_age > self.stale_timeout_ns:
                wrist_mask = 0
                invalid_reasons.append("wrist_stale")
            if hand_age > self.stale_timeout_ns and hand.get("source") != "preset_hand":
                hand_mask = 0
                invalid_reasons.append("hand_stale")
            wrist_key = (str(wrist["source"]), int(wrist["source_seq"]))
            raw_output_delta = (
                list(wrist["delta"])
                if wrist_key != self.last_emitted_wrist_key
                else [0.0] * 6
            )
            planned_pose, output_delta, planning = self.wrist_planner.step(
                source=str(wrist["source"]),
                target_pose=wrist["pose"],
                source_receive_ns=int(wrist["receive_utc_ns"]),
                now_ns=now_ns,
                valid=wrist_mask != 0,
            )
            if (
                planning["name"] == "off"
                and planning.get("valid_for_dispatch")
                and planning.get("reset_reason") != "source_or_profile_changed"
            ):
                output_delta = raw_output_delta
            if planning.get("hold_expired"):
                wrist_mask = 0
                invalid_reasons.append("planner_hold_expired")
            self.output_seq += 1
            condition = E1_CONDITIONS.get((str(arm_mode), str(hand_mode)), "NON_E1")
            goal = {
                "type": "canonical_goal",
                "schema_version": self.schema_version,
                "seq": self.output_seq,
                "source_time_ns": max(int(wrist["source_time_ns"]), int(hand["source_time_ns"])),
                "receive_utc_ns": now_ns,
                "condition_id": condition,
                "no_send": bool(wrist.get("no_send", False)),
                "wrist_source": wrist["source"],
                "hand_source": hand["source"],
                "source_state": {
                    "wrist_seq": int(wrist["source_seq"]),
                    "hand_seq": int(hand["source_seq"]),
                    "wrist_age_sec": wrist_age / 1e9,
                    "hand_age_sec": hand_age / 1e9,
                },
                "adapter_wrist_delta": list(output_delta),
                "raw_intent": {
                    "adapter_wrist_delta": raw_output_delta,
                    "wrist_pose_C": dict(wrist["pose"]),
                    "arm_target_q_rad": list(wrist["target_q"]),
                },
                "planning": planning,
                "composition": {
                    "translation": "canonical_world_left",
                    "rotation": "wrist_body_right",
                    "rotation_parameterization": "rotvec_rad_xyz",
                },
                "wrist_pose_C": planned_pose,
                "hand_position_units": list(hand["target_units"]),
                "hand_skeleton_C": list(hand["skeleton"]),
                "hand_point_names": list(CANONICAL_JOINT_NAMES),
                "hand_channel_names": list(WA100_HAND_CHANNEL_NAMES),
                "validity": {"wrist_dof_mask": wrist_mask, "hand_dof_mask": hand_mask},
                "calibrations": {
                    "wrist_mapping": wrist["mapping_id"],
                    "wa100_kinematics": self.hand_kinematics.calibration_id,
                    "wa100_kinematics_sha256": self.hand_kinematics.content_sha256,
                    "hand_mapping": hand.get("calibration_id", ""),
                },
                "invalid_reasons": sorted(set(invalid_reasons)),
                "final_targets": {
                    "arm_target_q_rad": list(wrist["target_q"]),
                    "hand_target_units": list(hand.get("target_units") or []),
                },
                "gestures": json.loads(json.dumps(hand.get("gestures") or {})),
                "hand_processing": {
                    "vr_anchor_stability": dict(hand.get("vr_anchor_stability") or {}),
                    "processing_time_ms": float(hand.get("processing_time_ms") or 0.0),
                },
            }
            if hand_mask == 0:
                goal["gestures"]["active_anchor"] = "none"
                for key in ("thumb_index_pinch", "thumb_middle_pinch"):
                    if isinstance(goal["gestures"].get(key), dict):
                        goal["gestures"][key]["active"] = False
                        goal["gestures"][key]["confidence"] = 0.0
            valid, reason = validate_canonical_goal(goal)
            if not valid:
                self.last_error = reason
                return None
            self.last_goal = goal
            self.last_emitted_wrist_key = wrist_key
            self.last_error = ",".join(goal["invalid_reasons"])
            return json.loads(json.dumps(goal))

    def status(self) -> Dict[str, Any]:
        with self.lock:
            goal = self.last_goal or {}
            return {
                "enabled": True,
                "seq": int(goal.get("seq", 0)),
                "condition_id": goal.get("condition_id", ""),
                "no_send": bool(goal.get("no_send", False)),
                "wrist_source": goal.get("wrist_source", ""),
                "hand_source": goal.get("hand_source", ""),
                "validity": dict(goal.get("validity") or {}),
                "calibrations": dict(goal.get("calibrations") or {}),
                "planning": dict(goal.get("planning") or self.wrist_planner.status()),
                "hand_position_units": list(goal.get("hand_position_units") or []),
                "gestures": dict(goal.get("gestures") or {}),
                "hand_processing": dict(goal.get("hand_processing") or {}),
                "last_error": self.last_error,
            }

    def set_planner_override(self, profile: Optional[str]) -> str:
        with self.lock:
            return self.wrist_planner.set_override(profile)
