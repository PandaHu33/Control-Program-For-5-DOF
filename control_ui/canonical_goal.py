"""Canonical master-side arm/hand semantics used by the E1 experiment.

The module deliberately stops at the nominal command boundary.  It does not
contain contact inference, admittance, or any E2 closed-loop behaviour.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import numpy as np

try:
    from .wa100_kinematics import WA100Kinematics
except ImportError:
    from wa100_kinematics import WA100Kinematics


CANONICAL_SCHEMA_VERSION = 1
WRIST_DOF_POSITION_X = 1 << 0
WRIST_DOF_POSITION_Y = 1 << 1
WRIST_DOF_POSITION_Z = 1 << 2
WRIST_DOF_ROLL = 1 << 3
WRIST_SUPPORTED_MASK = (
    WRIST_DOF_POSITION_X | WRIST_DOF_POSITION_Y | WRIST_DOF_POSITION_Z | WRIST_DOF_ROLL
)
HAND_ALL_VALID_MASK = (1 << 21) - 1

CANONICAL_JOINT_NAMES = (
    "wrist",
    "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip",
    "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
)
CANONICAL_PARENTS = (-1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19)
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


class FrozenHi5HandModel:
    """Frozen kinematic model converting Hi5 local rotations into 21 points."""

    def __init__(self, payload: Dict[str, Any], source_path: Optional[Path] = None):
        if int(payload.get("schema_version", 0)) != 1:
            raise ValueError("unsupported Hi5 hand model schema")
        offsets = np.asarray(payload.get("offsets_w_m"), dtype=float)
        if offsets.shape != (21, 3) or not np.all(np.isfinite(offsets)):
            raise ValueError("Hi5 hand model must contain finite offsets_w_m[21][3]")
        parents = tuple(int(value) for value in payload.get("parents", []))
        if parents != CANONICAL_PARENTS:
            raise ValueError("Hi5 hand model parent order does not match canonical order")
        source_indices = payload.get("source_rotation_indices")
        if not isinstance(source_indices, list) or len(source_indices) != 21:
            raise ValueError("Hi5 source_rotation_indices must contain 21 entries")
        self.offsets = offsets
        self.source_rotation_indices = tuple(None if value is None else int(value) for value in source_indices)
        self.euler_order = str(payload.get("euler_order", "zyx")).lower()
        neutral = np.asarray(payload.get("neutral_source_rotations_deg", np.zeros((21, 3))), dtype=float)
        if neutral.shape != (21, 3) or not np.all(np.isfinite(neutral)):
            raise ValueError("Hi5 neutral_source_rotations_deg must contain finite [21][3] values")
        self.neutral_source_rotations_deg = neutral
        signs = np.asarray(payload.get("source_rotation_signs", np.ones((21, 3))), dtype=float)
        if signs.shape != (21, 3) or not np.all(np.isin(signs, (-1.0, 1.0))):
            raise ValueError("Hi5 source_rotation_signs must contain only +/-1 [21][3] values")
        self.source_rotation_signs = signs
        self.canonical_scale = float(payload.get("canonical_scale", 1.0))
        if not math.isfinite(self.canonical_scale) or self.canonical_scale <= 0.0:
            raise ValueError("Hi5 canonical_scale must be finite and positive")
        thumb_mapping = payload.get("thumb_semantic_mapping")
        self.thumb_semantic_mapping = None
        if thumb_mapping is not None:
            if not isinstance(thumb_mapping, dict):
                raise ValueError("Hi5 thumb_semantic_mapping must be an object")
            pitch_weights = np.asarray(thumb_mapping.get("pitch_joint_weights"), dtype=float)
            yaw_weights = np.asarray(thumb_mapping.get("yaw_joint_weights"), dtype=float)
            if pitch_weights.shape != (3,) or yaw_weights.shape != (3,) or not all(
                np.all(np.isfinite(value)) for value in (pitch_weights, yaw_weights)
            ):
                raise ValueError("Hi5 thumb semantic joint weights must contain three finite values")
            open_target = float(thumb_mapping.get("open_target", 2000.0))
            close_target = float(thumb_mapping.get("close_target", 0.0))
            if not all(math.isfinite(value) for value in (open_target, close_target)) or abs(open_target - close_target) < 1e-9:
                raise ValueError("Hi5 thumb semantic target range is invalid")
            self.thumb_semantic_mapping = {
                "pitch_target_index": int(thumb_mapping.get("pitch_target_index", 0)),
                "yaw_target_index": int(thumb_mapping.get("yaw_target_index", 1)),
                "open_target": open_target,
                "close_target": close_target,
                "pitch_max_deg": float(thumb_mapping.get("pitch_max_deg", 0.0)),
                "yaw_max_deg": float(thumb_mapping.get("yaw_max_deg", 0.0)),
                "pitch_joint_weights": pitch_weights,
                "yaw_joint_weights": yaw_weights,
            }
        self.wa100_thumb_kinematics = None
        self.wa100_thumb_rotation = None
        self.wa100_thumb_translation = None
        wa100_thumb = payload.get("wa100_thumb_reference")
        if wa100_thumb is not None:
            if not isinstance(wa100_thumb, dict) or source_path is None:
                raise ValueError("Hi5 wa100_thumb_reference requires a file-backed model")
            reference_path = (Path(source_path).resolve().parent / str(wa100_thumb.get("kinematics_model_path"))).resolve()
            expected_hash = str(wa100_thumb.get("kinematics_model_sha256") or "").lower()
            actual_hash = hashlib.sha256(reference_path.read_bytes()).hexdigest()
            if expected_hash and actual_hash != expected_hash:
                raise ValueError("WA100 thumb reference model sha256 mismatch")
            kinematics = WA100Kinematics.load(reference_path)
            expected_calibration = str(wa100_thumb.get("calibration_id") or "")
            actual_calibration = str(kinematics.model["calibration"].get("calibration_id") or "")
            if expected_calibration and actual_calibration != expected_calibration:
                raise ValueError("WA100 thumb reference calibration_id mismatch")
            rotation = np.asarray(wa100_thumb.get("canonical_from_wa100_rotation"), dtype=float)
            translation = np.asarray(wa100_thumb.get("canonical_translation_m", [0.0, 0.0, 0.0]), dtype=float)
            if rotation.shape != (3, 3) or translation.shape != (3,) or not all(
                np.all(np.isfinite(value)) for value in (rotation, translation)
            ):
                raise ValueError("WA100 thumb Canonical registration is invalid")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9) or not math.isclose(
                float(np.linalg.det(rotation)), 1.0, abs_tol=1e-9
            ):
                raise ValueError("WA100 thumb Canonical registration must be a proper rotation")
            self.wa100_thumb_kinematics = kinematics
            self.wa100_thumb_rotation = rotation
            self.wa100_thumb_translation = translation
        self.calibration_id = str(payload.get("calibration_id") or "")
        if not self.calibration_id:
            raise ValueError("Hi5 hand model calibration_id is required")
        canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
        self.content_sha256 = hashlib.sha256(canonical).hexdigest()
        expected = str(payload.get("content_sha256") or "").strip().lower()
        if expected and expected != self.content_sha256:
            # The hash field itself cannot be included in its own digest.  Accept
            # the conventional digest computed after removing that field.
            unhashed = dict(payload)
            unhashed.pop("content_sha256", None)
            raw = json.dumps(unhashed, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
            self.content_sha256 = hashlib.sha256(raw).hexdigest()
            if expected != self.content_sha256:
                raise ValueError("Hi5 hand model content_sha256 mismatch")
        self.source_path = source_path

    @classmethod
    def load(cls, path: Path | str) -> "FrozenHi5HandModel":
        source = Path(path).resolve()
        return cls(json.loads(source.read_text(encoding="utf-8")), source)

    def skeleton(self, rotations: Any, mapped_target_units: Any = None) -> np.ndarray:
        values = np.asarray(rotations, dtype=float).reshape(21, 3)
        if not np.all(np.isfinite(values)):
            raise ValueError("Hi5 rotations contain non-finite values")
        positions = np.zeros((21, 3), dtype=float)
        orientations = np.zeros((21, 4), dtype=float)
        orientations[:, 3] = 1.0
        thumb_locals = None
        if self.thumb_semantic_mapping is not None and mapped_target_units is not None:
            targets = np.asarray(mapped_target_units, dtype=float).reshape(-1)
            mapping = self.thumb_semantic_mapping
            required = max(mapping["pitch_target_index"], mapping["yaw_target_index"])
            if targets.size <= required or not np.all(np.isfinite(targets)):
                raise ValueError("Hi5 mapped_target_units cannot drive semantic thumb mapping")
            denominator = mapping["close_target"] - mapping["open_target"]
            pitch_ratio = float(np.clip(
                (targets[mapping["pitch_target_index"]] - mapping["open_target"]) / denominator, 0.0, 1.0
            ))
            yaw_ratio = float(np.clip(
                (targets[mapping["yaw_target_index"]] - mapping["open_target"]) / denominator, 0.0, 1.0
            ))
            pitch = pitch_ratio * mapping["pitch_max_deg"] * mapping["pitch_joint_weights"]
            yaw = yaw_ratio * mapping["yaw_max_deg"] * mapping["yaw_joint_weights"]
            thumb_locals = [
                euler_deg_to_quaternion_xyzw([0.0, pitch[index], yaw[index]], self.euler_order)
                for index in range(3)
            ]
        for joint in range(1, 21):
            parent = CANONICAL_PARENTS[joint]
            positions[joint] = positions[parent] + quaternion_rotate_xyzw(
                orientations[parent], self.offsets[joint]
            )
            source_index = self.source_rotation_indices[joint]
            if thumb_locals is not None and joint in (1, 2, 3):
                local = thumb_locals[joint - 1]
            elif source_index is None:
                local = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
            else:
                relative_deg = (
                    values[source_index] - self.neutral_source_rotations_deg[source_index] + 180.0
                ) % 360.0 - 180.0
                relative_deg *= self.source_rotation_signs[source_index]
                local = euler_deg_to_quaternion_xyzw(relative_deg, self.euler_order)
            orientations[joint] = normalize_quaternion_xyzw(
                quaternion_multiply_xyzw(orientations[parent], local)
            )
        if self.wa100_thumb_kinematics is not None and mapped_target_units is not None:
            targets = np.asarray(mapped_target_units, dtype=float).reshape(-1)
            if targets.shape != (6,) or not np.all(np.isfinite(targets)):
                raise ValueError("Hi5 mapped_target_units cannot drive WA100 thumb reference")
            thumb_wa100 = self.wa100_thumb_kinematics.thumb_feature_points_from_motor_units(targets)
            positions[1:5] = (
                self.wa100_thumb_rotation @ thumb_wa100.T
            ).T + self.wa100_thumb_translation
        # Scale about the wrist origin, preserving the Canonical wrist pose.
        return positions * self.canonical_scale


def pico_skeleton_in_wrist_frame(positions: Any) -> np.ndarray:
    points = np.asarray(positions, dtype=float).reshape(21, 3)
    if not np.all(np.isfinite(points)):
        raise ValueError("PICO hand positions contain non-finite values")
    wrist = points[0]
    x_axis = points[5] - wrist
    middle = points[9] - wrist
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm <= 1e-9:
        raise ValueError("PICO palm x axis is degenerate")
    x_axis /= x_norm
    # PICO's right-hand stream uses the opposite tracked palm normal from the
    # Hi5/WA100 registration.  This cross order is the empirically verified
    # palm-down display for the recorded PICO right hand.
    z_axis = np.cross(x_axis, middle)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm <= 1e-9:
        raise ValueError("PICO palm plane is degenerate")
    z_axis /= z_norm
    y_axis = np.cross(z_axis, x_axis)
    rotation_world_from_wrist = np.column_stack((x_axis, y_axis, z_axis))
    local = (rotation_world_from_wrist.T @ (points - wrist).T).T
    # The PICO right-hand joint stream is laterally mirrored relative to the
    # operator-facing Canonical display.  Mirror only left/right so the thumb
    # is shown on the left while preserving the empirically verified
    # palm-down normal above.
    local[:, 1] *= -1.0
    return local


def validate_canonical_goal(payload: Any) -> tuple[bool, str]:
    if not isinstance(payload, dict) or payload.get("type") != "canonical_goal":
        return False, "invalid_type"
    if int(payload.get("schema_version", 0)) != CANONICAL_SCHEMA_VERSION:
        return False, "invalid_schema"
    if _finite_vector(payload.get("adapter_wrist_delta"), 6) is None:
        return False, "invalid_adapter_wrist_delta"
    pose = payload.get("wrist_pose_C") or {}
    if _finite_vector(pose.get("position_m"), 3) is None or normalize_quaternion_xyzw(pose.get("orientation_xyzw")) is None:
        return False, "invalid_wrist_pose"
    try:
        skeleton = np.asarray(payload.get("hand_skeleton_w"), dtype=float)
    except (TypeError, ValueError):
        return False, "invalid_hand_skeleton"
    if skeleton.shape != (21, 3) or not np.all(np.isfinite(skeleton)):
        return False, "invalid_hand_skeleton"
    validity = payload.get("validity") or {}
    if not isinstance(validity.get("wrist_dof_mask"), int) or not isinstance(validity.get("hand_node_mask"), int):
        return False, "invalid_validity_masks"
    return True, ""


class CanonicalGoalBuilder:
    """Thread-safe aggregation of one wrist adapter and one hand adapter."""

    def __init__(self, hand_model_path: Path | str, stale_timeout_sec: float = 0.5):
        self.lock = threading.RLock()
        self.hand_model = FrozenHi5HandModel.load(hand_model_path)
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
            self.wrist = {
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
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        reasons = []
        try:
            if source == "pico_hand":
                skeleton = pico_skeleton_in_wrist_frame(value.get("rightPositions"))
                calibration_id = str(value.get("calibration_id") or "pico-xr-hands-direct-v1")
            elif source == "data_glove":
                supplied_id = str(value.get("hand_model_calibration_id") or self.hand_model.calibration_id)
                if supplied_id != self.hand_model.calibration_id:
                    return False, "hand_model_calibration_id_mismatch"
                skeleton = self.hand_model.skeleton(
                    value.get("rightRotations"), value.get("mapped_target_units")
                )
                calibration_id = str(value.get("glove_calibration_id") or value.get("calibration_id") or "")
                if not calibration_id:
                    reasons.append("missing_glove_calibration_id")
            else:
                return False, "unsupported_hand_source"
        except (TypeError, ValueError) as exc:
            return False, str(exc)
        targets = value.get("mapped_target_units")
        target_values = _finite_vector(targets, 6)
        if target_values is not None and not np.all((target_values >= 0.0) & (target_values <= 2000.0)):
            target_values = None
        if target_values is None:
            reasons.append("missing_legacy_compatible_hand_targets")
        with self.lock:
            self.hand = {
                "source": source,
                "source_seq": int(value.get("frameId", value.get("frame_id", -1))),
                "source_time_ns": int(value.get("source_timestamp_ns") or value.get("receiver_wall_time_ns") or receive_utc_ns),
                "receive_utc_ns": receive_utc_ns,
                "skeleton": skeleton.tolist(),
                "target_units": target_values.tolist() if target_values is not None else [],
                "calibration_id": calibration_id,
                "invalid_reasons": reasons,
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
            if hand_age > self.stale_timeout_ns:
                hand_mask = 0
                invalid_reasons.append("hand_stale")
            if not hand.get("target_units"):
                hand_mask = 0
            wrist_key = (str(wrist["source"]), int(wrist["source_seq"]))
            output_delta = (
                list(wrist["delta"])
                if wrist_key != self.last_emitted_wrist_key
                else [0.0] * 6
            )
            self.output_seq += 1
            condition = E1_CONDITIONS.get((str(arm_mode), str(hand_mode)), "NON_E1")
            goal = {
                "type": "canonical_goal",
                "schema_version": CANONICAL_SCHEMA_VERSION,
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
                "adapter_wrist_delta": output_delta,
                "composition": {
                    "translation": "canonical_world_left",
                    "rotation": "wrist_body_right",
                    "rotation_parameterization": "rotvec_rad_xyz",
                },
                "wrist_pose_C": dict(wrist["pose"]),
                "hand_skeleton_w": list(hand["skeleton"]),
                "joint_names": list(CANONICAL_JOINT_NAMES),
                "validity": {"wrist_dof_mask": wrist_mask, "hand_node_mask": hand_mask},
                "calibrations": {
                    "wrist_mapping": wrist["mapping_id"],
                    "hand_model": self.hand_model.calibration_id,
                    "hand_model_sha256": self.hand_model.content_sha256,
                    "hand_mapping": hand.get("calibration_id", ""),
                },
                "invalid_reasons": sorted(set(invalid_reasons)),
                "final_targets": {
                    "arm_target_q_rad": list(wrist["target_q"]),
                    "hand_target_units": list(hand.get("target_units") or []),
                },
            }
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
                "last_error": self.last_error,
            }
