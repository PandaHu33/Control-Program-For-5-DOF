"""Right-controller/Hi5 wrist fusion for the master-side teleoperation input.

The controller remains the only source of translation and control authority.
Hi5 ``Hand`` (bone 1) only contributes short-term orientation increments.
"""

from __future__ import annotations

import copy
import json
import math
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, Optional, Sequence

import numpy as np


HI5_BONE_COUNT = 21
HI5_FOREARM_INDEX = 0
HI5_HAND_INDEX = 1
HI5_LOCAL_FINGER_START = 2


def normalize_quaternion_xyzw(value: Any) -> Optional[np.ndarray]:
    try:
        quaternion = np.asarray(value, dtype=float).reshape(-1)[:4]
    except (TypeError, ValueError):
        return None
    if quaternion.size != 4 or not np.all(np.isfinite(quaternion)):
        return None
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > 1e-9 else None


def quaternion_multiply_xyzw(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    lx, ly, lz, lw = np.asarray(left, dtype=float)
    rx, ry, rz, rw = np.asarray(right, dtype=float)
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=float,
    )


def quaternion_conjugate_xyzw(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = np.asarray(value, dtype=float)
    return np.array([-x, -y, -z, w], dtype=float)


def quaternion_slerp_xyzw(left: Sequence[float], right: Sequence[float], ratio: float) -> np.ndarray:
    first = normalize_quaternion_xyzw(left)
    second = normalize_quaternion_xyzw(right)
    if first is None or second is None:
        raise ValueError("invalid quaternion")
    amount = max(0.0, min(1.0, float(ratio)))
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second, dot = -second, -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        result = first + amount * (second - first)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    scale = math.sin(angle)
    return (
        math.sin((1.0 - amount) * angle) / scale * first
        + math.sin(amount * angle) / scale * second
    )


def quaternion_angle_deg_xyzw(left: Sequence[float], right: Sequence[float]) -> float:
    first = normalize_quaternion_xyzw(left)
    second = normalize_quaternion_xyzw(right)
    if first is None or second is None:
        return float("inf")
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(float(np.dot(first, second)))))))


def euler_deg_to_quaternion_xyzw(euler: Sequence[float], order: str = "zyx") -> np.ndarray:
    values = dict(zip("xyz", np.asarray(euler, dtype=float)))
    result = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    for axis in str(order).lower():
        half = math.radians(float(values[axis])) * 0.5
        current = np.array([0.0, 0.0, 0.0, math.cos(half)], dtype=float)
        current["xyz".index(axis)] = math.sin(half)
        result = quaternion_multiply_xyzw(result, current)
    normalized = normalize_quaternion_xyzw(result)
    if normalized is None:
        raise ValueError("invalid Euler rotation")
    return normalized


def wxyz_to_xyzw(value: Sequence[float]) -> np.ndarray:
    w, x, y, z = np.asarray(value, dtype=float)
    result = normalize_quaternion_xyzw([x, y, z, w])
    if result is None:
        raise ValueError("invalid stored quaternion")
    return result


class GloveFrameReceiver:
    """Receive the versioned, read-only Hi5 stream emitted by the WA100 bridge."""

    def __init__(self, host: str, port: int, stopped: threading.Event):
        self.host, self.port, self.stopped = str(host), int(port), stopped
        self.lock = threading.RLock()
        self.latest_frame: Dict[str, Any] = {}
        self.received_at = 0.0
        self.last_error = f"waiting for Hi5 Hand frames on {self.host}:{self.port}"
        self._socket: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._run, name="GloveFrameReceiver", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._thread.join(timeout=1.0)

    def snapshot(self) -> tuple[Dict[str, Any], Optional[float]]:
        with self.lock:
            frame, received_at = copy.deepcopy(self.latest_frame), self.received_at
        age = time.monotonic() - received_at if received_at > 0.0 else None
        return frame, age

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.settimeout(0.5)
            while not self.stopped.is_set():
                try:
                    raw, _source = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    payload = json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if (
                    payload.get("type") != "glove_sensor_frame"
                    or int(payload.get("schema_version", 0)) < 2
                    or int(payload.get("hand_index", -1)) != HI5_HAND_INDEX
                ):
                    continue
                rotations = payload.get("rightRotations")
                if not isinstance(rotations, list) or len(rotations) < HI5_BONE_COUNT * 3:
                    continue
                try:
                    values = np.asarray(rotations[: HI5_BONE_COUNT * 3], dtype=float)
                except (TypeError, ValueError):
                    continue
                if not np.all(np.isfinite(values)):
                    continue
                with self.lock:
                    self.latest_frame = payload
                    self.received_at = time.monotonic()
                    self.last_error = f"Hi5 frame {payload.get('frameId', -1)}"
        finally:
            try:
                sock.close()
            except OSError:
                pass


class MasterWristFusion:
    """SO(3) complementary fusion with controller-only failover."""

    def __init__(self, settings: Dict[str, Any], config_dir: Path):
        self.settings = settings or {}
        self.mode_requested = str(self.settings.get("mode", "m3")).lower()
        if self.mode_requested not in {"m1", "m2", "m3"}:
            self.mode_requested = "m3"
        self.euler_order = str(self.settings.get("glove_euler_order", "zyx")).lower()
        self.glove_stale_sec = max(0.01, float(self.settings.get("glove_stale_timeout_sec", 0.15)))
        self.controller_stale_sec = max(0.01, float(self.settings.get("controller_stale_timeout_sec", 0.40)))
        self.correction_tau_sec = max(0.01, float(self.settings.get("controller_correction_tau_sec", 0.20)))
        self.weight_tau_sec = max(0.01, float(self.settings.get("weight_ramp_tau_sec", 0.15)))
        self.innovation_soft_deg = max(0.0, float(self.settings.get("innovation_soft_deg", 10.0)))
        self.innovation_reject_deg = max(
            self.innovation_soft_deg + 1e-6, float(self.settings.get("innovation_reject_deg", 25.0))
        )
        raw_path = str(self.settings.get("alignment_calibration_path", "")).strip()
        path = Path(raw_path)
        if raw_path and not path.is_absolute():
            path = (config_dir / path).resolve()
        self.calibration_path = path if raw_path else None
        self.calibration: Dict[str, Any] = {}
        self.calibration_error = "alignment calibration path is not configured"
        self.calibration_id = ""
        self.controller_time_offset_sec = 0.0
        self.glove_calibration_error = ""
        self.expected_glove_calibration_id = str(
            self.settings.get("glove_calibration_id", "")
        ).strip()
        raw_glove_calibration_path = str(
            self.settings.get("glove_calibration_path", "")
        ).strip()
        glove_calibration_path = Path(raw_glove_calibration_path)
        if not self.expected_glove_calibration_id and raw_glove_calibration_path:
            if not glove_calibration_path.is_absolute():
                glove_calibration_path = (config_dir / glove_calibration_path).resolve()
            try:
                glove_calibration = json.loads(
                    glove_calibration_path.read_text(encoding="utf-8")
                )
                if not bool((glove_calibration.get("quality") or {}).get("quality_pass", False)):
                    raise ValueError("glove mapping quality_pass is false")
                self.expected_glove_calibration_id = str(
                    glove_calibration["calibration_id"]
                )
            except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.glove_calibration_error = str(exc)
        self._controller_to_hand: Optional[np.ndarray] = None
        self._glove_world_from_controller_world: Optional[np.ndarray] = None
        self._load_calibration()
        self.fused: Optional[np.ndarray] = None
        self.previous_aligned_glove: Optional[np.ndarray] = None
        self.last_glove_seq: Optional[int] = None
        self.last_update_monotonic = time.monotonic()
        self.glove_weight = 0.0
        self.output_seq = 0
        self.controller_history = deque(maxlen=128)

    def _load_calibration(self) -> None:
        if self.calibration_path is None:
            return
        try:
            payload = json.loads(self.calibration_path.read_text(encoding="utf-8"))
            quality = payload.get("quality") or {}
            rotation = payload.get("rotation_alignment") or {}
            conventions = payload.get("conventions") or {}
            if int(payload.get("schema_version", 0)) < 2:
                raise ValueError("alignment schema predates explicit Hi5 Hand semantics")
            if int(conventions.get("glove_orientation_index", -1)) != HI5_HAND_INDEX:
                raise ValueError("alignment calibration was not fitted from Hi5 Hand=1")
            if not bool(quality.get("quality_pass", False)):
                raise ValueError("quality_pass is false")
            if rotation.get("mode") != "rigid_hand_eye":
                raise ValueError("rotation mode is not rigid_hand_eye")
            self._controller_to_hand = wxyz_to_xyzw(
                rotation["controller_to_glove_hand_quaternion_wxyz"]
            )
            self._glove_world_from_controller_world = wxyz_to_xyzw(
                rotation["glove_world_from_controller_world_quaternion_wxyz"]
            )
            self.calibration, self.calibration_id = payload, str(payload["calibration_id"])
            self.controller_time_offset_sec = float(
                (payload.get("time_alignment") or {}).get(
                    "controller_time_offset_to_glove_sec", 0.0
                )
            )
            self.calibration_error = ""
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.calibration = {}
            self.calibration_id = ""
            self._controller_to_hand = None
            self._glove_world_from_controller_world = None
            self.calibration_error = str(exc)

    def _aligned_glove(self, frame: Dict[str, Any]) -> Optional[np.ndarray]:
        if self._controller_to_hand is None or self._glove_world_from_controller_world is None:
            return None
        rotations = frame.get("rightRotations")
        try:
            hand_euler = np.asarray(rotations, dtype=float).reshape(-1, 3)[HI5_HAND_INDEX]
            hand = euler_deg_to_quaternion_xyzw(hand_euler, self.euler_order)
        except (TypeError, ValueError, IndexError):
            return None
        return quaternion_multiply_xyzw(
            quaternion_multiply_xyzw(
                quaternion_conjugate_xyzw(self._glove_world_from_controller_world), hand
            ),
            quaternion_conjugate_xyzw(self._controller_to_hand),
        )

    def _paired_controller_quaternion(
        self, controller_pose: Optional[Dict[str, Any]], glove_frame: Dict[str, Any]
    ) -> tuple[Optional[np.ndarray], Optional[float]]:
        if controller_pose is not None:
            sequence = int(controller_pose.get("seq", -1))
            if not self.controller_history or sequence != self.controller_history[-1][0]:
                self.controller_history.append(
                    (
                        sequence,
                        float(controller_pose.get("wall_time", 0.0)),
                        normalize_quaternion_xyzw(controller_pose.get("rotation")),
                    )
                )
        glove_time = float(glove_frame.get("receiver_wall_time_ns", 0)) / 1e9 if glove_frame else 0.0
        candidates = [
            entry for entry in self.controller_history if entry[1] > 0.0 and entry[2] is not None
        ]
        if glove_time <= 0.0 or not candidates:
            return normalize_quaternion_xyzw(
                controller_pose.get("rotation") if controller_pose else None
            ), None
        best = min(
            candidates,
            key=lambda entry: abs(entry[1] + self.controller_time_offset_sec - glove_time),
        )
        return best[2], abs(best[1] + self.controller_time_offset_sec - glove_time)

    def update(
        self,
        controller_pose: Optional[Dict[str, Any]],
        glove_frame: Dict[str, Any],
        glove_age_sec: Optional[float],
    ) -> tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        now, wall_ns = time.monotonic(), time.time_ns()
        dt = max(1e-4, min(0.2, now - self.last_update_monotonic))
        self.last_update_monotonic = now
        controller_age = (
            now - float(controller_pose.get("received_at", 0.0)) if controller_pose is not None else None
        )
        controller_q = normalize_quaternion_xyzw(
            controller_pose.get("rotation") if controller_pose is not None else None
        )
        try:
            controller_position = np.asarray(
                controller_pose.get("position") if controller_pose is not None else [], dtype=float
            ).reshape(-1)
        except (TypeError, ValueError):
            controller_position = np.asarray([], dtype=float)
        controller_position_valid = bool(
            controller_position.size >= 3 and np.all(np.isfinite(controller_position[:3]))
        )
        controller_tracked = bool(controller_pose and controller_pose.get("tracked", False))
        deadman = bool(controller_pose and controller_pose.get("deadman_active", False))
        controller_fresh = bool(
            controller_pose is not None
            and controller_q is not None
            and controller_position_valid
            and controller_age is not None
            and controller_age <= self.controller_stale_sec
        )
        controller_authorized = controller_fresh and controller_tracked and deadman

        glove_valid_flag = bool(glove_frame.get("valid", True)) if glove_frame else False
        glove_fresh = bool(
            glove_frame
            and glove_valid_flag
            and glove_age_sec is not None
            and glove_age_sec <= self.glove_stale_sec
        )
        aligned_glove = self._aligned_glove(glove_frame) if glove_fresh else None
        glove_calibration_id = str(
            glove_frame.get("glove_calibration_id")
            or glove_frame.get("calibration_id")
            or ""
        )
        version_ok = bool(
            self.expected_glove_calibration_id
            and glove_calibration_id == self.expected_glove_calibration_id
        )
        paired_controller_q, paired_time_gap_sec = self._paired_controller_quaternion(
            controller_pose, glove_frame
        )
        innovation = (
            quaternion_angle_deg_xyzw(paired_controller_q, aligned_glove)
            if paired_controller_q is not None and aligned_glove is not None
            else None
        )

        reasons = []
        if not controller_fresh:
            reasons.append("controller_stale_or_missing")
        elif not controller_tracked:
            reasons.append("controller_not_tracked")
        elif not deadman:
            reasons.append("deadman_inactive")
        if self.mode_requested == "m1":
            reasons.append("m1_controller_only")
        elif not self.calibration_id:
            reasons.append("alignment_calibration_invalid")
        if not glove_fresh:
            reasons.append("glove_stale_or_invalid")
        if self.mode_requested == "m3" and not version_ok:
            reasons.append("glove_calibration_id_mismatch")
        if (
            self.mode_requested == "m3"
            and paired_time_gap_sec is not None
            and paired_time_gap_sec > 0.06
        ):
            reasons.append("paired_time_gap_exceeded")
        if (
            self.mode_requested == "m3"
            and innovation is not None
            and innovation >= self.innovation_reject_deg
        ):
            reasons.append("orientation_innovation_rejected")

        target_weight = 0.0
        fusion_allowed = bool(
            controller_fresh
            and aligned_glove is not None
            and self.calibration_id
            and self.mode_requested in {"m2", "m3"}
            and (self.mode_requested != "m3" or version_ok)
            and (
                self.mode_requested != "m3"
                or paired_time_gap_sec is None
                or paired_time_gap_sec <= 0.06
            )
            and (
                self.mode_requested != "m3"
                or innovation is None
                or innovation < self.innovation_reject_deg
            )
        )
        if fusion_allowed:
            if self.mode_requested == "m2" or innovation is None or innovation <= self.innovation_soft_deg:
                target_weight = 1.0
            elif innovation < self.innovation_reject_deg:
                target_weight = (
                    self.innovation_reject_deg - innovation
                ) / (self.innovation_reject_deg - self.innovation_soft_deg)
        weight_alpha = 1.0 - math.exp(-dt / self.weight_tau_sec)
        self.glove_weight += (target_weight - self.glove_weight) * weight_alpha
        if self.glove_weight < 1e-4:
            self.glove_weight = 0.0

        if controller_q is not None and self.fused is None:
            self.fused = controller_q.copy()
        glove_seq = int(glove_frame.get("frameId", -1)) if glove_frame else -1
        if aligned_glove is not None and glove_seq != self.last_glove_seq:
            increment_weight = self.glove_weight if fusion_allowed and target_weight > 0.0 else 0.0
            if self.previous_aligned_glove is not None and self.fused is not None and increment_weight > 0.0:
                delta = quaternion_multiply_xyzw(
                    quaternion_conjugate_xyzw(self.previous_aligned_glove), aligned_glove
                )
                weighted_delta = quaternion_slerp_xyzw(
                    [0.0, 0.0, 0.0, 1.0], delta, increment_weight
                )
                self.fused = quaternion_multiply_xyzw(self.fused, weighted_delta)
            self.previous_aligned_glove = aligned_glove
            self.last_glove_seq = glove_seq
        if controller_q is not None:
            correction = 1.0 - math.exp(-dt / self.correction_tau_sec)
            self.fused = controller_q.copy() if self.fused is None else quaternion_slerp_xyzw(
                self.fused, controller_q, correction
            )

        output_pose = copy.deepcopy(controller_pose) if controller_pose is not None else None
        orientation_valid = bool(controller_authorized and self.fused is not None)
        active_mode = "invalid"
        if controller_authorized:
            active_mode = "fused" if self.glove_weight > 1e-4 and fusion_allowed else "controller_only"
            if output_pose is not None and orientation_valid:
                output_pose["rotation"] = self.fused.copy()
                output_pose["fusion_mode"] = active_mode
                output_pose["orientation_calibration_id"] = self.calibration_id

        finger_targets = glove_frame.get("mapped_target_units") if glove_fresh else None
        finger_valid = bool(
            glove_fresh
            and isinstance(finger_targets, list)
            and len(finger_targets) == 6
            and all(
                isinstance(value, (int, float))
                and math.isfinite(float(value))
                and 0.0 <= float(value) <= 2000.0
                for value in finger_targets
            )
        )
        rotations = glove_frame.get("rightRotations") if glove_frame else None
        local_fingers = rotations[HI5_LOCAL_FINGER_START * 3 :] if isinstance(rotations, list) else []
        self.output_seq += 1
        state = {
            "type": "master_fusion_state",
            "schema_version": 1,
            "seq": self.output_seq,
            "publish_wall_time_ns": wall_ns,
            "mode_requested": self.mode_requested,
            "mode": active_mode,
            "degradation_reasons": reasons,
            "controller": {
                "seq": int(controller_pose.get("seq", -1)) if controller_pose else -1,
                "device_time": float(controller_pose.get("device_time", 0.0)) if controller_pose else 0.0,
                "receiver_wall_time_ns": int(float(controller_pose.get("wall_time", 0.0)) * 1e9)
                if controller_pose else 0,
                "age_sec": controller_age,
                "tracked": controller_tracked,
                "deadman_active": deadman,
                "position_m": controller_position[:3].tolist()
                if controller_position_valid else [],
                "rotation_xyzw": controller_q.tolist() if controller_q is not None else [],
                "calibration_id": self.calibration_id,
            },
            "glove": {
                "frame_id": glove_seq,
                "receiver_wall_time_ns": int(glove_frame.get("receiver_wall_time_ns", 0))
                if glove_frame else 0,
                "age_sec": glove_age_sec,
                "valid": glove_fresh,
                "calibration_id": glove_calibration_id,
                "forearm_index": HI5_FOREARM_INDEX,
                "hand_index": HI5_HAND_INDEX,
                "forearm_euler_deg": rotations[0:3]
                if isinstance(rotations, list) and len(rotations) >= 3 else [],
                "hand_euler_deg": rotations[3:6] if isinstance(rotations, list) and len(rotations) >= 6 else [],
                "local_finger_rotations_deg": local_fingers,
                "mapped_target_units": finger_targets if finger_valid else [],
            },
            "orientation": {
                "controller_xyzw": controller_q.tolist() if controller_q is not None else [],
                "glove_aligned_xyzw": aligned_glove.tolist() if aligned_glove is not None else [],
                "fused_xyzw": self.fused.tolist() if self.fused is not None else [],
                "innovation_deg": innovation,
                "glove_weight": self.glove_weight,
                "paired_time_gap_ms": (
                    paired_time_gap_sec * 1000.0 if paired_time_gap_sec is not None else None
                ),
            },
            "validity": {
                "position": controller_authorized,
                "orientation": orientation_valid,
                "finger": finger_valid,
            },
            "calibrations": {
                "time_and_extrinsic": self.calibration_id,
                "glove_mapping": glove_calibration_id,
                "alignment_error": self.calibration_error,
                "glove_mapping_error": self.glove_calibration_error,
                "expected_glove_mapping": self.expected_glove_calibration_id,
            },
        }
        return output_pose, state
