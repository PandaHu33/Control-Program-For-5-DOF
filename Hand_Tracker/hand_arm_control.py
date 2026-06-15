#!/usr/bin/env python3
"""Local hand-recognition service for the H5 control UI.

This process does not solve arm IK and does not send arm UDP commands. It only
recognizes a hand, applies the unlock gesture/deadzone/gain logic, and exposes:

- /stream.mjpg: camera image with MediaPipe hand skeleton and overlay
- /frame.jpg: latest single JPEG frame for WebViews that do not render MJPEG
- /events: Server-Sent Events containing xyz incremental input axes
- /api/status: latest JSON status snapshot

The H5 page consumes those axes and routes them through the same position-mode
IK and UDP frame path already used by keyboard and gamepad control.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
    import cv2
except Exception as exc:
    cv2 = None
    _CV2_IMPORT_ERROR = exc
else:
    _CV2_IMPORT_ERROR = None

try:
    import numpy as np
except Exception as exc:
    np = None
    _NUMPY_IMPORT_ERROR = exc
else:
    _NUMPY_IMPORT_ERROR = None

try:
    from tracker import DepthCameraTracker
    import utils
    _CAMERA_INPUT_AVAILABLE = True
    _CAMERA_IMPORT_ERROR = None
except Exception as exc:
    DepthCameraTracker = None
    utils = None
    _CAMERA_INPUT_AVAILABLE = False
    _CAMERA_IMPORT_ERROR = exc


DEFAULT_CONFIG: Dict[str, Any] = {
    "service": {
        "host": "127.0.0.1",
        "port": 8091,
        "event_hz": 20.0,
        "mjpeg_hz": 15.0,
        "jpeg_quality": 80,
    },
    "input": {
        "source": "vr",
    },
    "vr": {
        "host": "127.0.0.1",
        "port": 5005,
        "hand": "right",
        "stale_timeout_sec": 0.6,
        "deadzone_m": 0.015,
        "smoothing_alpha": 0.18,
        "max_axis": 0.45,
        "axis_gain_x": 2.0,
        "axis_gain_y": 2.0,
        "axis_gain_z": 2.0,
        "axis_sign": {
            "x_from_vr_y": 1.0,
            "y_from_vr_x": 1.0,
            "z_from_vr_z": 1.0,
        },
    },
    "camera": {
        "index": "auto",
        "min_detection_confidence": 0.7,
        "min_tracking_confidence": 0.7,
        "show_timing": True,
    },
    "control": {
        "unlock_gesture": "victory",
        "unlock_hold_sec": 0.35,
        "state_toggle_cooldown_sec": 5.0,
        "control_clutch_grace_sec": 0.35,
        "pinch_threshold": 0.45,
        "deadzone_xy": 0.06,
        "deadzone_depth": 0.04,
        "smoothing_alpha": 0.18,
        "axis_gain_x": 1.2,
        "axis_gain_y": 1.5,
        "axis_gain_z": 1.5,
        "max_axis": 0.35,
        "position_step_m": 0.003,
        "axis_sign": {
            "x_from_depth": 1.0,
            "y_from_image_x": -1.0,
            "z_from_image_y": -1.0,
        },
    },
}


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: Optional[Path]) -> Dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    if not path or not path.exists():
        return config
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required to read hand_control_config.yaml") from exc
    with path.open("r", encoding="utf-8") as fh:
        loaded = yaml.safe_load(fh) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Config root must be a mapping: {path}")
    return deep_merge(config, loaded)


def cfg_get(config: Dict[str, Any], path: Iterable[str], default: Any = None) -> Any:
    node: Any = config
    for item in path:
        if not isinstance(node, dict) or item not in node:
            return default
        node = node[item]
    return node


def parse_camera_index(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in {"auto", "none"}:
        return None
    return int(value)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign(abs(value) - deadzone, value)


@dataclass
class ControlSettings:
    unlock_gesture: str
    unlock_hold_sec: float
    state_toggle_cooldown_sec: float
    control_clutch_grace_sec: float
    pinch_threshold: float
    deadzone_xy: float
    deadzone_depth: float
    smoothing_alpha: float
    gain: np.ndarray
    max_axis: float
    position_step_m: float
    axis_sign: np.ndarray


class HandAxisController:
    def __init__(self, settings: ControlSettings):
        self.settings = settings
        self.anchor: Optional[np.ndarray] = None
        self.toggle_started_at: Optional[float] = None
        self.last_state_changed_at = float("-inf")
        self.toggle_armed = True
        self.unlocked = False
        self.last_clutch_at = float("-inf")
        self.axis = np.zeros(3, dtype=float)
        self.raw_axis = np.zeros(3, dtype=float)

    def reset(self) -> None:
        self.anchor = None
        self.toggle_started_at = None
        self.last_state_changed_at = float("-inf")
        self.toggle_armed = True
        self.unlocked = False
        self.last_clutch_at = float("-inf")
        self.axis[:] = 0.0
        self.raw_axis[:] = 0.0

    def _zero_axes(self) -> None:
        self.axis[:] = 0.0
        self.raw_axis[:] = 0.0

    def _release_clutch(self, hand_point: np.ndarray) -> None:
        self.anchor = hand_point.copy()
        self.raw_axis[:] = 0.0
        self.axis *= 0.75
        self.axis[np.abs(self.axis) < 0.01] = 0.0

    def _cooldown_remaining(self, now: float) -> float:
        elapsed = now - self.last_state_changed_at
        return max(0.0, self.settings.state_toggle_cooldown_sec - elapsed)

    def _lock(self, reason: str, now: float) -> None:
        if self.unlocked:
            print(f"[HandVision] locked: {reason}")
        self.unlocked = False
        self.anchor = None
        self.toggle_started_at = None
        self.last_state_changed_at = now
        self.toggle_armed = False
        self._zero_axes()

    def _unlock(self, hand_point: np.ndarray, now: float) -> None:
        self.unlocked = True
        self.anchor = hand_point.copy()
        self.toggle_started_at = None
        self.last_state_changed_at = now
        self.last_clutch_at = now
        self.toggle_armed = True
        self._zero_axes()
        print("[HandVision] unlocked; current hand position is neutral")

    def update_no_hand(self, now: float) -> Dict[str, Any]:
        self.toggle_started_at = None
        self.toggle_armed = True
        self.anchor = None
        self._zero_axes()
        message = "unlocked; no hand" if self.unlocked else "no hand"
        return self.snapshot("no_hand", None, None, message)

    def update(self, lm: Any, now: float) -> Dict[str, Any]:
        gesture, fingers = utils.classify_hand_gesture(lm, pinch_threshold=self.settings.pinch_threshold)
        hand_point = utils.get_wrist_control_point(lm)
        unlock_active = gesture == self.settings.unlock_gesture

        if self.unlocked:
            self.toggle_started_at = None
            self.toggle_armed = True
            if self.anchor is None:
                self.anchor = hand_point.copy()
                self._zero_axes()
                return self.snapshot(gesture, fingers, hand_point, "controlling; neutral reset")
            if unlock_active:
                self.last_clutch_at = now
                return self._update_axis(gesture, fingers, hand_point, clutch_active=True)
            if now - self.last_clutch_at <= self.settings.control_clutch_grace_sec:
                return self._update_axis(gesture, fingers, hand_point, clutch_active=True, message="controlling; clutch grace")
            return self._update_axis(gesture, fingers, hand_point)

        if not unlock_active:
            self.toggle_started_at = None
            self.toggle_armed = True
            self._zero_axes()
            return self.snapshot(gesture, fingers, hand_point, "waiting for unlock gesture")

        self._zero_axes()
        if not self.toggle_armed:
            return self.snapshot(gesture, fingers, hand_point, "release toggle gesture before next switch")

        remaining = self._cooldown_remaining(now)
        if remaining > 0:
            self.toggle_started_at = None
            return self.snapshot(gesture, fingers, hand_point, f"toggle cooldown {remaining:.1f}s; neutral reset")

        if self.toggle_started_at is None:
            self.toggle_started_at = now

        if not self.unlocked:
            if now - self.toggle_started_at >= self.settings.unlock_hold_sec:
                self._unlock(hand_point, now)
                return self.snapshot(gesture, fingers, hand_point, "unlocked")
            return self.snapshot(gesture, fingers, hand_point, "hold unlock gesture")

        if now - self.toggle_started_at >= self.settings.unlock_hold_sec:
            self._lock("toggle gesture", now)
            return self.snapshot(gesture, fingers, hand_point, "locked")
        return self.snapshot(gesture, fingers, hand_point, "hold lock gesture")

    def _update_axis(
        self,
        gesture: str,
        fingers: Dict[str, bool],
        hand_point: np.ndarray,
        clutch_active: bool = False,
        message: str = "controlling",
    ) -> Dict[str, Any]:
        if not clutch_active and gesture != self.settings.unlock_gesture:
            self._release_clutch(hand_point)
            return self.snapshot(gesture, fingers, hand_point, "clutch released; neutral reset")

        if self.anchor is None:
            self.anchor = hand_point.copy()

        offset = hand_point - self.anchor
        if float(np.linalg.norm(offset)) < 0.08:
            recenter_alpha = 0.03
            self.anchor = (1.0 - recenter_alpha) * self.anchor + recenter_alpha * hand_point
            offset = hand_point - self.anchor

        raw = np.array(
            [
                self.settings.axis_sign[0] * apply_deadzone(float(offset[2]), self.settings.deadzone_depth),
                self.settings.axis_sign[1] * apply_deadzone(float(offset[0]), self.settings.deadzone_xy),
                self.settings.axis_sign[2] * apply_deadzone(float(offset[1]), self.settings.deadzone_xy),
            ],
            dtype=float,
        )
        raw = raw * self.settings.gain
        max_axis = max(self.settings.max_axis, 1e-6)
        raw = max_axis * np.tanh(raw / max_axis)
        alpha = clamp(self.settings.smoothing_alpha, 0.0, 1.0)
        self.raw_axis = raw
        self.axis = (1.0 - alpha) * self.axis + alpha * raw
        self.axis[np.abs(self.axis) < 0.01] = 0.0
        return self.snapshot(gesture, fingers, hand_point, message)

    def snapshot(
        self,
        gesture: str,
        fingers: Optional[Dict[str, bool]],
        hand_point: Optional[np.ndarray],
        message: str,
    ) -> Dict[str, Any]:
        offset = None
        if hand_point is not None and self.anchor is not None:
            offset = hand_point - self.anchor
        return {
            "ok": True,
            "online": True,
            "time": time.time(),
            "gesture": gesture,
            "required_gesture": self.settings.unlock_gesture,
            "unlocked": self.unlocked,
            "message": message,
            "axis": {
                "x": float(self.axis[0]),
                "y": float(self.axis[1]),
                "z": float(self.axis[2]),
            },
            "raw_axis": {
                "x": float(self.raw_axis[0]),
                "y": float(self.raw_axis[1]),
                "z": float(self.raw_axis[2]),
            },
            "hand": {
                "x": float(hand_point[0]) if hand_point is not None else None,
                "y": float(hand_point[1]) if hand_point is not None else None,
                "depth": float(hand_point[2]) if hand_point is not None else None,
            },
            "offset": {
                "x": float(offset[0]) if offset is not None else 0.0,
                "y": float(offset[1]) if offset is not None else 0.0,
                "depth": float(offset[2]) if offset is not None else 0.0,
            },
            "fingers": fingers or {},
            "mapping": {
                "position_step_m": float(self.settings.position_step_m),
                "max_axis": float(self.settings.max_axis),
            },
        }



class VRWristPoseReceiver:
    """TCP server for wrist pose lines sent by Unity over adb reverse.

    Expected line format:
      seq,time,hand,tracked,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w
    """

    def __init__(self, host: str, port: int, stopped: threading.Event):
        self.host = host
        self.port = int(port)
        self.stopped = stopped
        self.lock = threading.RLock()
        self.latest_by_hand: Dict[str, Dict[str, Any]] = {}
        self.connected = False
        self.last_error = "waiting for Unity wrist TCP connection"
        self._server: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._run, name="VRWristPoseReceiver", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        for sock in (self._client, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._thread.join(timeout=1.0)

    def latest(self, hand: str) -> Optional[Dict[str, Any]]:
        key = str(hand or "right").lower()
        with self.lock:
            if key in self.latest_by_hand:
                return copy.deepcopy(self.latest_by_hand[key])
            return None

    def status(self) -> Dict[str, Any]:
        with self.lock:
            return {"connected": self.connected, "message": self.last_error}

    def _run(self) -> None:
        try:
            self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._server.bind((self.host, self.port))
            self._server.listen(1)
            self._server.settimeout(0.5)
            with self.lock:
                self.last_error = f"listening on {self.host}:{self.port}"
            print(f"[VRWrist] listening on {self.host}:{self.port}; expected adb reverse tcp:{self.port} tcp:{self.port}")

            while not self.stopped.is_set():
                try:
                    client, addr = self._server.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stopped.is_set():
                        with self.lock:
                            self.last_error = str(exc)
                    break

                self._client = client
                client.settimeout(0.5)
                with self.lock:
                    self.connected = True
                    self.last_error = f"connected from {addr[0]}:{addr[1]}"
                print(f"[VRWrist] connected from {addr[0]}:{addr[1]}")
                self._read_client(client)
                with self.lock:
                    self.connected = False
                    self.last_error = "Unity wrist TCP disconnected; waiting for reconnect"
                print("[VRWrist] disconnected")
        except OSError as exc:
            with self.lock:
                self.connected = False
                self.last_error = str(exc)
            print(f"[VRWrist] fatal: {exc}")
        finally:
            if self._server is not None:
                try:
                    self._server.close()
                except OSError:
                    pass

    def _read_client(self, client: socket.socket) -> None:
        buf = b""
        try:
            while not self.stopped.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    self._parse_line(raw.decode("ascii", errors="replace").strip())
        except OSError as exc:
            with self.lock:
                self.last_error = str(exc)
        finally:
            try:
                client.close()
            except OSError:
                pass
            self._client = None

    def _parse_line(self, line: str) -> None:
        if not line:
            return
        parts = [item.strip() for item in line.split(",")]
        if len(parts) < 11:
            return
        try:
            hand = parts[2].lower()
            if hand not in {"left", "right"}:
                return
            tracked = int(float(parts[3])) == 1
            if not tracked:
                with self.lock:
                    self.latest_by_hand.pop(hand, None)
                    self.last_error = f"{hand} wrist control disabled or not tracked"
                return
            pose = {
                "seq": int(float(parts[0])),
                "device_time": float(parts[1]),
                "hand": hand,
                "position": np.array([float(parts[4]), float(parts[5]), float(parts[6])], dtype=float),
                "rotation": np.array([float(parts[7]), float(parts[8]), float(parts[9]), float(parts[10])], dtype=float),
                "received_at": time.monotonic(),
                "wall_time": time.time(),
            }
        except (TypeError, ValueError):
            return
        with self.lock:
            self.latest_by_hand[hand] = pose
            self.last_error = f"receiving {hand} wrist seq={pose['seq']}"


class VRWristAxisController:
    def __init__(self, config: Dict[str, Any], settings: ControlSettings):
        self.settings = settings
        self.hand = str(cfg_get(config, ("vr", "hand"), "right")).lower()
        self.stale_timeout_sec = float(cfg_get(config, ("vr", "stale_timeout_sec"), 0.6))
        self.deadzone_m = float(cfg_get(config, ("vr", "deadzone_m"), 0.015))
        self.alpha = clamp(float(cfg_get(config, ("vr", "smoothing_alpha"), settings.smoothing_alpha)), 0.0, 1.0)
        self.max_axis = float(cfg_get(config, ("vr", "max_axis"), settings.max_axis))
        self.gain = np.array([
            float(cfg_get(config, ("vr", "axis_gain_x"), 2.0)),
            float(cfg_get(config, ("vr", "axis_gain_y"), 2.0)),
            float(cfg_get(config, ("vr", "axis_gain_z"), 2.0)),
        ], dtype=float)
        signs = cfg_get(config, ("vr", "axis_sign"), {}) or {}
        self.sign = np.array([
            float(signs.get("x_from_vr_y", 1.0)),
            float(signs.get("y_from_vr_x", 1.0)),
            float(signs.get("z_from_vr_z", 1.0)),
        ], dtype=float)
        self.anchor: Optional[np.ndarray] = None
        self.active = False
        self.axis = np.zeros(3, dtype=float)
        self.raw_axis = np.zeros(3, dtype=float)

    def update(self, pose: Optional[Dict[str, Any]], receiver_status: Dict[str, Any]) -> Dict[str, Any]:
        now = time.monotonic()
        if pose is None:
            self._stop_control()
            return self.snapshot(False, None, "waiting for VR wrist pose; " + str(receiver_status.get("message", "")))

        age = now - float(pose.get("received_at", 0.0))
        if age > self.stale_timeout_sec:
            self._stop_control()
            return self.snapshot(False, pose, f"VR wrist stale {age:.2f}s")

        point = np.asarray(pose["position"], dtype=float)
        if not self.active or self.anchor is None:
            self.active = True
            self.anchor = point.copy()
            self._zero()
            return self.snapshot(True, pose, "VR wrist neutral set")

        offset = point - self.anchor
        raw = np.array([
            self.sign[0] * apply_deadzone(float(offset[1]), self.deadzone_m),
            self.sign[1] * apply_deadzone(float(offset[0]), self.deadzone_m),
            self.sign[2] * apply_deadzone(float(offset[2]), self.deadzone_m),
        ], dtype=float)
        raw = raw * self.gain
        max_axis = max(self.max_axis, 1e-6)
        raw = max_axis * np.tanh(raw / max_axis)
        self.raw_axis = raw
        self.axis = (1.0 - self.alpha) * self.axis + self.alpha * raw
        self.axis[np.abs(self.axis) < 0.01] = 0.0
        return self.snapshot(True, pose, "VR wrist controlling")

    def _zero(self) -> None:
        self.axis[:] = 0.0
        self.raw_axis[:] = 0.0

    def _stop_control(self) -> None:
        self.active = False
        self.anchor = None
        self._zero()

    def recenter(self, pose: Optional[Dict[str, Any]]) -> None:
        if pose is not None:
            self.anchor = np.asarray(pose["position"], dtype=float).copy()
            self.active = True
        self._zero()

    def snapshot(self, online: bool, pose: Optional[Dict[str, Any]], message: str) -> Dict[str, Any]:
        point = np.asarray(pose["position"], dtype=float) if pose is not None else None
        offset = point - self.anchor if point is not None and self.anchor is not None else np.zeros(3, dtype=float)
        return {
            "ok": True,
            "online": online,
            "time": time.time(),
            "source": "vr",
            "gesture": "vr_wrist" if online else "no_vr_wrist",
            "required_gesture": "none",
            "unlocked": online,
            "control_active": bool(self.active and online),
            "tracking_active": bool(online),
            "message": message,
            "axis": {"x": float(self.axis[0]), "y": float(self.axis[1]), "z": float(self.axis[2])},
            "raw_axis": {"x": float(self.raw_axis[0]), "y": float(self.raw_axis[1]), "z": float(self.raw_axis[2])},
            "hand": {
                "x": float(point[0]) if point is not None else None,
                "y": float(point[1]) if point is not None else None,
                "depth": float(point[2]) if point is not None else None,
            },
            "offset": {"x": float(offset[0]), "y": float(offset[1]), "depth": float(offset[2])},
            "fingers": {},
            "mapping": {"position_step_m": float(self.settings.position_step_m), "max_axis": float(self.max_axis)},
            "vr": {
                "hand": self.hand,
                "seq": int(pose.get("seq", -1)) if pose is not None else -1,
                "age_sec": float(time.monotonic() - pose["received_at"]) if pose is not None else None,
                "active": bool(self.active and online),
                "anchor_set": self.anchor is not None,
            },
        }


class SharedState:
    def __init__(self, initial_status: Dict[str, Any]):
        self.lock = threading.RLock()
        self.status = initial_status
        self.frame_jpeg: Optional[bytes] = None
        self.frame_seq = 0
        self.stopped = threading.Event()

    def update_status(self, status: Dict[str, Any]) -> None:
        with self.lock:
            self.status = status

    def update_frame(self, frame_jpeg: bytes) -> None:
        with self.lock:
            self.frame_jpeg = frame_jpeg
            self.frame_seq += 1

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return copy.deepcopy(self.status)

    def jpeg_snapshot(self) -> tuple[Optional[bytes], int]:
        with self.lock:
            return self.frame_jpeg, self.frame_seq


def build_settings(config: Dict[str, Any]) -> ControlSettings:
    signs = cfg_get(config, ("control", "axis_sign"), {}) or {}
    return ControlSettings(
        unlock_gesture=str(cfg_get(config, ("control", "unlock_gesture"), "victory")),
        unlock_hold_sec=float(cfg_get(config, ("control", "unlock_hold_sec"), 0.35)),
        state_toggle_cooldown_sec=float(cfg_get(config, ("control", "state_toggle_cooldown_sec"), 5.0)),
        control_clutch_grace_sec=float(cfg_get(config, ("control", "control_clutch_grace_sec"), 0.35)),
        pinch_threshold=float(cfg_get(config, ("control", "pinch_threshold"), 0.45)),
        deadzone_xy=float(cfg_get(config, ("control", "deadzone_xy"), 0.06)),
        deadzone_depth=float(cfg_get(config, ("control", "deadzone_depth"), 0.04)),
        smoothing_alpha=float(cfg_get(config, ("control", "smoothing_alpha"), 0.18)),
        gain=np.array(
            [
                float(cfg_get(config, ("control", "axis_gain_x"), 1.2)),
                float(cfg_get(config, ("control", "axis_gain_y"), 1.5)),
                float(cfg_get(config, ("control", "axis_gain_z"), 1.5)),
            ],
            dtype=float,
        ),
        max_axis=float(cfg_get(config, ("control", "max_axis"), 0.35)),
        position_step_m=float(cfg_get(config, ("control", "position_step_m"), 0.003)),
        axis_sign=np.array(
            [
                float(signs.get("x_from_depth", 1.0)),
                float(signs.get("y_from_image_x", -1.0)),
                float(signs.get("z_from_image_y", -1.0)),
            ],
            dtype=float,
        ),
    )


def draw_overlay(frame: np.ndarray, status: Dict[str, Any]) -> None:
    unlocked = bool(status.get("unlocked"))
    color = (0, 220, 0) if unlocked else (0, 180, 255)
    state = "UNLOCKED" if unlocked else "LOCKED"
    axis = status.get("axis", {})
    hand = status.get("hand", {})
    lines = [
        f"State: {state} | Gesture: {status.get('gesture')} | Need: {status.get('required_gesture')}",
        f"Axis XYZ: {axis.get('x', 0):+.3f}, {axis.get('y', 0):+.3f}, {axis.get('z', 0):+.3f}",
        f"Hand x/y/depth: {hand.get('x') or 0:.3f}, {hand.get('y') or 0:.3f}, {hand.get('depth') or 0:.3f}",
        str(status.get("message", "")),
    ]
    y = 28
    for line in lines:
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2)
        y += 28


def make_handler(shared: SharedState, event_hz: float, mjpeg_hz: float, allow_status_frame_fallback: bool = True):
    class HandServiceHandler(BaseHTTPRequestHandler):
        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self) -> None:
            self.send_response(200)
            self._cors()
            self.end_headers()

        def do_GET(self) -> None:
            if self.path.startswith("/api/status"):
                self.send_json(shared.snapshot())
            elif self.path.startswith("/events"):
                self.send_events()
            elif self.path.startswith("/frame.jpg"):
                self.send_frame_jpeg()
            elif self.path.startswith("/stream.mjpg"):
                self.send_mjpeg()
            else:
                self.send_response(404)
                self._cors()
                self.end_headers()

        def send_json(self, body: Dict[str, Any]) -> None:
            raw = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def send_events(self) -> None:
            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            delay = 1.0 / max(event_hz, 1.0)
            while not shared.stopped.is_set():
                payload = json.dumps(shared.snapshot(), ensure_ascii=False)
                try:
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
                time.sleep(delay)

        def send_frame_jpeg(self) -> None:
            jpeg, _ = shared.jpeg_snapshot()
            if jpeg is None and allow_status_frame_fallback:
                jpeg = draw_vr_frame(shared.snapshot())

            if not jpeg:
                self.send_response(503)
                self._cors()
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return

            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store, no-cache, max-age=0")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def send_mjpeg(self) -> None:
            boundary = b"--frame"
            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            delay = 1.0 / max(mjpeg_hz, 1.0)
            last_seq = -1
            while not shared.stopped.is_set():
                jpeg, seq = shared.jpeg_snapshot()
                if jpeg is not None and seq != last_seq:
                    last_seq = seq
                    try:
                        self.wfile.write(boundary + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg + b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
                time.sleep(delay)

        def log_message(self, fmt: str, *args: Any) -> None:
            return

    return HandServiceHandler


def capture_loop(config: Dict[str, Any], shared: SharedState, controller: HandAxisController) -> None:
    if not _CAMERA_INPUT_AVAILABLE or DepthCameraTracker is None or utils is None or cv2 is None:
        raise RuntimeError(f"camera input is unavailable: {_CAMERA_IMPORT_ERROR or _CV2_IMPORT_ERROR}")
    tracker = None
    jpeg_quality = int(cfg_get(config, ("service", "jpeg_quality"), 80))
    try:
        tracker = DepthCameraTracker(
            camera_index=parse_camera_index(cfg_get(config, ("camera", "index"), 0)),
            min_detection_confidence=float(cfg_get(config, ("camera", "min_detection_confidence"), 0.7)),
            min_tracking_confidence=float(cfg_get(config, ("camera", "min_tracking_confidence"), 0.7)),
            show_timing=bool(cfg_get(config, ("camera", "show_timing"), True)),
        )
        while not shared.stopped.is_set():
            now = time.monotonic()
            hand_results, _, _, frame = tracker.get_frames_and_process_hands()
            if frame is None:
                time.sleep(0.01)
                continue

            if hand_results and hand_results.multi_hand_landmarks:
                lm = hand_results.multi_hand_landmarks[0]
                tracker.draw_landmarks(frame, lm)
                status = controller.update(lm, now)
            else:
                status = controller.update_no_hand(now)

            draw_overlay(frame, status)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if ok:
                shared.update_frame(encoded.tobytes())
            shared.update_status(status)
    except Exception as exc:
        shared.update_status({
            "ok": False,
            "online": False,
            "time": time.time(),
            "gesture": "error",
            "required_gesture": controller.settings.unlock_gesture,
            "unlocked": False,
            "message": str(exc),
            "axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "raw_axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "hand": {"x": None, "y": None, "depth": None},
            "offset": {"x": 0.0, "y": 0.0, "depth": 0.0},
            "fingers": {},
            "mapping": {"position_step_m": controller.settings.position_step_m, "max_axis": controller.settings.max_axis},
        })
        print(f"[HandVision] fatal: {exc}")
        while not shared.stopped.is_set():
            time.sleep(0.5)
    finally:
        if tracker:
            tracker.close()



def draw_vr_frame(status: Dict[str, Any]) -> bytes:
    if cv2 is None or np is None:
        return b""
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    online = bool(status.get("online"))
    color = (0, 220, 0) if online else (0, 180, 255)
    axis = status.get("axis", {})
    hand = status.get("hand", {})
    lines = [
        "VR Wrist Input",
        f"State: {'ONLINE' if online else 'WAITING'} | {status.get('message', '')}",
        f"Axis XYZ: {axis.get('x', 0):+.3f}, {axis.get('y', 0):+.3f}, {axis.get('z', 0):+.3f}",
        f"Wrist x/y/z: {hand.get('x') or 0:.3f}, {hand.get('y') or 0:.3f}, {hand.get('depth') or 0:.3f}",
    ]
    y = 48
    for line in lines:
        cv2.putText(frame, line[:92], (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)
        y += 44
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return encoded.tobytes() if ok else b""


def vr_capture_loop(config: Dict[str, Any], shared: SharedState, settings: ControlSettings) -> None:
    stopped = shared.stopped
    host = str(cfg_get(config, ("vr", "host"), "127.0.0.1"))
    port = int(cfg_get(config, ("vr", "port"), 5005))
    receiver = VRWristPoseReceiver(host, port, stopped)
    controller = VRWristAxisController(config, settings)
    interval = 1.0 / max(float(cfg_get(config, ("service", "event_hz"), 20.0)), 1.0)
    receiver.start()
    try:
        while not stopped.is_set():
            pose = receiver.latest(controller.hand)
            status = controller.update(pose, receiver.status())
            shared.update_status(status)
            frame = draw_vr_frame(status)
            if frame:
                shared.update_frame(frame)
            time.sleep(interval)
    except Exception as exc:
        shared.update_status({
            "ok": False,
            "online": False,
            "time": time.time(),
            "source": "vr",
            "gesture": "error",
            "required_gesture": "none",
            "unlocked": False,
            "control_active": False,
            "tracking_active": False,
            "message": str(exc),
            "axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "raw_axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "hand": {"x": None, "y": None, "depth": None},
            "offset": {"x": 0.0, "y": 0.0, "depth": 0.0},
            "fingers": {},
            "mapping": {"position_step_m": settings.position_step_m, "max_axis": settings.max_axis},
            "vr": {"hand": str(cfg_get(config, ("vr", "hand"), "right")).lower(), "seq": -1, "age_sec": None, "active": False, "anchor_set": False},
        })
        print(f"[VRWrist] fatal: {exc}")
        while not stopped.is_set():
            time.sleep(0.5)
    finally:
        receiver.close()


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).with_name("hand_control_config.yaml")
    parser = argparse.ArgumentParser(description="Local hand-recognition service for the H5 control UI.")
    parser.add_argument("--config", type=Path, default=default_config, help="YAML config path.")
    parser.add_argument("--host", type=str, default=None, help="HTTP service host. Overrides config.")
    parser.add_argument("--port", type=int, default=None, help="HTTP service port. Overrides config.")
    parser.add_argument("--camera-index", type=str, default=None, help="Camera index or 'auto'. Overrides config.")
    parser.add_argument("--input-source", choices=["vr", "camera"], default=None, help="Input source. Defaults to config, normally vr.")
    parser.add_argument("--vr-host", type=str, default=None, help="VR wrist TCP listen host.")
    parser.add_argument("--vr-port", type=int, default=None, help="VR wrist TCP listen port.")
    parser.add_argument("--vr-hand", choices=["left", "right"], default=None, help="VR hand used for arm control.")
    parser.add_argument("--unlock-gesture", choices=["victory", "open_palm", "fist", "pinch"], default=None)
    parser.add_argument("--position-step", type=float, default=None, help="Per UI tick XYZ step in meters.")
    parser.add_argument("--axis-gain", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"))
    return parser.parse_args()


def apply_cli_overrides(config: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    if args.host is not None:
        config["service"]["host"] = args.host
    if args.port is not None:
        config["service"]["port"] = args.port
    if args.camera_index is not None:
        config["camera"]["index"] = args.camera_index
    if args.input_source is not None:
        config.setdefault("input", {})["source"] = args.input_source
    if args.vr_host is not None:
        config.setdefault("vr", {})["host"] = args.vr_host
    if args.vr_port is not None:
        config.setdefault("vr", {})["port"] = args.vr_port
    if args.vr_hand is not None:
        config.setdefault("vr", {})["hand"] = args.vr_hand
    if args.unlock_gesture is not None:
        config["control"]["unlock_gesture"] = args.unlock_gesture
    if args.position_step is not None:
        config["control"]["position_step_m"] = args.position_step
    if args.axis_gain is not None:
        config["control"]["axis_gain_x"] = args.axis_gain[0]
        config["control"]["axis_gain_y"] = args.axis_gain[1]
        config["control"]["axis_gain_z"] = args.axis_gain[2]
    return config


def main() -> int:
    args = parse_args()
    config = apply_cli_overrides(load_config(args.config), args)
    settings = build_settings(config)
    controller = HandAxisController(settings)

    input_source = str(cfg_get(config, ("input", "source"), "vr")).lower()
    if input_source == "vr":
        initial_status = {
            "ok": True,
            "online": False,
            "time": time.time(),
            "source": "vr",
            "gesture": "starting",
            "required_gesture": "none",
            "unlocked": False,
            "control_active": False,
            "tracking_active": False,
            "message": "starting VR wrist service",
            "axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "raw_axis": {"x": 0.0, "y": 0.0, "z": 0.0},
            "hand": {"x": None, "y": None, "depth": None},
            "offset": {"x": 0.0, "y": 0.0, "depth": 0.0},
            "fingers": {},
            "mapping": {"position_step_m": settings.position_step_m, "max_axis": settings.max_axis},
            "vr": {"hand": str(cfg_get(config, ("vr", "hand"), "right")).lower(), "seq": -1, "age_sec": None, "active": False, "anchor_set": False},
        }
    else:
        initial_status = controller.snapshot("starting", {}, None, "starting hand vision service")
        initial_status["online"] = False
    shared = SharedState(initial_status)

    host = str(cfg_get(config, ("service", "host"), "127.0.0.1"))
    port = int(cfg_get(config, ("service", "port"), 8091))
    event_hz = float(cfg_get(config, ("service", "event_hz"), 20.0))
    mjpeg_hz = float(cfg_get(config, ("service", "mjpeg_hz"), 15.0))

    handler = make_handler(shared, event_hz, mjpeg_hz, allow_status_frame_fallback=(input_source == "vr"))
    server = ThreadingHTTPServer((host, port), handler)
    if input_source == "vr":
        capture_thread = threading.Thread(target=vr_capture_loop, args=(config, shared, settings), daemon=True)
    elif input_source == "camera":
        capture_thread = threading.Thread(target=capture_loop, args=(config, shared, controller), daemon=True)
    else:
        raise ValueError(f"Unsupported input source: {input_source}")
    capture_thread.start()

    print(f"[HandVision] service on http://{host}:{port}")
    print(
        "[HandVision] source=%s unlock=%s gain=(%.2f, %.2f, %.2f) step=%.4fm"
        % (input_source, settings.unlock_gesture, settings.gain[0], settings.gain[1], settings.gain[2], settings.position_step_m)
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n[HandVision] interrupted")
    finally:
        shared.stopped.set()
        server.server_close()
        capture_thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
