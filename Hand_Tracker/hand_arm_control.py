#!/usr/bin/env python3
"""Local hand-recognition service for the H5 control UI.

This process does not solve arm IK and does not send arm UDP commands. It only
recognizes a hand, applies the unlock gesture/deadzone/gain logic, and exposes:

- /stream.mjpg: camera image with MediaPipe hand skeleton and overlay
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
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import cv2
import numpy as np

try:
    from tracker import DepthCameraTracker
    import utils
except ImportError:
    from .tracker import DepthCameraTracker
    from . import utils


DEFAULT_CONFIG: Dict[str, Any] = {
    "service": {
        "host": "127.0.0.1",
        "port": 8091,
        "event_hz": 20.0,
        "mjpeg_hz": 15.0,
        "jpeg_quality": 80,
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


def make_handler(shared: SharedState, event_hz: float, mjpeg_hz: float):
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


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).with_name("hand_control_config.yaml")
    parser = argparse.ArgumentParser(description="Local hand-recognition service for the H5 control UI.")
    parser.add_argument("--config", type=Path, default=default_config, help="YAML config path.")
    parser.add_argument("--host", type=str, default=None, help="HTTP service host. Overrides config.")
    parser.add_argument("--port", type=int, default=None, help="HTTP service port. Overrides config.")
    parser.add_argument("--camera-index", type=str, default=None, help="Camera index or 'auto'. Overrides config.")
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

    initial_status = controller.snapshot("starting", {}, None, "starting hand vision service")
    initial_status["online"] = False
    shared = SharedState(initial_status)

    host = str(cfg_get(config, ("service", "host"), "127.0.0.1"))
    port = int(cfg_get(config, ("service", "port"), 8091))
    event_hz = float(cfg_get(config, ("service", "event_hz"), 20.0))
    mjpeg_hz = float(cfg_get(config, ("service", "mjpeg_hz"), 15.0))

    handler = make_handler(shared, event_hz, mjpeg_hz)
    server = ThreadingHTTPServer((host, port), handler)
    capture_thread = threading.Thread(target=capture_loop, args=(config, shared, controller), daemon=True)
    capture_thread.start()

    print(f"[HandVision] service on http://{host}:{port}")
    print(
        "[HandVision] unlock=%s gain=(%.2f, %.2f, %.2f) step=%.4fm"
        % (settings.unlock_gesture, settings.gain[0], settings.gain[1], settings.gain[2], settings.position_step_m)
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
