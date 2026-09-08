#!/usr/bin/env python3
"""Bridge Unity XR hand JSON from adb-reversed TCP to local UDP.

Typical setup:
  adb reverse tcp:5006 tcp:5006
  python Hand_Tracker/unity_hand_udp_bridge.py
  wa100-sdk-publish/build/bin/udp_receiver_unity.exe right

Unity connects to 127.0.0.1:5006 on the headset. adb reverse delivers that
TCP stream to this script on the PC. Valid packets are forwarded to
udp_receiver_unity.cpp at 127.0.0.1:25001, while left-fist state is evaluated
locally and published to the H5 arm-control service on UDP 25002.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from typing import Any, Dict, Optional, Tuple


REQUIRED_ARRAYS = ("rightPositions", "rightRotations")
OPTIONAL_ARRAYS = ("leftPositions", "leftRotations")
EXPECTED_FLOAT_COUNT = 21 * 3
FINGER_CURL_CALIBRATION = (
    (5, 6, 8, 153.179, 55.0),
    (9, 10, 12, 147.612, 55.0),
    (13, 14, 16, 154.575, 56.0),
    (17, 18, 20, 160.095, 55.0),
)


class XrSkeletonOneEuroFilter:
    """One Euro filter for one 21x3 XR hand-position skeleton.

    Filtering happens before any curl, pinch or WA100 retargeting so every
    downstream consumer observes the same conditioned geometry.
    """

    RESET_GAP_SEC = 0.25
    MIN_CUTOFF_HZ = 1.5
    BETA = 0.6
    DERIVATIVE_CUTOFF_HZ = 1.0

    def __init__(self, adaptive: bool = True, thumb_cutoff_hz: float = 0.18,
                 finger_cutoff_hz: float = 0.35, local_beta: float = 5.0) -> None:
        if not all(math.isfinite(x) and x > 0 for x in (thumb_cutoff_hz, finger_cutoff_hz, local_beta)):
            raise ValueError("XR filter settings must be positive and finite")
        self.adaptive = adaptive
        self.thumb_cutoff_hz = thumb_cutoff_hz
        self.finger_cutoff_hz = finger_cutoff_hz
        self.local_beta = local_beta
        self.generation = 0
        self.basis = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self.raw: Optional[list[float]] = None
        self.filtered: Optional[list[float]] = None
        self.derivative = [0.0] * EXPECTED_FLOAT_COUNT
        self.last_time: Optional[float] = None

    @staticmethod
    def _alpha(dt: float, cutoff_hz: float) -> float:
        safe_cutoff = max(float(cutoff_hz), 1e-6)
        tau = 1.0 / (2.0 * math.pi * safe_cutoff)
        return dt / (tau + dt)

    def reset(self) -> None:
        self.raw = None
        self.filtered = None
        self.derivative = [0.0] * EXPECTED_FLOAT_COUNT
        self.last_time = None

    def _initialize(self, values: list[float], now: float) -> list[float]:
        self.generation += 1
        sample = [float(value) for value in values]
        self.raw = sample.copy()
        self.filtered = sample.copy()
        self.derivative = [0.0] * EXPECTED_FLOAT_COUNT
        self.last_time = float(now)
        return sample

    def _output(self) -> list[float]:
        result = self.filtered.copy()
        if self.adaptive:
            for joint in range(1, 21):
                base = joint * 3
                local = self.filtered[base:base + 3]
                for axis in range(3):
                    result[base + axis] = result[axis] + sum(local[k] * self.basis[k][axis] for k in range(3))
        return result

    def _palm_basis(self, sample: list[float]) -> None:
        # One rigid palm frame for every joint: wrist translation and whole-hand
        # rotation no longer enter independent finger filters at different gains.
        x = [sample[15 + a] - sample[51 + a] for a in range(3)]
        y = [sample[27 + a] - sample[a] for a in range(3)]
        x_length = math.sqrt(sum(v * v for v in x))
        if x_length < 1e-6:
            return
        x = [v / x_length for v in x]
        dot = sum(x[a] * y[a] for a in range(3))
        y = [y[a] - dot * x[a] for a in range(3)]
        y_length = math.sqrt(sum(v * v for v in y))
        if y_length < 1e-6:
            return
        y = [v / y_length for v in y]
        z = [x[1]*y[2]-x[2]*y[1], x[2]*y[0]-x[0]*y[2], x[0]*y[1]-x[1]*y[0]]
        self.basis = (x, y, z)

    def apply(self, values: list, now: Optional[float] = None) -> list[float]:
        if len(values) != EXPECTED_FLOAT_COUNT:
            raise ValueError(f"XR skeleton length {len(values)} != {EXPECTED_FLOAT_COUNT}")
        sample = [float(value) for value in values]
        if not all(math.isfinite(value) for value in sample):
            raise ValueError("XR skeleton contains non-finite positions")
        timestamp = float(time.monotonic() if now is None else now)
        if not math.isfinite(timestamp):
            raise ValueError("XR timestamp must be finite")
        if self.last_time is not None and timestamp <= self.last_time:
            return self._output()
        if self.adaptive:
            self._palm_basis(sample)
            for joint in range(1, 21):
                base = joint * 3
                relative = [sample[base + a] - sample[a] for a in range(3)]
                sample[base:base + 3] = [sum(relative[a] * axis[a] for a in range(3)) for axis in self.basis]
        if self.raw is None or self.filtered is None or self.last_time is None:
            self._initialize(sample, timestamp)
            return self._output()

        elapsed = timestamp - self.last_time
        if not math.isfinite(elapsed) or elapsed <= 0.0 or elapsed > self.RESET_GAP_SEC:
            if elapsed <= 0.0:
                return self._output()
            self._initialize(sample, timestamp)
            return self._output()

        dt = max(0.002, min(0.05, elapsed))
        if self.adaptive:
            # Bound acquisition spikes before derivative adaptation; otherwise
            # a single bad joint opens the One Euro bandwidth exactly when it
            # should reject the measurement.
            for joint in range(1, 21):
                base = joint * 3
                innovation = [sample[base + axis] - self.raw[base + axis] for axis in range(3)]
                length = math.sqrt(sum(x * x for x in innovation))
                limit = 0.0015 + 0.3 * dt
                if length > limit:
                    for axis in range(3):
                        sample[base + axis] = self.raw[base + axis] + innovation[axis] * limit / length
        derivative_alpha = self._alpha(dt, self.DERIVATIVE_CUTOFF_HZ)
        raw_derivatives = [0.0] * EXPECTED_FLOAT_COUNT
        for index in range(EXPECTED_FLOAT_COUNT):
            raw_derivative = (sample[index] - self.raw[index]) / dt
            self.derivative[index] += derivative_alpha * (raw_derivative - self.derivative[index])
            raw_derivatives[index] = self.derivative[index]

        # One adaptive cutoff per 3-D joint avoids axis-dependent response.
        for joint in range(21):
            base = joint * 3
            speed = math.sqrt(sum(raw_derivatives[base + axis] ** 2 for axis in range(3)))
            cutoff = self.MIN_CUTOFF_HZ + self.BETA * speed
            if self.adaptive and joint > 0:
                minimum = self.thumb_cutoff_hz if joint <= 4 else self.finger_cutoff_hz
                # Local speed excludes whole-hand translation. Suppress the
                # noise floor, but open the bandwidth for deliberate motion.
                cutoff = minimum + self.local_beta * max(0.0, speed - 0.015)
                residual = math.sqrt(sum((sample[base + axis] - self.filtered[base + axis]) ** 2 for axis in range(3)))
                cutoff += min(2.0, 35.0 * max(0.0, residual - 0.004))
                cutoff = min(cutoff, 2.5 if joint <= 4 else 5.0)
            alpha = self._alpha(dt, cutoff)
            gain = 1.0
            if self.adaptive and joint > 0:
                deadband = 0.0015 if joint <= 4 else 0.0008
                gain = max(0.0, 1.0 - deadband / max(residual, 1e-12))
            for axis in range(3):
                index = base + axis
                self.filtered[index] += alpha * gain * (sample[index] - self.filtered[index])

        self.raw = sample
        self.last_time = timestamp
        return self._output()


def validate_packet(payload: Dict[str, Any]) -> Tuple[bool, str]:
    for key in REQUIRED_ARRAYS:
        value = payload.get(key)
        if not isinstance(value, list):
            return False, f"missing array {key}"
        if len(value) != EXPECTED_FLOAT_COUNT:
            return False, f"{key} length {len(value)} != {EXPECTED_FLOAT_COUNT}"
        if not all(isinstance(item, (int, float)) for item in value):
            return False, f"{key} contains non-number values"

    for key in OPTIONAL_ARRAYS:
        value = payload.get(key)
        if value is None:
            continue
        if not isinstance(value, list):
            return False, f"{key} is not an array"
        if len(value) != EXPECTED_FLOAT_COUNT:
            return False, f"{key} length {len(value)} != {EXPECTED_FLOAT_COUNT}"

    if payload.get("rightTracked") is False:
        return False, "rightTracked=false"

    return True, "ok"


def _joint(values: list, index: int) -> Tuple[float, float, float]:
    offset = index * 3
    return float(values[offset]), float(values[offset + 1]), float(values[offset + 2])


def _joint_angle_deg(a: Tuple[float, float, float], b: Tuple[float, float, float], c: Tuple[float, float, float]) -> float:
    ba = (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    bc = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
    ba_len = math.sqrt(sum(value * value for value in ba))
    bc_len = math.sqrt(sum(value * value for value in bc))
    if ba_len <= 1e-9 or bc_len <= 1e-9:
        return 180.0
    cosine = sum(ba[i] * bc[i] for i in range(3)) / (ba_len * bc_len)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _finger_curl_ratio(values: list, mcp: int, pip: int, tip: int, open_deg: float, close_deg: float) -> float:
    angle = _joint_angle_deg(_joint(values, mcp), _joint(values, pip), _joint(values, tip))
    return max(0.0, min(1.0, (open_deg - angle) / max(open_deg - close_deg, 1e-6)))


def classify_left_fist(
    payload: Dict[str, Any],
    previous: bool = False,
    on_threshold: float = 0.72,
    off_threshold: float = 0.58,
) -> Tuple[bool, float, list]:
    positions = payload.get("leftPositions")
    tracked = bool(payload.get("leftTracked", positions is not None))
    if not tracked or not isinstance(positions, list) or len(positions) != EXPECTED_FLOAT_COUNT:
        return False, 0.0, []
    curls = [
        _finger_curl_ratio(positions, mcp, pip, tip, open_deg, close_deg)
        for mcp, pip, tip, open_deg, close_deg in FINGER_CURL_CALIBRATION
    ]
    score = min(curls) if curls else 0.0
    threshold = off_threshold if previous else on_threshold
    return score >= threshold, score, curls


def right_operator_closure_prior(payload: Dict[str, Any]) -> Tuple[Optional[float], list]:
    """Return a human-intent prior, never a WA100 actual-closure measurement."""
    positions = payload.get("rightPositions")
    tracked = bool(payload.get("rightTracked", positions is not None))
    if not tracked or not isinstance(positions, list) or len(positions) != EXPECTED_FLOAT_COUNT:
        return None, []
    curls = [
        _finger_curl_ratio(positions, mcp, pip, tip, open_deg, close_deg)
        for mcp, pip, tip, open_deg, close_deg in FINGER_CURL_CALIBRATION
    ]
    return (sum(curls) / len(curls) if curls else None), curls


class UnityHandBridge:
    def __init__(
        self,
        tcp_host: str,
        tcp_port: int,
        udp_host: str,
        udp_port: int,
        verbose: bool,
        latest_only: bool,
        stale_timeout_sec: float,
        state_udp_host: str = "127.0.0.1",
        state_udp_port: int = 25002,
        fist_on_threshold: float = 0.72,
        fist_off_threshold: float = 0.58,
    ):
        self.tcp_host = tcp_host
        self.tcp_port = int(tcp_port)
        self.udp_host = udp_host
        self.udp_port = int(udp_port)
        self.verbose = verbose
        self.latest_only = latest_only
        self.stale_timeout_sec = max(float(stale_timeout_sec), 0.0)
        self.state_udp_host = state_udp_host
        self.state_udp_port = int(state_udp_port)
        self.fist_on_threshold = float(fist_on_threshold)
        self.fist_off_threshold = float(fist_off_threshold)
        self.left_fist = False
        self.right_skeleton_filter = XrSkeletonOneEuroFilter()
        self.left_skeleton_filter = XrSkeletonOneEuroFilter(adaptive=False)
        self.stop_event = threading.Event()
        self.forwarded = 0
        self.dropped = 0
        self.superseded = 0
        self.last_frame_id: Optional[int] = None
        self.last_log_time = 0.0

    def serve_forever(self) -> None:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.tcp_host, self.tcp_port))
        server.listen(1)
        server.settimeout(0.5)

        print(f"[UnityHandBridge] TCP listen {self.tcp_host}:{self.tcp_port}")
        print(f"[UnityHandBridge] UDP forward {self.udp_host}:{self.udp_port}")
        print(f"[UnityHandBridge] H5 gesture state UDP {self.state_udp_host}:{self.state_udp_port}")
        print(f"[UnityHandBridge] latest_only={int(self.latest_only)} stale_timeout_sec={self.stale_timeout_sec:.3f}")
        print(f"[UnityHandBridge] expected adb reverse tcp:{self.tcp_port} tcp:{self.tcp_port}")

        try:
            while not self.stop_event.is_set():
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                print(f"[UnityHandBridge] Unity connected from {addr[0]}:{addr[1]}")
                try:
                    self._read_client(client, udp_sock)
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    self._reset_skeleton_filters()
                    print("[UnityHandBridge] Unity disconnected")
        finally:
            server.close()
            udp_sock.close()

    def _read_client(self, client: socket.socket, udp_sock: socket.socket) -> None:
        client.settimeout(0.5)
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = client.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break

            buffer += chunk
            ready_lines = []
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                raw = raw.strip()
                if raw:
                    ready_lines.append(raw)

            if not ready_lines:
                continue

            if self.latest_only and len(ready_lines) > 1:
                dropped = len(ready_lines) - 1
                self.dropped += dropped
                self.superseded += dropped
                if self.verbose:
                    print(f"[UnityHandBridge] dropped {dropped} superseded TCP packets from one read")
                ready_lines = ready_lines[-1:]

            for raw in ready_lines:
                self._handle_line(raw, udp_sock)

    def _handle_line(self, raw: bytes, udp_sock: socket.socket) -> None:
        if not raw:
            return
        recv_monotonic = time.monotonic()
        recv_wall = time.time()

        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._drop(f"json decode failed: {exc}")
            return

        if payload.get("rightTracked") is False:
            self.right_skeleton_filter.reset()
        if payload.get("leftTracked") is False:
            self.left_skeleton_filter.reset()

        ok, reason = validate_packet(payload)
        if not ok:
            self._drop(reason)
            return

        frame_id = int(payload.get("frameId", -1))
        if self.last_frame_id is not None and frame_id >= 0 and frame_id <= self.last_frame_id:
            self._drop(f"non-monotonic frameId {frame_id} <= {self.last_frame_id}")
            return

        age_sec = time.monotonic() - recv_monotonic
        if self.stale_timeout_sec > 0 and age_sec > self.stale_timeout_sec:
            self._drop(f"stale in bridge {age_sec:.3f}s")
            return

        try:
            self._filter_xr_skeletons(payload, recv_monotonic)
        except ValueError as exc:
            self._reset_skeleton_filters()
            self._drop(str(exc))
            return

        send_wall = time.time()
        # This listener is dedicated to PICO/Unity XR Hands.  Stamp provenance
        # explicitly so the shared WA100 UDP receiver cannot confuse the packet
        # with the legacy Hi5/data-glove schema that uses the same array names.
        payload["source"] = "xr-hands"
        payload.setdefault("jointFormat", "unity-xr-hands-21x3")
        payload["bridgeRecvTime"] = recv_wall
        payload["bridgeSendTime"] = send_wall
        payload["bridgeAgeSec"] = max(0.0, send_wall - recv_wall)
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

        udp_sock.sendto(encoded, (self.udp_host, self.udp_port))
        self.left_fist, fist_score, finger_curls = classify_left_fist(
            payload,
            previous=self.left_fist,
            on_threshold=self.fist_on_threshold,
            off_threshold=self.fist_off_threshold,
        )
        right_intent_closure, right_intent_curls = right_operator_closure_prior(payload)
        gesture_state = {
            "type": "xr_hand_gesture_state",
            "source": "h5:xr-hands",
            "frame_id": frame_id,
            "time": send_wall,
            "left_tracked": bool(payload.get("leftTracked", payload.get("leftPositions") is not None)),
            "left_fist": bool(self.left_fist),
            "left_fist_score": float(fist_score),
            "left_finger_curls": [float(value) for value in finger_curls],
            "deadman": 1 if self.left_fist else 0,
            # Auxiliary intent only.  PerceptionAssistMonitor deliberately does
            # not use this as robot closure or grasp evidence.
            "right_operator_intent_closure_prior": right_intent_closure,
            "right_operator_intent_finger_curls": [float(value) for value in right_intent_curls],
            "right_prior_is_robot_actual_closure": False,
        }
        udp_sock.sendto(
            json.dumps(gesture_state, separators=(",", ":")).encode("utf-8"),
            (self.state_udp_host, self.state_udp_port),
        )
        self.forwarded += 1
        self.last_frame_id = frame_id
        self._log_status(force=self.verbose)

    def _filter_xr_skeletons(self, payload: Dict[str, Any], now: float) -> None:
        filtered_sides = []
        for side, skeleton_filter in (
            ("right", self.right_skeleton_filter),
            ("left", self.left_skeleton_filter),
        ):
            key = f"{side}Positions"
            raw = payload.get(key)
            tracked = bool(payload.get(f"{side}Tracked", raw is not None))
            if not tracked or raw is None:
                skeleton_filter.reset()
                continue
            payload[f"{key}Raw"] = list(raw)
            payload[key] = skeleton_filter.apply(raw, now)
            filtered_sides.append(side)
        payload["xrSkeletonFilter"] = {
            "name": "one_euro_palm_local_deadband_v4",
            "stage": "post_xr_acquisition_pre_retarget",
            "sides": filtered_sides,
            "min_cutoff_hz": XrSkeletonOneEuroFilter.MIN_CUTOFF_HZ,
            "beta": XrSkeletonOneEuroFilter.BETA,
            "derivative_cutoff_hz": XrSkeletonOneEuroFilter.DERIVATIVE_CUTOFF_HZ,
            "thumb_cutoff_hz": self.right_skeleton_filter.thumb_cutoff_hz,
            "finger_cutoff_hz": self.right_skeleton_filter.finger_cutoff_hz,
            "local_beta": self.right_skeleton_filter.local_beta,
            "thumb_deadband_m": 0.0015,
            "finger_deadband_m": 0.0008,
            "right_generation": self.right_skeleton_filter.generation,
            "left_generation": self.left_skeleton_filter.generation,
        }

    def _reset_skeleton_filters(self) -> None:
        self.right_skeleton_filter.reset()
        self.left_skeleton_filter.reset()

    def _drop(self, reason: str) -> None:
        self.dropped += 1
        if self.verbose or self.dropped <= 5 or self.dropped % 100 == 0:
            print(f"[UnityHandBridge] drop #{self.dropped}: {reason}")

    def _log_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_log_time < 2.0:
            return
        self.last_log_time = now
        print(
            "[UnityHandBridge] forwarded="
            f"{self.forwarded} dropped={self.dropped} superseded={self.superseded} lastFrame={self.last_frame_id}"
        )

def main() -> None:
    parser = argparse.ArgumentParser(description="Unity right-hand TCP to local UDP bridge")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=5006)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=25001)
    parser.add_argument("--state-udp-host", default="127.0.0.1")
    parser.add_argument("--state-udp-port", type=int, default=25002)
    parser.add_argument("--fist-on-threshold", type=float, default=0.72)
    parser.add_argument("--fist-off-threshold", type=float, default=0.58)
    parser.add_argument("--no-latest-only", dest="latest_only", action="store_false", help="Forward every valid TCP packet instead of keeping only the latest packet from each read.")
    parser.add_argument("--stale-timeout-sec", type=float, default=0.40, help="Drop packets that wait inside this bridge longer than this many seconds.")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--thumb-cutoff-hz", type=float, default=0.18)
    parser.add_argument("--finger-cutoff-hz", type=float, default=0.35)
    parser.add_argument("--local-beta", type=float, default=5.0)
    parser.set_defaults(latest_only=True)
    args = parser.parse_args()

    bridge = UnityHandBridge(
        args.tcp_host,
        args.tcp_port,
        args.udp_host,
        args.udp_port,
        args.verbose,
        args.latest_only,
        args.stale_timeout_sec,
        args.state_udp_host,
        args.state_udp_port,
        args.fist_on_threshold,
        args.fist_off_threshold,
    )
    bridge.right_skeleton_filter = XrSkeletonOneEuroFilter(
        thumb_cutoff_hz=args.thumb_cutoff_hz,
        finger_cutoff_hz=args.finger_cutoff_hz, local_beta=args.local_beta)
    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        bridge.stop_event.set()
        print("\n[UnityHandBridge] stopped")


if __name__ == "__main__":
    main()
