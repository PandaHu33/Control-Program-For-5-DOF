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
    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        bridge.stop_event.set()
        print("\n[UnityHandBridge] stopped")


if __name__ == "__main__":
    main()
