#!/usr/bin/env python3
"""Bridge Unity XR hand JSON from adb-reversed TCP to local UDP.

Typical setup:
  adb reverse tcp:5006 tcp:5006
  python Hand_Tracker/unity_hand_udp_bridge.py
  wa100-sdk-publish/examples/udp_receiver_unity.exe right

Unity connects to 127.0.0.1:5006 on the headset. adb reverse delivers that
TCP stream to this script on the PC, and this script forwards valid JSON
packets to udp_receiver_unity.cpp at 127.0.0.1:25001.
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


REQUIRED_ARRAYS = ("rightPositions", "rightRotations")
OPTIONAL_ARRAYS = ("leftPositions", "leftRotations")
EXPECTED_FLOAT_COUNT = 21 * 3
HAND_LATENCY_LOG_FIELDS = [
    "source",
    "frame_id",
    "unity_time",
    "python_recv_time",
    "bridge_send_time",
    "udp_host",
    "udp_port",
    "right_tracked",
    "stale",
    "dropped",
    "drop_reason",
    "forwarded_count",
    "dropped_count",
    "superseded_count",
]


def default_latency_log_path() -> Path:
    return Path(__file__).resolve().with_name("logs") / "unity_hand_latency.csv"


def resolve_latency_log_path(value: str) -> Optional[Path]:
    if not value:
        return None
    path = Path(value)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


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
        latency_log: str,
    ):
        self.tcp_host = tcp_host
        self.tcp_port = int(tcp_port)
        self.udp_host = udp_host
        self.udp_port = int(udp_port)
        self.verbose = verbose
        self.latest_only = latest_only
        self.stale_timeout_sec = max(float(stale_timeout_sec), 0.0)
        self.latency_log_path = resolve_latency_log_path(latency_log)
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
                self._write_latency_row(
                    {
                        "source": "xr-hands",
                        "dropped": 1,
                        "drop_reason": "superseded_by_latest_packet",
                    }
                )
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
            self._drop(f"json decode failed: {exc}", recv_wall=recv_wall)
            return

        ok, reason = validate_packet(payload)
        if not ok:
            self._drop(reason, payload=payload, recv_wall=recv_wall)
            return

        frame_id = int(payload.get("frameId", -1))
        if self.last_frame_id is not None and frame_id >= 0 and frame_id <= self.last_frame_id:
            self._drop(f"non-monotonic frameId {frame_id} <= {self.last_frame_id}", payload=payload, recv_wall=recv_wall)
            return

        age_sec = time.monotonic() - recv_monotonic
        if self.stale_timeout_sec > 0 and age_sec > self.stale_timeout_sec:
            self._drop(f"stale in bridge {age_sec:.3f}s", payload=payload, recv_wall=recv_wall, stale=True)
            return

        send_wall = time.time()
        payload["bridgeRecvTime"] = recv_wall
        payload["bridgeSendTime"] = send_wall
        payload["bridgeAgeSec"] = max(0.0, send_wall - recv_wall)
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

        udp_sock.sendto(encoded, (self.udp_host, self.udp_port))
        self.forwarded += 1
        self.last_frame_id = frame_id
        self._write_latency_row(
            {
                "source": payload.get("source", "xr-hands"),
                "frame_id": frame_id,
                "unity_time": payload.get("unityTime", ""),
                "python_recv_time": recv_wall,
                "bridge_send_time": send_wall,
                "udp_host": self.udp_host,
                "udp_port": self.udp_port,
                "right_tracked": 1 if payload.get("rightTracked", True) else 0,
                "stale": 0,
                "dropped": 0,
                "forwarded_count": self.forwarded,
                "dropped_count": self.dropped,
                "superseded_count": self.superseded,
            }
        )
        self._log_status(force=self.verbose)

    def _drop(
        self,
        reason: str,
        payload: Optional[Dict[str, Any]] = None,
        recv_wall: Optional[float] = None,
        stale: bool = False,
    ) -> None:
        self.dropped += 1
        if self.verbose or self.dropped <= 5 or self.dropped % 100 == 0:
            print(f"[UnityHandBridge] drop #{self.dropped}: {reason}")
        payload = payload or {}
        self._write_latency_row(
            {
                "source": payload.get("source", "xr-hands"),
                "frame_id": payload.get("frameId", ""),
                "unity_time": payload.get("unityTime", ""),
                "python_recv_time": recv_wall if recv_wall is not None else "",
                "bridge_send_time": "",
                "udp_host": self.udp_host,
                "udp_port": self.udp_port,
                "right_tracked": 1 if payload.get("rightTracked", True) else 0,
                "stale": 1 if stale else 0,
                "dropped": 1,
                "drop_reason": reason,
                "forwarded_count": self.forwarded,
                "dropped_count": self.dropped,
                "superseded_count": self.superseded,
            }
        )

    def _log_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_log_time < 2.0:
            return
        self.last_log_time = now
        print(
            "[UnityHandBridge] forwarded="
            f"{self.forwarded} dropped={self.dropped} superseded={self.superseded} lastFrame={self.last_frame_id}"
        )

    def _write_latency_row(self, row: Dict[str, Any]) -> None:
        if self.latency_log_path is None:
            return
        out = {key: row.get(key, "") for key in HAND_LATENCY_LOG_FIELDS}
        needs_header = not self.latency_log_path.exists() or self.latency_log_path.stat().st_size == 0
        with self.latency_log_path.open("a", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=HAND_LATENCY_LOG_FIELDS)
            if needs_header:
                writer.writeheader()
            writer.writerow(out)


def main() -> None:
    parser = argparse.ArgumentParser(description="Unity right-hand TCP to local UDP bridge")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=5006)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=25001)
    parser.add_argument("--no-latest-only", dest="latest_only", action="store_false", help="Forward every valid TCP packet instead of keeping only the latest packet from each read.")
    parser.add_argument("--stale-timeout-sec", type=float, default=0.40, help="Drop packets that wait inside this bridge longer than this many seconds.")
    parser.add_argument("--latency-log", default=str(default_latency_log_path()), help="CSV path for XR hand bridge latency/drop observations. Empty disables logging.")
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
        args.latency_log,
    )
    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        bridge.stop_event.set()
        print("\n[UnityHandBridge] stopped")


if __name__ == "__main__":
    main()
