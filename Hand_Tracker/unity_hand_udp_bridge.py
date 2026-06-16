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
import json
import socket
import threading
import time
from typing import Any, Dict, Optional, Tuple


REQUIRED_ARRAYS = ("rightPositions", "rightRotations")
OPTIONAL_ARRAYS = ("leftPositions", "leftRotations")
EXPECTED_FLOAT_COUNT = 21 * 3


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
    def __init__(self, tcp_host: str, tcp_port: int, udp_host: str, udp_port: int, verbose: bool):
        self.tcp_host = tcp_host
        self.tcp_port = int(tcp_port)
        self.udp_host = udp_host
        self.udp_port = int(udp_port)
        self.verbose = verbose
        self.stop_event = threading.Event()
        self.forwarded = 0
        self.dropped = 0
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
            while b"\n" in buffer:
                raw, buffer = buffer.split(b"\n", 1)
                self._handle_line(raw.strip(), udp_sock)

    def _handle_line(self, raw: bytes, udp_sock: socket.socket) -> None:
        if not raw:
            return

        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._drop(f"json decode failed: {exc}")
            return

        ok, reason = validate_packet(payload)
        if not ok:
            self._drop(reason)
            return

        udp_sock.sendto(raw, (self.udp_host, self.udp_port))
        self.forwarded += 1
        self.last_frame_id = int(payload.get("frameId", -1))
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
            f"{self.forwarded} dropped={self.dropped} lastFrame={self.last_frame_id}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Unity right-hand TCP to local UDP bridge")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=5006)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=25001)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    bridge = UnityHandBridge(args.tcp_host, args.tcp_port, args.udp_host, args.udp_port, args.verbose)
    try:
        bridge.serve_forever()
    except KeyboardInterrupt:
        bridge.stop_event.set()
        print("\n[UnityHandBridge] stopped")


if __name__ == "__main__":
    main()
