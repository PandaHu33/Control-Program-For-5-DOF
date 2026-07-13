#!/usr/bin/env python3
"""Send zero-motion keyboard frames through the real WebSocket/UDP path.

This is a software-path probe for an unpowered arm.  A synthetic telemetry
frame is injected only into the local Windows bridge so its stale-telemetry
guard can be exercised without pretending that the Jetson has motor feedback.
The low-level node's startup-homing interlock remains unchanged.
"""

import argparse
import base64
import binascii
import csv
import json
import os
import socket
import struct
import time
import urllib.request
from pathlib import Path


FRAME_SIZE = 293
CRC_OFFSET = 289
CLIENT_ID = "4c4154454e435950524f424530303031"


def pack_frame(ind, source_time_ms, note, client_id=CLIENT_ID):
    body = b"".join([
        struct.pack("<I", int(ind) & 0xFFFFFFFF),
        struct.pack("<Q", int(source_time_ms) & 0xFFFFFFFFFFFFFFFF),
        struct.pack("<7f", *([0.0] * 7)),
        struct.pack("<7f", *([0.0] * 7)),
        struct.pack("<7f", *([0.0] * 7)),
        struct.pack("<6f", *([0.0] * 6)),
        struct.pack("<6f", *([0.0] * 6)),
        struct.pack("<B", 0),
        struct.pack("<16f", *([0.0] * 16)),
        bytes.fromhex(client_id),
        note.encode("utf-8")[:64].ljust(64, b"\x00"),
    ])
    if len(body) != CRC_OFFSET:
        raise RuntimeError("unexpected H5 frame layout")
    frame = body + struct.pack("<I", binascii.crc32(body) & 0xFFFFFFFF)
    if len(frame) != FRAME_SIZE:
        raise RuntimeError("unexpected H5 frame size")
    return frame


def post_json(url, body):
    request = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def websocket_connect(host, port):
    sock = socket.create_connection((host, port), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        "GET /udp HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    ) % (host, port, key)
    sock.sendall(request.encode("ascii"))
    response = sock.recv(4096)
    if not response.startswith(b"HTTP/1.1 101"):
        raise RuntimeError("WebSocket handshake failed: %r" % response[:120])
    return sock


def websocket_send_binary(sock, payload):
    mask = os.urandom(4)
    length = len(payload)
    if length < 126:
        header = bytes([0x82, 0x80 | length])
    elif length <= 0xFFFF:
        header = bytes([0x82, 0xFE]) + struct.pack("!H", length)
    else:
        header = bytes([0x82, 0xFF]) + struct.pack("!Q", length)
    masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
    sock.sendall(header + mask + masked)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--api-port", type=int, default=8090)
    parser.add_argument("--ws-port", type=int, default=8080)
    parser.add_argument("--telemetry-port", type=int, default=14550)
    parser.add_argument("--period-ms", type=int, nargs="+", default=[100, 50, 20])
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.samples < 3:
        parser.error("--samples must be at least 3")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = args.output or Path(__file__).with_name("latency_logs") / ("probe_%s.csv" % stamp)
    output.parent.mkdir(parents=True, exist_ok=True)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ws = websocket_connect(args.host, args.ws_port)
    api = "http://%s:%d/api/arm/control" % (args.host, args.api_port)
    rows = []
    ind = int(time.time() * 1000) & 0xFFFFFFFF
    try:
        udp.sendto(pack_frame(0, int(time.time() * 1000), "latency-probe-telemetry"),
                   (args.host, args.telemetry_port))
        time.sleep(0.05)
        result = post_json(api, {"mode": "keyboard", "client_id": CLIENT_ID})
        if not result.get("ok"):
            raise RuntimeError("keyboard authority rejected: %s" % result.get("message"))

        for period_ms in args.period_ms:
            deadline = time.perf_counter()
            for sample in range(args.samples):
                udp.sendto(pack_frame(0, int(time.time() * 1000), "latency-probe-telemetry"),
                           (args.host, args.telemetry_port))
                source_time_ms = int(time.time() * 1000)
                event = "press" if sample == 0 else ("release" if sample == args.samples - 1 else "held")
                websocket_send_binary(ws, pack_frame(ind, source_time_ms, "ui:keyboard"))
                rows.append({
                    "ind": ind,
                    "source_time_ms": source_time_ms,
                    "period_ms": period_ms,
                    "event": event,
                })
                ind = (ind + 1) & 0xFFFFFFFF
                deadline += period_ms / 1000.0
                time.sleep(max(0.0, deadline - time.perf_counter()))
    finally:
        try:
            post_json(api, {"mode": "idle", "client_id": CLIENT_ID})
        except Exception:
            pass
        try:
            ws.close()
        finally:
            udp.close()

    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["ind", "source_time_ms", "period_ms", "event"])
        writer.writeheader()
        writer.writerows(rows)
    print("samples=%d manifest=%s" % (len(rows), output.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
