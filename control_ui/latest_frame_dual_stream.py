#!/usr/bin/env python
"""Capture two USB cameras with latest-frame semantics and publish RTSP.

Each camera is read by its own thread. The thread continuously overwrites the
most recent frame, so old frames are intentionally dropped instead of queued.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-index", type=int, required=True)
    parser.add_argument("--right-index", type=int, required=True)
    parser.add_argument("--left-name", default="left")
    parser.add_argument("--right-name", default="right")
    parser.add_argument("--left-mirror", action="store_true")
    parser.add_argument("--right-mirror", action="store_true")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--layout", choices=("vstack", "hstack"), default="vstack")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--status-json", required=True)
    parser.add_argument("--ffmpeg-log", required=True)
    parser.add_argument("--crf", type=int, default=26)
    return parser.parse_args()


def fourcc_to_int(text: str) -> int:
    normalized = (text or "MJPG").upper()
    if len(normalized) != 4:
        raise ValueError(f"fourcc must be four characters, got {text!r}")
    return cv2.VideoWriter_fourcc(*normalized)


def fourcc_to_text(value: float) -> str:
    code = int(value)
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))


@dataclass
class CameraState:
    label: str
    index: int
    width: int
    height: int
    fps: float
    fourcc: str
    lock: threading.Lock
    latest_frame: Optional[np.ndarray] = None
    latest_time: float = 0.0
    read_count: int = 0
    error: str = ""


class CameraReader:
    def __init__(self, state: CameraState, stop_event: threading.Event):
        self.state = state
        self.stop_event = stop_event
        self.thread = threading.Thread(target=self._run, name=f"camera-{state.label}", daemon=True)
        self.capture: Optional[cv2.VideoCapture] = None

    def start(self) -> None:
        self.thread.start()

    def join(self, timeout: float = 2.0) -> None:
        self.thread.join(timeout=timeout)

    def _open_capture(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.state.index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(self.state.index)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera index {self.state.index}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.state.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.state.height)
        cap.set(cv2.CAP_PROP_FPS, self.state.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc_to_int(self.state.fourcc))
        return cap

    def _run(self) -> None:
        try:
            self.capture = self._open_capture()
            actual_w = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
            actual_fourcc = fourcc_to_text(self.capture.get(cv2.CAP_PROP_FOURCC))
            print(
                f"[{self.state.label}] index={self.state.index} "
                f"actual={actual_w}x{actual_h}@{actual_fps:.1f} fourcc={actual_fourcc}",
                flush=True,
            )

            while not self.stop_event.is_set():
                ok, frame = self.capture.read()
                now = time.monotonic()
                if not ok or frame is None:
                    time.sleep(0.002)
                    continue
                with self.state.lock:
                    self.state.latest_frame = frame
                    self.state.latest_time = now
                    self.state.read_count += 1
        except Exception as exc:  # noqa: BLE001 - keep camera thread failure visible.
            with self.state.lock:
                self.state.error = str(exc)
            print(f"[{self.state.label}] ERROR: {exc}", file=sys.stderr, flush=True)
            self.stop_event.set()
        finally:
            if self.capture is not None:
                self.capture.release()


def get_latest(state: CameraState) -> tuple[Optional[np.ndarray], float, int, str]:
    with state.lock:
        frame = None if state.latest_frame is None else state.latest_frame.copy()
        return frame, state.latest_time, state.read_count, state.error


def resize_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def prepare_frame(frame: np.ndarray, width: int, height: int, mirror: bool) -> np.ndarray:
    prepared = resize_frame(frame, width, height)
    return cv2.flip(prepared, 1) if mirror else prepared


def build_ffmpeg_command(args: argparse.Namespace, out_w: int, out_h: int) -> list[str]:
    fps_int = max(1, int(round(args.fps)))
    x264_params = (
        f"keyint={fps_int}:min-keyint={fps_int}:scenecut=0:"
        "sync-lookahead=0:rc-lookahead=0:sliced-threads=1:repeat-headers=1"
    )
    return [
        args.ffmpeg,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{out_w}x{out_h}",
        "-r",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-x264-params",
        x264_params,
        "-crf",
        str(args.crf),
        "-pix_fmt",
        "yuv420p",
        "-r",
        str(args.fps),
        "-g",
        str(fps_int),
        "-bf",
        "0",
        "-muxdelay",
        "0",
        "-muxpreload",
        "0",
        "-f",
        "rtsp",
        "-rtsp_transport",
        "tcp",
        args.output,
    ]


def write_status(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    tmp.replace(path)


def main() -> int:
    args = parse_args()
    cv2.setNumThreads(1)

    stop_event = threading.Event()

    def _stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    left = CameraState(
        label=args.left_name,
        index=args.left_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        lock=threading.Lock(),
    )
    right = CameraState(
        label=args.right_name,
        index=args.right_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        lock=threading.Lock(),
    )

    readers = [CameraReader(left, stop_event), CameraReader(right, stop_event)]
    for reader in readers:
        reader.start()

    out_w = args.width if args.layout == "vstack" else args.width * 2
    out_h = args.height * 2 if args.layout == "vstack" else args.height
    ffmpeg_log_path = Path(args.ffmpeg_log)
    ffmpeg_log_path.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg_log = ffmpeg_log_path.open("ab", buffering=0)
    ffmpeg_cmd = build_ffmpeg_command(args, out_w, out_h)
    proc = subprocess.Popen(
        ffmpeg_cmd,
        stdin=subprocess.PIPE,
        stdout=ffmpeg_log,
        stderr=ffmpeg_log,
        bufsize=0,
    )

    status_path = Path(args.status_json)
    write_status(
        status_path,
        {
            "ok": True,
            "pythonPid": os.getpid(),
            "ffmpegPid": proc.pid,
            "output": args.output,
            "layout": args.layout,
            "leftMirror": bool(args.left_mirror),
            "rightMirror": bool(args.right_mirror),
            "size": f"{out_w}x{out_h}",
            "fps": args.fps,
            "startedAt": time.time(),
        },
    )
    print(f"[stream] ffmpeg pid={proc.pid} output={args.output} size={out_w}x{out_h}", flush=True)

    period = 1.0 / max(args.fps, 1.0)
    next_tick = time.perf_counter()
    written = 0
    last_report_time = time.monotonic()
    last_left_count = 0
    last_right_count = 0
    last_written = 0

    try:
        while not stop_event.is_set():
            if proc.poll() is not None:
                print(f"[stream] ffmpeg exited code={proc.returncode}", file=sys.stderr, flush=True)
                return 2

            left_frame, left_ts, left_count, left_error = get_latest(left)
            right_frame, right_ts, right_count, right_error = get_latest(right)
            if left_error or right_error:
                print(f"[stream] camera error: left={left_error!r} right={right_error!r}", file=sys.stderr, flush=True)
                return 2
            if left_frame is None or right_frame is None:
                time.sleep(0.005)
                continue

            left_frame = prepare_frame(left_frame, args.width, args.height, args.left_mirror)
            right_frame = prepare_frame(right_frame, args.width, args.height, args.right_mirror)
            if args.layout == "vstack":
                combined = np.vstack((left_frame, right_frame))
            else:
                combined = np.hstack((left_frame, right_frame))

            try:
                assert proc.stdin is not None
                proc.stdin.write(np.ascontiguousarray(combined).tobytes())
            except BrokenPipeError:
                print("[stream] ffmpeg pipe closed", file=sys.stderr, flush=True)
                return 2
            written += 1

            now = time.monotonic()
            if now - last_report_time >= 2.0:
                elapsed = now - last_report_time
                left_read_fps = (left_count - last_left_count) / elapsed
                right_read_fps = (right_count - last_right_count) / elapsed
                write_fps = (written - last_written) / elapsed
                left_age_ms = (now - left_ts) * 1000.0
                right_age_ms = (now - right_ts) * 1000.0
                print(
                    f"[stats] read={left_read_fps:.1f}/{right_read_fps:.1f} fps "
                    f"write={write_fps:.1f} fps age={left_age_ms:.0f}/{right_age_ms:.0f} ms",
                    flush=True,
                )
                write_status(
                    status_path,
                    {
                        "ok": True,
                        "pythonPid": os.getpid(),
                        "ffmpegPid": proc.pid,
                        "output": args.output,
                        "layout": args.layout,
                        "leftMirror": bool(args.left_mirror),
                        "rightMirror": bool(args.right_mirror),
                        "size": f"{out_w}x{out_h}",
                        "fps": args.fps,
                        "writtenFrames": written,
                        "leftReadFps": left_read_fps,
                        "rightReadFps": right_read_fps,
                        "writeFps": write_fps,
                        "leftAgeMs": left_age_ms,
                        "rightAgeMs": right_age_ms,
                        "updatedAt": time.time(),
                    },
                )
                last_report_time = now
                last_left_count = left_count
                last_right_count = right_count
                last_written = written

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()
    finally:
        stop_event.set()
        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        for reader in readers:
            reader.join()
        ffmpeg_log.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
