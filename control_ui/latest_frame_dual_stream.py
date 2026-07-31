#!/usr/bin/env python
"""Capture two USB cameras with latest-frame semantics and publish RTSP.

Each camera is read by its own thread. The thread continuously overwrites the
most recent frame, so old frames are intentionally dropped instead of queued.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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
    parser.add_argument("--left-vertical-flip", action="store_true")
    parser.add_argument("--right-vertical-flip", action="store_true")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--layout", choices=("vstack", "hstack"), default="vstack")
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--status-json", required=True)
    parser.add_argument("--ffmpeg-log", required=True)
    parser.add_argument("--crf", type=int, default=26)
    parser.add_argument("--service-host", default="0.0.0.0")
    parser.add_argument("--service-port", type=int, default=8092)
    parser.add_argument("--recording-root", default="recordings")
    return parser.parse_args()


def fourcc_to_int(text: str) -> int:
    normalized = (text or "MJPG").upper()
    if len(normalized) != 4:
        raise ValueError(f"fourcc must be four characters, got {text!r}")
    return cv2.VideoWriter_fourcc(*normalized)


def fourcc_to_text(value: float) -> str:
    code = int(value)
    return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))


def apply_vertical_flip(frame: np.ndarray, enabled: bool) -> np.ndarray:
    return cv2.flip(frame, 0) if enabled else frame


@dataclass
class CameraState:
    label: str
    index: int
    width: int
    height: int
    fps: float
    fourcc: str
    vertical_flip: bool
    lock: threading.Lock
    latest_frame: Optional[np.ndarray] = None
    latest_time: float = 0.0
    latest_utc_ns: int = 0
    latest_monotonic_ns: int = 0
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
                frame = apply_vertical_flip(frame, self.state.vertical_flip)
                with self.state.lock:
                    self.state.latest_frame = frame
                    self.state.latest_time = now
                    self.state.latest_utc_ns = time.time_ns()
                    self.state.latest_monotonic_ns = time.monotonic_ns()
                    self.state.read_count += 1
        except Exception as exc:  # noqa: BLE001 - keep camera thread failure visible.
            with self.state.lock:
                self.state.error = str(exc)
            print(f"[{self.state.label}] ERROR: {exc}", file=sys.stderr, flush=True)
            self.stop_event.set()
        finally:
            if self.capture is not None:
                self.capture.release()


def get_latest(state: CameraState) -> tuple[Optional[np.ndarray], float, int, int, int, str]:
    with state.lock:
        frame = None if state.latest_frame is None else state.latest_frame.copy()
        return (
            frame, state.latest_time, state.latest_utc_ns,
            state.latest_monotonic_ns, state.read_count, state.error,
        )


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


class DualCameraRecorder:
    """Own two FFmpeg encoders and timestamp sidecars for one session."""

    def __init__(self, root: Path, ffmpeg: str, width: int, height: int, fps: float, crf: int):
        self.root = root.resolve()
        self.ffmpeg = ffmpeg
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.crf = int(crf)
        self.lock = threading.RLock()
        self.session_id: Optional[str] = None
        self.directory: Optional[Path] = None
        self.processes: dict[str, subprocess.Popen] = {}
        self.files = {}
        self.writers = {}
        self.last_counts = {"left": -1, "right": -1}
        self.frame_counts = {"left": 0, "right": 0}
        self.errors: list[str] = []
        self._recover_partial_recordings()

    def _recover_partial_recordings(self) -> None:
        """Best-effort remux of finalized MKV files left by an interrupted backend."""
        if not self.root.exists():
            return
        for partial in self.root.glob("*/*/*.partial.mkv"):
            final = partial.with_name(partial.name.replace(".partial.mkv", ".mp4"))
            if final.exists() or partial.stat().st_size == 0:
                continue
            try:
                completed = subprocess.run(
                    [self.ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-i", str(partial), "-c", "copy", str(final)],
                    capture_output=True, text=True, timeout=30,
                )
                if completed.returncode == 0:
                    partial.unlink(missing_ok=True)
                else:
                    print(f"[camera-recorder] recovery failed for {partial}: {completed.stderr.strip()}", file=sys.stderr)
            except Exception as exc:  # noqa: BLE001
                print(f"[camera-recorder] recovery failed for {partial}: {exc}", file=sys.stderr)

    def _encoder_command(self, output: Path) -> list[str]:
        return [
            self.ffmpeg, "-hide_banner", "-loglevel", "warning", "-f", "rawvideo",
            "-pix_fmt", "bgr24", "-s", f"{self.width}x{self.height}", "-r", str(self.fps),
            "-i", "-", "-an", "-c:v", "libx264", "-preset", "ultrafast", "-crf", str(self.crf),
            "-pix_fmt", "yuv420p", "-f", "matroska", str(output),
        ]

    def start(self, date: str, session_id: str, started_at_ns: int) -> dict:
        safe_date = "".join(ch for ch in str(date) if ch.isdigit())
        safe_id = "".join(ch for ch in str(session_id) if ch.isalnum() or ch in "_.-")
        if not safe_date or not safe_id or safe_id != session_id:
            return {"ok": False, "message": "invalid recording identity"}
        with self.lock:
            if self.session_id:
                return {"ok": False, "message": "camera recording is already active"}
            directory = (self.root / safe_date / safe_id).resolve()
            if self.root not in directory.parents:
                return {"ok": False, "message": "recording path escaped configured root"}
            directory.mkdir(parents=True, exist_ok=True)
            try:
                for side in ("left", "right"):
                    partial = directory / f"{side}.partial.mkv"
                    log = (directory / f"{side}_encoder.log").open("ab", buffering=0)
                    proc = subprocess.Popen(
                        self._encoder_command(partial), stdin=subprocess.PIPE, stdout=log, stderr=log, bufsize=0
                    )
                    fh = (directory / f"{side}_frames.csv").open("w", newline="", encoding="utf-8")
                    writer = csv.DictWriter(
                        fh,
                        ["frame_index", "capture_utc_ns", "capture_monotonic_ns", "encoded_pts_ns", "drop_count"],
                    )
                    writer.writeheader()
                    self.processes[side] = proc
                    self.files[side] = (fh, log)
                    self.writers[side] = writer
                self.session_id, self.directory = safe_id, directory
                self.last_counts = {"left": -1, "right": -1}
                self.frame_counts = {"left": 0, "right": 0}
                self.errors = []
                return {"ok": True, "session_id": safe_id, "started_at_ns": int(started_at_ns)}
            except Exception as exc:  # noqa: BLE001
                self.errors.append(str(exc))
                self._close_encoders()
                self.session_id = None
                self.directory = None
                self.processes, self.files, self.writers = {}, {}, {}
                return {"ok": False, "message": str(exc)}

    def write(self, side: str, frame: np.ndarray, utc_ns: int, monotonic_ns: int, read_count: int) -> None:
        with self.lock:
            if not self.session_id or read_count == self.last_counts.get(side):
                return
            proc = self.processes.get(side)
            if proc is None or proc.poll() is not None or proc.stdin is None:
                self.errors.append(f"{side} encoder unavailable")
                return
            try:
                prepared = resize_frame(frame, self.width, self.height)
                proc.stdin.write(np.ascontiguousarray(prepared).tobytes())
                frame_index = self.frame_counts[side]
                dropped = max(0, int(read_count) - max(0, self.last_counts[side]) - 1) if self.last_counts[side] >= 0 else 0
                self.writers[side].writerow({
                    "frame_index": frame_index,
                    "capture_utc_ns": int(utc_ns),
                    "capture_monotonic_ns": int(monotonic_ns),
                    "encoded_pts_ns": int(round(frame_index * 1e9 / max(self.fps, 1.0))),
                    "drop_count": dropped,
                })
                self.last_counts[side] = int(read_count)
                self.frame_counts[side] += 1
                if self.frame_counts[side] % max(1, int(round(self.fps))) == 0:
                    self.files[side][0].flush()
            except (BrokenPipeError, OSError) as exc:
                self.errors.append(f"{side} encoder write failed: {exc}")

    def _close_encoders(self) -> None:
        for proc in self.processes.values():
            try:
                if proc.stdin:
                    proc.stdin.close()
            except Exception:
                pass
        for side, proc in self.processes.items():
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                self.errors.append(f"{side} encoder forced closed")
        for fh, log in self.files.values():
            try: fh.flush(); fh.close()
            except Exception: pass
            try: log.close()
            except Exception: pass

    def stop(self, session_id: str) -> dict:
        with self.lock:
            if not self.session_id:
                return {"ok": True, "message": "no active camera recording"}
            if session_id and session_id != self.session_id:
                return {"ok": False, "message": "camera session id mismatch"}
            active_id, directory = self.session_id, self.directory
            self._close_encoders()
            for side in ("left", "right"):
                partial, final = directory / f"{side}.partial.mkv", directory / f"{side}.mp4"
                if not partial.exists():
                    self.errors.append(f"missing {partial.name}")
                    continue
                completed = subprocess.run(
                    [self.ffmpeg, "-y", "-hide_banner", "-loglevel", "error", "-i", str(partial), "-c", "copy", str(final)],
                    capture_output=True, text=True, timeout=30,
                )
                if completed.returncode == 0:
                    partial.unlink(missing_ok=True)
                else:
                    self.errors.append(f"{side} remux failed: {completed.stderr.strip()}")
            result = {
                "ok": not self.errors, "session_id": active_id,
                "counts": dict(self.frame_counts), "errors": list(self.errors),
            }
            self.session_id = None
            self.directory = None
            self.processes, self.files, self.writers = {}, {}, {}
            return result

    def status(self) -> dict:
        with self.lock:
            return {
                "recording": bool(self.session_id), "session_id": self.session_id,
                "counts": dict(self.frame_counts), "errors": list(self.errors),
            }


def start_camera_http_server(host: str, port: int, left: CameraState, right: CameraState, recorder: DualCameraRecorder):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, code, payload):
            raw = json.dumps(payload).encode("utf-8")
            self.send_response(code); self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Content-Length", str(len(raw)))
            self.end_headers(); self.wfile.write(raw)

        def do_GET(self):
            if self.path == "/health":
                now = time.monotonic()
                def health(state):
                    _frame, stamp, _utc, _mono, count, error = get_latest(state)
                    age = now - stamp if stamp else None
                    return {"fresh": bool(count and age is not None and age < 0.5 and not error), "age_sec": age, "frames": count, "error": error}
                self._json(200, {"ok": True, "left": health(left), "right": health(right), **recorder.status()})
                return
            if self.path not in ("/left.mjpg", "/right.mjpg"):
                self._json(404, {"ok": False, "message": "not found"}); return
            state = left if self.path.startswith("/left") else right
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers()
            last_count = -1
            try:
                while True:
                    frame, _stamp, _utc, _mono, count, _error = get_latest(state)
                    if frame is None or count == last_count:
                        time.sleep(0.01); continue
                    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if not ok: continue
                    raw = encoded.tobytes(); last_count = count
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(raw)).encode() + b"\r\n\r\n" + raw + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0")); body = json.loads(self.rfile.read(length) or b"{}")
                if self.path == "/record/start":
                    result = recorder.start(body.get("date", ""), body.get("session_id", ""), body.get("started_at_ns", 0))
                elif self.path == "/record/stop":
                    result = recorder.stop(body.get("session_id", ""))
                else:
                    self._json(404, {"ok": False, "message": "not found"}); return
                self._json(200, result)
            except Exception as exc:  # noqa: BLE001
                self._json(500, {"ok": False, "message": str(exc)})

        def log_message(self, _format, *_args):
            return

    server = ThreadingHTTPServer((host, int(port)), Handler)
    thread = threading.Thread(target=server.serve_forever, name="camera-http", daemon=True)
    thread.start()
    return server


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
        vertical_flip=args.left_vertical_flip,
        lock=threading.Lock(),
    )
    right = CameraState(
        label=args.right_name,
        index=args.right_index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        vertical_flip=args.right_vertical_flip,
        lock=threading.Lock(),
    )

    readers = [CameraReader(left, stop_event), CameraReader(right, stop_event)]
    for reader in readers:
        reader.start()

    recorder = DualCameraRecorder(
        Path(args.recording_root), args.ffmpeg, args.width, args.height, args.fps, args.crf
    )
    camera_http = start_camera_http_server(
        args.service_host, args.service_port, left, right, recorder
    )
    print(f"[camera-service] http://{args.service_host}:{args.service_port}", flush=True)

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
            "leftVerticalFlip": bool(args.left_vertical_flip),
            "rightVerticalFlip": bool(args.right_vertical_flip),
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

            left_frame, left_ts, left_utc_ns, left_mono_ns, left_count, left_error = get_latest(left)
            right_frame, right_ts, right_utc_ns, right_mono_ns, right_count, right_error = get_latest(right)
            if left_error or right_error:
                print(f"[stream] camera error: left={left_error!r} right={right_error!r}", file=sys.stderr, flush=True)
                return 2
            if left_frame is None or right_frame is None:
                time.sleep(0.005)
                continue

            recorder.write("left", left_frame, left_utc_ns, left_mono_ns, left_count)
            recorder.write("right", right_frame, right_utc_ns, right_mono_ns, right_count)
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
                        "leftVerticalFlip": bool(args.left_vertical_flip),
                        "rightVerticalFlip": bool(args.right_vertical_flip),
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
            recorder.stop(recorder.session_id or "")
        except Exception:
            pass
        try:
            camera_http.shutdown()
        except Exception:
            pass
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
