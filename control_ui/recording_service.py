"""Server-side multimodal recording and timestamp alignment.

The control UI backend owns one :class:`RecordingManager`.  Producers may call
``observe_*`` continuously; samples are only written while a session is active.
Video bytes are owned by the camera service, while this module validates and
indexes its per-frame timestamp sidecars during finalization.
"""

from __future__ import annotations

import csv
import json
import math
import re
import shutil
import statistics
import threading
import time
import urllib.error
import urllib.request
from bisect import bisect_left
from collections import deque
from pathlib import Path


ARM_VECTOR_FIELDS = (
    "actual_q_rad",
    "target_q_rad",
    "actual_dq_rad_s",
    "target_dq_rad_s",
    "actual_current_ma",
    "commanded_torque_nm",
)
HAND_VECTOR_FIELDS = (
    "actual_position_units",
    "target_position_units",
    "command_position_units",
    "raw_current_ma",
    "filtered_current_ma",
    "compliance_offset_units",
)


def _vector(value, size, fill=None):
    values = list(value) if isinstance(value, (list, tuple)) else []
    values = values[:size] + [fill] * max(0, size - len(values))
    return values


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _json_write(path, payload):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)


def _http_json(url, method="GET", payload=None, timeout=4.0):
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(url, data=body, method=method)
    request.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


class ClockMapper:
    """Robustly map a remote UTC clock onto backend receive UTC.

    The low-delay envelope rejects most network queueing.  A centred linear
    regression on that envelope estimates both offset and modest clock drift.
    """

    def __init__(self, capacity=4096):
        self._pairs = deque(maxlen=int(capacity))

    def add(self, source_ns, receive_ns):
        source_ns, receive_ns = int(source_ns), int(receive_ns)
        if source_ns > 0 and receive_ns > 0:
            self._pairs.append((source_ns, receive_ns))

    def model(self):
        pairs = list(self._pairs)
        if not pairs:
            return {"slope": 1.0, "offset_ns": 0.0, "samples": 0, "residual_p95_ns": None}
        delays = sorted(receive - source for source, receive in pairs)
        cutoff = delays[min(len(delays) - 1, max(0, int(len(delays) * 0.25)))] + 2_000_000
        selected = [(source, receive) for source, receive in pairs if receive - source <= cutoff]
        if len(selected) < 2:
            selected = pairs
        x0 = float(selected[0][0])
        xs = [(float(source) - x0) / 1e9 for source, _ in selected]
        ys = [(float(receive) - float(source)) for source, receive in selected]
        mean_x, mean_y = statistics.fmean(xs), statistics.fmean(ys)
        denom = sum((x - mean_x) ** 2 for x in xs)
        drift_ns_per_sec = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom if denom else 0.0
        slope = 1.0 + drift_ns_per_sec / 1e9
        offset = mean_y - drift_ns_per_sec * mean_x
        def mapped(source):
            return float(source) + offset + drift_ns_per_sec * ((float(source) - x0) / 1e9)
        residuals = sorted(abs(receive - mapped(source)) for source, receive in selected)
        p95 = residuals[min(len(residuals) - 1, int(len(residuals) * 0.95))] if residuals else None
        return {
            "slope": slope,
            "offset_ns": offset,
            "source_origin_ns": int(x0),
            "drift_ns_per_sec": drift_ns_per_sec,
            "samples": len(pairs),
            "envelope_samples": len(selected),
            "residual_p95_ns": p95,
        }

    def map(self, source_ns):
        model = self.model()
        origin = int(model.get("source_origin_ns") or 0)
        drift = float(model.get("drift_ns_per_sec") or 0.0)
        return int(int(source_ns) + model["offset_ns"] + drift * ((int(source_ns) - origin) / 1e9))


class RecordingManager:
    ARM_COLUMNS = [
        "seq", "source_time_ns", "receive_utc_ns", "receive_monotonic_ns", "aligned_utc_ns",
        "current_valid_mask", "velocity_valid_mask", "control_mode",
    ] + [f"{field}_{joint}" for field in ARM_VECTOR_FIELDS for joint in range(1, 6)]
    HAND_COLUMNS = [
        "seq", "source_time_ns", "receive_utc_ns", "receive_monotonic_ns", "aligned_utc_ns",
        "feedback_valid", "position_valid_mask", "current_valid_mask", "position_zero_wrap_corrected_mask", "control_mode",
    ] + [f"{field}_{joint}" for field in HAND_VECTOR_FIELDS for joint in range(1, 7)]
    # One row per PerceptionAssistMonitor decision.  This is the phase-1
    # EpisodeRecord stream; vector/list fields remain JSON so channel identity is
    # not lost when configurations change.
    EPISODE_RECORD_COLUMNS = [
        "timestamp_ns", "timestamp_source", "receive_utc_ns", "receive_monotonic_ns",
        "decision_seq", "frame_id", "state", "closure_score", "current_residual_json",
        "loaded_fingers_json", "valid", "invalid_reason", "reason", "events_json",
        "prompt_event", "monitor_only", "grasp_success_confirmed",
        "ablation_current_only", "ablation_closure_only", "ablation_joint",
    ]

    def __init__(self, root, camera_base_url="http://127.0.0.1:8092", min_free_bytes=2 * 1024**3,
                 max_duration_sec=600, alignment_hz=30):
        self.root = Path(root)
        self.camera_base_url = camera_base_url.rstrip("/")
        self.min_free_bytes = int(min_free_bytes)
        self.max_duration_ns = int(float(max_duration_sec) * 1e9)
        self.alignment_hz = max(1.0, float(alignment_hz))
        alignment_label = f"{self.alignment_hz:g}".replace(".", "p")
        self.alignment_filename = f"aligned_{alignment_label}hz.csv"
        self.lock = threading.RLock()
        self.state = "idle"
        self.error = ""
        self.session = None
        self._arm_fh = self._hand_fh = self._episode_fh = None
        self._arm_writer = self._hand_writer = self._episode_writer = None
        self._arm_rows, self._hand_rows, self._episode_rows = [], [], []
        self._fusion_fh = self._controller_input_fh = self._glove_input_fh = None
        self._fusion_rows = []
        self._hand_channel_names = []
        self._last_master_controller_seq = self._last_master_glove_seq = None
        self._camera_counts = {"left": 0, "right": 0}
        self._last_arm_receive_ns = 0
        self._last_hand_receive_ns = 0
        self._clock = ClockMapper()
        self._recover_interrupted()

    def _recover_interrupted(self):
        if not self.root.exists():
            return
        for path in self.root.glob("*/*/manifest.inprogress.json"):
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                payload.update({"state": "interrupted", "complete": False, "recovered_at_ns": time.time_ns()})
                _json_write(path.with_name("manifest.json"), payload)
                path.unlink(missing_ok=True)
            except Exception:
                continue

    def observe_arm(self, payload, receive_utc_ns=None, receive_monotonic_ns=None):
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        receive_monotonic_ns = int(receive_monotonic_ns or time.monotonic_ns())
        source_ns = int(payload.get("source_time_ns") or 0)
        self._last_arm_receive_ns = receive_utc_ns
        self._clock.add(source_ns, receive_utc_ns)
        with self.lock:
            if self.state != "recording":
                return
            row = {
                "seq": int(payload.get("seq") or 0),
                "source_time_ns": source_ns,
                "receive_utc_ns": receive_utc_ns,
                "receive_monotonic_ns": receive_monotonic_ns,
                "aligned_utc_ns": self._clock.map(source_ns) if source_ns else receive_utc_ns,
                "current_valid_mask": int(payload.get("current_valid_mask") or 0),
                "velocity_valid_mask": int(payload.get("velocity_valid_mask") or 0),
                "control_mode": str(payload.get("control_mode") or "unknown"),
            }
            for field in ARM_VECTOR_FIELDS:
                for index, value in enumerate(_vector(payload.get(field), 5), 1):
                    row[f"{field}_{index}"] = value if _finite(value) else ""
            self._arm_writer.writerow(row)
            self._arm_rows.append(row)

    def observe_hand(self, payload, receive_utc_ns=None, receive_monotonic_ns=None):
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        receive_monotonic_ns = int(receive_monotonic_ns or time.monotonic_ns())
        source_ns = int(payload.get("source_time_ns") or receive_utc_ns)
        self._last_hand_receive_ns = receive_utc_ns
        with self.lock:
            if self.state != "recording":
                return
            channel_names = payload.get("channel_names")
            if isinstance(channel_names, list) and len(channel_names) == 6:
                self._hand_channel_names = [str(name) for name in channel_names]
            row = {
                "seq": int(payload.get("seq") or 0),
                "source_time_ns": source_ns,
                "receive_utc_ns": receive_utc_ns,
                "receive_monotonic_ns": receive_monotonic_ns,
                "aligned_utc_ns": source_ns,
                "feedback_valid": 1 if payload.get("feedback_valid") else 0,
                "position_valid_mask": sum(
                    (1 << index) for index, valid in enumerate(_vector(payload.get("position_valid"), 6, False)) if valid
                ),
                "current_valid_mask": sum(
                    (1 << index) for index, valid in enumerate(_vector(payload.get("current_valid"), 6, False)) if valid
                ),
                "position_zero_wrap_corrected_mask": sum(
                    (1 << index) for index, corrected in enumerate(
                        _vector(payload.get("position_zero_wrap_corrected"), 6, False)
                    ) if corrected
                ),
                "control_mode": str(payload.get("control_mode") or "unknown"),
            }
            for field in HAND_VECTOR_FIELDS:
                for index, value in enumerate(_vector(payload.get(field), 6), 1):
                    row[f"{field}_{index}"] = value if _finite(value) else ""
            self._hand_writer.writerow(row)
            self._hand_rows.append(row)

    def observe_episode(self, decision, receive_utc_ns=None, receive_monotonic_ns=None):
        """Append the unified monitor decision without interpreting it as success."""
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        receive_monotonic_ns = int(receive_monotonic_ns or time.monotonic_ns())
        ablation = decision.get("ablation") or {}
        row = {
            "timestamp_ns": int(decision.get("timestamp_ns") or receive_utc_ns),
            "timestamp_source": str(decision.get("timestamp_source") or "bridge_receive_utc_ns"),
            "receive_utc_ns": receive_utc_ns,
            "receive_monotonic_ns": receive_monotonic_ns,
            "decision_seq": int(decision.get("decision_seq") or 0),
            "frame_id": int(decision.get("frame_id") or 0),
            "state": str(decision.get("state") or "FREE"),
            "closure_score": decision.get("closure_score") if _finite(decision.get("closure_score")) else "",
            "current_residual_json": json.dumps(decision.get("current_residual") or [], separators=(",", ":")),
            "loaded_fingers_json": json.dumps(decision.get("loaded_fingers") or [], separators=(",", ":")),
            "valid": 1 if decision.get("valid") else 0,
            "invalid_reason": str(decision.get("invalid_reason") or ""),
            "reason": str(decision.get("reason") or ""),
            "events_json": json.dumps(decision.get("events") or [], separators=(",", ":")),
            "prompt_event": str(decision.get("prompt_event") or ""),
            "monitor_only": 1,
            "grasp_success_confirmed": 0,
            "ablation_current_only": 1 if ablation.get("current_only") else 0,
            "ablation_closure_only": 1 if ablation.get("closure_only") else 0,
            "ablation_joint": 1 if ablation.get("joint") else 0,
        }
        with self.lock:
            if self.state != "recording":
                return
            self._episode_writer.writerow(row)
            self._episode_rows.append(row)

    def observe_master_fusion(self, payload, receive_utc_ns=None, receive_monotonic_ns=None):
        """Record the fused state and de-duplicate its embedded raw inputs."""
        receive_utc_ns = int(receive_utc_ns or time.time_ns())
        receive_monotonic_ns = int(receive_monotonic_ns or time.monotonic_ns())
        if payload.get("type") != "master_fusion_state":
            return
        with self.lock:
            if self.state != "recording":
                return
            record = dict(payload)
            record["record_receive_utc_ns"] = receive_utc_ns
            record["record_receive_monotonic_ns"] = receive_monotonic_ns
            self._fusion_fh.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            self._fusion_rows.append(record)

            controller = dict(payload.get("controller") or {})
            controller_seq = int(controller.get("seq", -1))
            if controller_seq >= 0 and controller_seq != self._last_master_controller_seq:
                controller["record_receive_utc_ns"] = receive_utc_ns
                self._controller_input_fh.write(
                    json.dumps(controller, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
                self._last_master_controller_seq = controller_seq

            glove = dict(payload.get("glove") or {})
            glove_seq = int(glove.get("frame_id", -1))
            if glove_seq >= 0 and glove_seq != self._last_master_glove_seq:
                glove["record_receive_utc_ns"] = receive_utc_ns
                self._glove_input_fh.write(
                    json.dumps(glove, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
                self._last_master_glove_seq = glove_seq

    def hand_telemetry_fresh(self, timeout_ns=500_000_000):
        now = time.time_ns()
        return bool(self._last_hand_receive_ns and now - self._last_hand_receive_ns <= int(timeout_ns))

    def readiness(self, arm_timeout_ns=500_000_000, hand_timeout_ns=500_000_000):
        now = time.time_ns()
        result = {
            "arm": bool(self._last_arm_receive_ns and now - self._last_arm_receive_ns <= arm_timeout_ns),
            "hand": bool(self._last_hand_receive_ns and now - self._last_hand_receive_ns <= hand_timeout_ns),
            "left_camera": False,
            "right_camera": False,
            "disk": False,
        }
        try:
            camera = _http_json(self.camera_base_url + "/health", timeout=1.5)
            result["camera_health"] = camera
            result["left_camera"] = bool(camera.get("left", {}).get("fresh"))
            result["right_camera"] = bool(camera.get("right", {}).get("fresh"))
            camera_counts = camera.get("counts") or {}
            self._camera_counts = {
                "left": int(camera_counts.get("left") or 0),
                "right": int(camera_counts.get("right") or 0),
            }
        except Exception:
            result["camera_error"] = "camera service unavailable"
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            result["disk"] = shutil.disk_usage(self.root).free >= self.min_free_bytes
        except OSError as exc:
            result["disk_error"] = str(exc)
        result["ok"] = all(result.get(key) for key in ("arm", "hand", "left_camera", "right_camera", "disk"))
        if result["ok"]:
            with self.lock:
                if self.state == "idle" and self.error == "required devices are not ready":
                    self.error = ""
        return result

    def _new_identity(self, method):
        now = time.time()
        local = time.localtime(now)
        day = time.strftime("%Y%m%d", local)
        base = time.strftime("%Y%m%d_%H%M%S", local) + f"_{int((now % 1) * 1000):03d}"
        day_dir = self.root / day
        day_dir.mkdir(parents=True, exist_ok=True)
        pattern = re.compile(rf"^{day}_\d{{6}}_\d{{3}}_(\d+)$")
        used_trials = []
        for existing in day_dir.iterdir():
            match = pattern.match(existing.name) if existing.is_dir() else None
            if match:
                used_trials.append(int(match.group(1)))
        counter = max(used_trials, default=0) + 1
        session_id = f"{base}_{counter:03d}"
        return day, counter, session_id, day_dir / session_id

    def start(self, method="unknown+unknown", require_ready=True):
        with self.lock:
            if self.state in {"arming", "recording", "stopping"}:
                return {"ok": False, "message": "a recording session is already active", **self.status()}
            self.state, self.error = "arming", ""
        ready = self.readiness()
        if require_ready and not ready["ok"]:
            with self.lock:
                # A rejected preflight is not a failed recording session.  Keep
                # the service idle so it recovers automatically when sources arrive.
                self.state, self.error = "idle", "required devices are not ready"
            return {"ok": False, "message": self.error, "readiness": ready, **self.status()}

        day, trial, session_id, directory = self._new_identity(method)
        directory.mkdir(parents=True)
        started_ns = time.time_ns()
        manifest = {
            "schema_version": 2,
            "session_id": session_id,
            "date": day,
            "task": "general_recording",
            "method": method,
            "trial": trial,
            "success": "unknown",
            "state": "arming",
            "complete": False,
            "started_at_ns": started_ns,
            "alignment_hz": self.alignment_hz,
            "alignment_file": self.alignment_filename,
            "max_duration_sec": self.max_duration_ns / 1e9,
            "readiness": ready,
        }
        _json_write(directory / "manifest.inprogress.json", manifest)
        arm_fh = hand_fh = episode_fh = fusion_fh = controller_input_fh = glove_input_fh = None
        try:
            camera = _http_json(
                self.camera_base_url + "/record/start", "POST",
                {"session_id": session_id, "date": day, "started_at_ns": started_ns}, timeout=5.0,
            )
            if not camera.get("ok"):
                raise RuntimeError(camera.get("message") or "camera recorder rejected start")
            arm_fh = (directory / "arm.csv").open("w", newline="", encoding="utf-8", buffering=1)
            hand_fh = (directory / "hand.csv").open("w", newline="", encoding="utf-8", buffering=1)
            episode_fh = (directory / "episode_records.csv").open("w", newline="", encoding="utf-8", buffering=1)
            fusion_fh = (directory / "master_fusion.jsonl").open("w", encoding="utf-8", buffering=1)
            controller_input_fh = (directory / "master_controller_input.jsonl").open(
                "w", encoding="utf-8", buffering=1
            )
            glove_input_fh = (directory / "master_glove_input.jsonl").open(
                "w", encoding="utf-8", buffering=1
            )
            arm_writer, hand_writer = csv.DictWriter(arm_fh, self.ARM_COLUMNS), csv.DictWriter(hand_fh, self.HAND_COLUMNS)
            episode_writer = csv.DictWriter(episode_fh, self.EPISODE_RECORD_COLUMNS)
            arm_writer.writeheader(); hand_writer.writeheader(); episode_writer.writeheader()
        except Exception as exc:
            for fh in (arm_fh, hand_fh, episode_fh, fusion_fh, controller_input_fh, glove_input_fh):
                if fh:
                    fh.close()
            try:
                _http_json(self.camera_base_url + "/record/stop", "POST", {"session_id": session_id}, timeout=2.0)
            except Exception:
                pass
            manifest.update({"state": "error", "error": str(exc), "stopped_at_ns": time.time_ns()})
            _json_write(directory / "manifest.json", manifest)
            (directory / "manifest.inprogress.json").unlink(missing_ok=True)
            with self.lock:
                self.state, self.error = "error", str(exc)
            return {"ok": False, "message": str(exc), **self.status()}

        with self.lock:
            self.session = {"id": session_id, "dir": directory, "manifest": manifest, "started_at_ns": started_ns}
            self._arm_fh, self._hand_fh, self._episode_fh = arm_fh, hand_fh, episode_fh
            self._arm_writer, self._hand_writer, self._episode_writer = arm_writer, hand_writer, episode_writer
            self._arm_rows, self._hand_rows, self._episode_rows = [], [], []
            self._fusion_fh = fusion_fh
            self._controller_input_fh = controller_input_fh
            self._glove_input_fh = glove_input_fh
            self._fusion_rows = []
            self._hand_channel_names = []
            self._last_master_controller_seq = self._last_master_glove_seq = None
            self._camera_counts = {"left": 0, "right": 0}
            self._clock = ClockMapper()
            self.state = "recording"
            manifest["state"] = "recording"
            _json_write(directory / "manifest.inprogress.json", manifest)
        return {"ok": True, "message": "recording started", **self.status()}

    def stop(self, incomplete_reason=None):
        with self.lock:
            if self.state == "stopping" and self.session:
                return {"ok": True, "message": "recording is stopping", **self.status()}
            if self.state != "recording" or not self.session:
                return {"ok": True, "message": "no active recording", **self.status()}
            self.state = "stopping"
            session = self.session
        stopped_ns, errors, camera = time.time_ns(), [], {}
        try:
            camera = _http_json(
                self.camera_base_url + "/record/stop", "POST", {"session_id": session["id"]}, timeout=90.0
            )
            if not camera.get("ok"):
                detail = "; ".join(camera.get("errors") or [])
                errors.append(detail or camera.get("message") or "camera stop failed")
        except Exception as exc:
            errors.append(f"camera stop failed: {exc}")
        with self.lock:
            for fh in (
                self._arm_fh, self._hand_fh, self._episode_fh,
                self._fusion_fh, self._controller_input_fh, self._glove_input_fh,
            ):
                if fh:
                    fh.flush(); fh.close()
            self._arm_fh = self._hand_fh = self._episode_fh = None
            self._arm_writer = self._hand_writer = self._episode_writer = None
            self._fusion_fh = self._controller_input_fh = self._glove_input_fh = None
        try:
            alignment = self._write_alignment(session, stopped_ns)
        except Exception as exc:
            alignment = {"ok": False, "error": str(exc)}
            errors.append(f"alignment failed: {exc}")
        if alignment.get("ok"):
            camera_counts = camera.get("counts") or {}
            for side in ("left", "right"):
                expected = camera_counts.get(side)
                actual = alignment.get("counts", {}).get(f"{side}_frames")
                if expected is not None and actual != int(expected):
                    errors.append(f"{side} frame timestamp count mismatch: encoder={expected}, sidecar={actual}")
                video = session["dir"] / f"{side}.mp4"
                if not video.exists() or video.stat().st_size <= 0:
                    errors.append(f"missing or empty {side}.mp4")
        manifest = dict(session["manifest"])
        duration_sec = (stopped_ns - session["started_at_ns"]) / 1e9
        fusion_summary = self._master_fusion_summary()
        manifest.update({
            "state": "incomplete" if incomplete_reason or errors else "complete",
            "complete": not bool(incomplete_reason or errors),
            "stopped_at_ns": stopped_ns,
            "duration_sec": duration_sec,
            "incomplete_reason": incomplete_reason or "",
            "errors": errors,
            "counts": {
                "arm": len(self._arm_rows),
                "hand": len(self._hand_rows),
                "episode_records": len(self._episode_rows),
                **fusion_summary["counts"],
                **alignment.get("counts", {}),
            },
            "rates_hz": {
                "arm": len(self._arm_rows) / duration_sec if duration_sec > 0 else 0.0,
                "hand": len(self._hand_rows) / duration_sec if duration_sec > 0 else 0.0,
                "master_fusion": len(self._fusion_rows) / duration_sec if duration_sec > 0 else 0.0,
                "master_controller_input": (
                    fusion_summary["counts"]["master_controller_input"] / duration_sec
                    if duration_sec > 0 else 0.0
                ),
                "master_glove_input": (
                    fusion_summary["counts"]["master_glove_input"] / duration_sec
                    if duration_sec > 0 else 0.0
                ),
            },
            "clock_model": self._clock.model(),
            "hand_channel_names": list(self._hand_channel_names),
            "camera": camera,
            "alignment": alignment,
            "master_fusion": fusion_summary,
        })
        _json_write(session["dir"] / "manifest.json", manifest)
        (session["dir"] / "manifest.inprogress.json").unlink(missing_ok=True)
        with self.lock:
            self.session = None
            self.state = "idle" if manifest["complete"] else "error"
            self.error = "; ".join(errors) or (incomplete_reason or "")
            self._arm_rows, self._hand_rows, self._episode_rows = [], [], []
            self._fusion_rows = []
            self._hand_channel_names = []
        return {"ok": manifest["complete"], "message": manifest["state"], "manifest": manifest, **self.status()}

    def _master_fusion_summary(self):
        rows = list(self._fusion_rows)
        controller_seqs, glove_seqs = [], []
        controller_ages, glove_ages = [], []
        calibration_ids = {"time_and_extrinsic": set(), "glove_mapping": set()}
        degradation_counts = {}
        mode_counts = {}
        last_mode = None
        degradation_events = 0
        for row in rows:
            controller = row.get("controller") or {}
            glove = row.get("glove") or {}
            calibrations = row.get("calibrations") or {}
            controller_seq, glove_seq = int(controller.get("seq", -1)), int(glove.get("frame_id", -1))
            if controller_seq >= 0 and (not controller_seqs or controller_seq != controller_seqs[-1]):
                controller_seqs.append(controller_seq)
            if glove_seq >= 0 and (not glove_seqs or glove_seq != glove_seqs[-1]):
                glove_seqs.append(glove_seq)
            if _finite(controller.get("age_sec")):
                controller_ages.append(float(controller["age_sec"]) * 1000.0)
            if _finite(glove.get("age_sec")):
                glove_ages.append(float(glove["age_sec"]) * 1000.0)
            for key in calibration_ids:
                value = str(calibrations.get(key) or "")
                if value:
                    calibration_ids[key].add(value)
            mode = str(row.get("mode") or "invalid")
            mode_counts[mode] = mode_counts.get(mode, 0) + 1
            if mode != last_mode and last_mode is not None and mode != "fused":
                degradation_events += 1
            last_mode = mode
            for reason in row.get("degradation_reasons") or []:
                degradation_counts[str(reason)] = degradation_counts.get(str(reason), 0) + 1

        def p95(values):
            ordered = sorted(values)
            return ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))] if ordered else None

        return {
            "files": {
                "fusion": "master_fusion.jsonl",
                "controller_input": "master_controller_input.jsonl",
                "glove_input": "master_glove_input.jsonl",
            },
            "counts": {
                "master_fusion": len(rows),
                "master_controller_input": len(controller_seqs),
                "master_glove_input": len(glove_seqs),
            },
            "drops": {
                "controller_sequence": sum(
                    max(0, current - previous - 1)
                    for previous, current in zip(controller_seqs, controller_seqs[1:])
                ),
                "glove_sequence": sum(
                    max(0, current - previous - 1)
                    for previous, current in zip(glove_seqs, glove_seqs[1:])
                ),
            },
            "data_age_p95_ms": {
                "controller": p95(controller_ages),
                "glove": p95(glove_ages),
            },
            "degradation_events": degradation_events,
            "degradation_reason_samples": degradation_counts,
            "mode_samples": mode_counts,
            "calibration_ids": {
                key: sorted(values) for key, values in calibration_ids.items()
            },
        }

    @staticmethod
    def _load_frames(path):
        if not Path(path).exists():
            return []
        with Path(path).open(newline="", encoding="utf-8") as fh:
            return [{**row, "aligned_utc_ns": int(row.get("capture_utc_ns") or 0)} for row in csv.DictReader(fh)]

    @staticmethod
    def _nearest(rows, times, tick, max_delta_ns=50_000_000):
        if not rows:
            return None, None, "no_frames"
        index = bisect_left(times, tick)
        choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(rows)]
        best = min(choices, key=lambda candidate: abs(times[candidate] - tick))
        delta = times[best] - tick
        if abs(delta) > max_delta_ns:
            return None, delta, "frame_gap"
        return rows[best], delta, ""

    @staticmethod
    def _interpolate(rows, times, tick, columns, max_gap_ns=100_000_000, validator=None):
        index = bisect_left(times, tick)
        if index <= 0 or index >= len(rows):
            return None, "outside_range"
        before, after = rows[index - 1], rows[index]
        ta, tb = times[index - 1], times[index]
        if tick - ta > max_gap_ns or tb - tick > max_gap_ns or tb <= ta:
            return None, "sample_gap"
        ratio = (tick - ta) / float(tb - ta)
        out = {}
        for column in columns:
            if (validator and not validator(column, before, after)) or not _finite(before.get(column)) or not _finite(after.get(column)):
                out[column] = ""
            else:
                out[column] = float(before[column]) + ratio * (float(after[column]) - float(before[column]))
        out["control_mode"] = before.get("control_mode", "unknown")
        return out, ""

    @staticmethod
    def _sequence_drops(rows):
        seqs = [int(row.get("seq") or 0) for row in rows]
        return sum(max(0, current - previous - 1) for previous, current in zip(seqs, seqs[1:]))

    @staticmethod
    def _error_stats(values):
        if not values:
            return {"mean_ms": None, "max_ms": None, "p95_ms": None}
        ordered = sorted(values)
        return {
            "mean_ms": statistics.fmean(ordered),
            "max_ms": ordered[-1],
            "p95_ms": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
        }

    def _write_alignment(self, session, stopped_ns=None):
        directory, start_ns = session["dir"], session["started_at_ns"]
        model = self._clock.model()
        arm_rows = list(self._arm_rows)
        for row in arm_rows:
            source = int(row.get("source_time_ns") or 0)
            if source:
                origin = int(model.get("source_origin_ns") or 0)
                drift = float(model.get("drift_ns_per_sec") or 0.0)
                row["aligned_utc_ns"] = int(source + model["offset_ns"] + drift * ((source - origin) / 1e9))
            else:
                row["aligned_utc_ns"] = int(row["receive_utc_ns"])
        hand_rows = list(self._hand_rows)
        left, right = self._load_frames(directory / "left_frames.csv"), self._load_frames(directory / "right_frames.csv")
        arm_rows.sort(key=lambda row: int(row["aligned_utc_ns"])); hand_rows.sort(key=lambda row: int(row["aligned_utc_ns"]))
        left.sort(key=lambda row: int(row["aligned_utc_ns"])); right.sort(key=lambda row: int(row["aligned_utc_ns"]))
        arm_times = [int(row["aligned_utc_ns"]) for row in arm_rows]
        hand_times = [int(row["aligned_utc_ns"]) for row in hand_rows]
        left_times = [int(row["aligned_utc_ns"]) for row in left]
        right_times = [int(row["aligned_utc_ns"]) for row in right]
        end_candidates = [times[-1] for times in (arm_times, hand_times, left_times, right_times) if times]
        if not end_candidates:
            raise ValueError("no samples available for alignment")
        end_ns, period_ns = min(end_candidates), int(1e9 / self.alignment_hz)
        arm_columns = [column for column in self.ARM_COLUMNS if any(column.startswith(field + "_") for field in ARM_VECTOR_FIELDS)]
        hand_columns = [column for column in self.HAND_COLUMNS if any(column.startswith(field + "_") for field in HAND_VECTOR_FIELDS)]
        columns = [
            "tick_index", "session_time_ns", "utc_ns",
            "left_frame_index", "left_delta_ms", "left_valid", "left_reason",
            "right_frame_index", "right_delta_ms", "right_valid", "right_reason",
            "arm_valid", "arm_reason", "arm_control_mode", "arm_current_valid_mask", "arm_velocity_valid_mask",
            "hand_valid", "hand_reason", "hand_control_mode", "hand_feedback_valid",
            "hand_position_valid_mask", "hand_current_valid_mask", "hand_position_zero_wrap_corrected_mask",
        ]
        columns += ["arm_" + column for column in arm_columns] + ["hand_" + column for column in hand_columns]
        rows, left_errors, right_errors, arm_errors, hand_errors = [], [], [], [], []
        tick, tick_index = start_ns, 0

        def arm_field_valid(column, before, after):
            if column.startswith("actual_current_ma_"):
                bit = int(column.rsplit("_", 1)[1]) - 1
                return all(int(row.get("current_valid_mask") or 0) & (1 << bit) for row in (before, after))
            if column.startswith("actual_dq_rad_s_"):
                bit = int(column.rsplit("_", 1)[1]) - 1
                return all(int(row.get("velocity_valid_mask") or 0) & (1 << bit) for row in (before, after))
            return True

        def hand_field_valid(column, before, after):
            if column.startswith(("raw_current_ma_", "filtered_current_ma_")):
                bit = int(column.rsplit("_", 1)[1]) - 1
                return all(int(row.get("current_valid_mask") or 0) & (1 << bit) for row in (before, after))
            if column.startswith("actual_position_units_"):
                bit = int(column.rsplit("_", 1)[1]) - 1
                return all(int(row.get("position_valid_mask") or 0) & (1 << bit) for row in (before, after))
            return bool(int(before.get("feedback_valid") or 0) and int(after.get("feedback_valid") or 0))

        while tick <= end_ns:
            left_row, left_delta, left_reason = self._nearest(left, left_times, tick)
            right_row, right_delta, right_reason = self._nearest(right, right_times, tick)
            arm, arm_reason = self._interpolate(arm_rows, arm_times, tick, arm_columns, validator=arm_field_valid)
            hand, hand_reason = self._interpolate(hand_rows, hand_times, tick, hand_columns, validator=hand_field_valid)
            arm_index, hand_index = bisect_left(arm_times, tick) - 1, bisect_left(hand_times, tick) - 1
            hand_after_index = hand_index + 1
            if hand and (
                hand_index < 0 or hand_after_index >= len(hand_rows)
                or not hand_field_valid("", hand_rows[hand_index], hand_rows[hand_after_index])
            ):
                hand, hand_reason = None, "invalid_feedback"
            row = {
                "tick_index": tick_index, "session_time_ns": tick - start_ns, "utc_ns": tick,
                "left_frame_index": left_row.get("frame_index", "") if left_row else "",
                "left_delta_ms": left_delta / 1e6 if left_delta is not None else "",
                "right_frame_index": right_row.get("frame_index", "") if right_row else "",
                "right_delta_ms": right_delta / 1e6 if right_delta is not None else "",
                "left_valid": 1 if left_row else 0, "left_reason": left_reason,
                "right_valid": 1 if right_row else 0, "right_reason": right_reason,
                "arm_valid": 1 if arm else 0, "arm_reason": arm_reason,
                "hand_valid": 1 if hand else 0, "hand_reason": hand_reason,
                "arm_control_mode": arm_rows[arm_index].get("control_mode", "unknown") if arm_index >= 0 else "",
                "arm_current_valid_mask": arm_rows[arm_index].get("current_valid_mask", 0) if arm_index >= 0 else 0,
                "arm_velocity_valid_mask": arm_rows[arm_index].get("velocity_valid_mask", 0) if arm_index >= 0 else 0,
                "hand_control_mode": hand_rows[hand_index].get("control_mode", "unknown") if hand_index >= 0 else "",
                "hand_feedback_valid": hand_rows[hand_index].get("feedback_valid", 0) if hand_index >= 0 else 0,
                "hand_position_valid_mask": hand_rows[hand_index].get("position_valid_mask", 0) if hand_index >= 0 else 0,
                "hand_current_valid_mask": hand_rows[hand_index].get("current_valid_mask", 0) if hand_index >= 0 else 0,
                "hand_position_zero_wrap_corrected_mask": hand_rows[hand_index].get(
                    "position_zero_wrap_corrected_mask", 0
                ) if hand_index >= 0 else 0,
            }
            if left_row is not None: left_errors.append(abs(left_delta) / 1e6)
            if right_row is not None: right_errors.append(abs(right_delta) / 1e6)
            if arm and 0 <= arm_index < len(arm_times) - 1:
                arm_errors.append(min(tick - arm_times[arm_index], arm_times[arm_index + 1] - tick) / 1e6)
            if hand and 0 <= hand_index < len(hand_times) - 1:
                hand_errors.append(min(tick - hand_times[hand_index], hand_times[hand_index + 1] - tick) / 1e6)
            if arm:
                row.update({"arm_" + key: value for key, value in arm.items() if key in arm_columns})
            if hand:
                row.update({"hand_" + key: value for key, value in hand.items() if key in hand_columns})
            rows.append(row); tick += period_ns; tick_index += 1
        with (directory / self.alignment_filename).open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, columns, extrasaction="ignore"); writer.writeheader(); writer.writerows(rows)
        streams = {"arm": arm_times, "hand": hand_times, "left": left_times, "right": right_times}
        stopped_ns = int(stopped_ns or end_ns)
        offsets = {
            name: {
                "start_ms": (times[0] - start_ns) / 1e6 if times else None,
                "end_ms": (stopped_ns - times[-1]) / 1e6 if times else None,
            }
            for name, times in streams.items()
        }
        left_drops = sum(int(row.get("drop_count") or 0) for row in left)
        right_drops = sum(int(row.get("drop_count") or 0) for row in right)
        return {
            "ok": True,
            "counts": {"left_frames": len(left), "right_frames": len(right), "aligned_rows": len(rows)},
            "frame_alignment": {"left": self._error_stats(left_errors), "right": self._error_stats(right_errors)},
            "sample_alignment": {"arm": self._error_stats(arm_errors), "hand": self._error_stats(hand_errors)},
            "stream_offsets": offsets,
            "drops": {
                "arm_sequence": self._sequence_drops(arm_rows),
                "hand_sequence": self._sequence_drops(hand_rows),
                "left_frames": left_drops,
                "right_frames": right_drops,
            },
        }

    def status(self):
        with self.lock:
            now = time.time_ns()
            session = self.session
            return {
                "recording_state": self.state,
                "session_id": session["id"] if session else None,
                "elapsed_sec": (now - session["started_at_ns"]) / 1e9 if session else 0.0,
                "counts": {
                    "arm": len(self._arm_rows), "hand": len(self._hand_rows),
                    "episode_records": len(self._episode_rows),
                    "master_fusion": len(self._fusion_rows),
                    "left_frames": self._camera_counts["left"],
                    "right_frames": self._camera_counts["right"],
                },
                "error": self.error,
            }

    def watchdog(self, grace_ns=1_000_000_000):
        with self.lock:
            if self.state != "recording":
                return None
            session = self.session
        now = time.time_ns()
        if session and now - session["started_at_ns"] >= self.max_duration_ns:
            return self.stop()
        missing = []
        if not self._last_arm_receive_ns or now - self._last_arm_receive_ns > grace_ns:
            missing.append("arm telemetry")
        if not self._last_hand_receive_ns or now - self._last_hand_receive_ns > grace_ns:
            missing.append("hand telemetry")
        try:
            camera = _http_json(self.camera_base_url + "/health", timeout=1.0)
            if not camera.get("left", {}).get("fresh"): missing.append("left camera")
            if not camera.get("right", {}).get("fresh"): missing.append("right camera")
        except Exception:
            missing.append("camera service")
        return self.stop("lost: " + ", ".join(missing)) if missing else None
