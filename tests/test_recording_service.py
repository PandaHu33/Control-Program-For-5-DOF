import csv
import json
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest import mock

from control_ui.recording_service import ClockMapper, RecordingManager


class ClockMapperTests(unittest.TestCase):
    def test_low_delay_envelope_rejects_queueing_and_maps_clock(self):
        mapper = ClockMapper()
        offset = 37_000_000
        for index in range(200):
            source = 1_700_000_000_000_000_000 + index * 10_000_000
            queueing = 40_000_000 if index % 10 == 0 else (index % 3) * 100_000
            mapper.add(source, source + offset + queueing)
        model = mapper.model()
        self.assertGreater(model["envelope_samples"], 100)
        self.assertLess(abs(model["offset_ns"] - offset), 2_000_000)
        self.assertLess(model["residual_p95_ns"], 2_000_000)

    def test_clock_drift_is_estimated_without_losing_nanosecond_scale(self):
        mapper = ClockMapper()
        origin = 1_700_000_000_000_000_000
        for index in range(300):
            source = origin + index * 10_000_000
            drift = index * 1_000  # 100 ppm over 10 ms samples.
            mapper.add(source, source + 25_000_000 + drift + (index % 3) * 50_000)
        mapped = mapper.map(origin + 2_000_000_000)
        self.assertLess(abs(mapped - (origin + 2_000_000_000 + 25_200_000)), 500_000)


class RecordingManagerTests(unittest.TestCase):
    def camera_api(self, url, method="GET", payload=None, timeout=0):
        if url.endswith("/health"):
            return {
                "ok": True,
                "left": {"fresh": True, "age_sec": 0.02, "error": ""},
                "right": {"fresh": True, "age_sec": 0.03, "error": ""},
            }
        return {"ok": True}

    def test_canonical_writer_is_nonblocking_and_overflow_is_explicit(self):
        entered = threading.Event()
        release = threading.Event()

        class SlowFile:
            def write(self, _text):
                entered.set()
                release.wait(2.0)

        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(
                Path(temp_dir), min_free_bytes=0, canonical_queue_capacity=64
            )
            manager._start_canonical_writer("test-session", SlowFile())
            payload = {"type": "canonical_goal", "schema_version": 4, "seq": 1}
            started = time.perf_counter()
            self.assertTrue(manager.observe_canonical_goal(payload))
            self.assertLess(time.perf_counter() - started, 0.05)
            self.assertTrue(entered.wait(0.5))
            for seq in range(2, 66):
                self.assertTrue(manager.observe_canonical_goal({**payload, "seq": seq}))
            self.assertFalse(manager.observe_canonical_goal({**payload, "seq": 66}))
            release.set()
            error = manager._stop_canonical_writer()
            self.assertIn("queue overflow", error)

    def test_canonical_sequence_summary_separates_missing_reordered_and_duplicate_rows(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0)
            manager._canonical_rows = [
                {"seq": seq, "validity": {}, "calibrations": {}}
                for seq in (1, 3, 2, 4, 4)
            ]
            summary = manager._canonical_summary()
            self.assertEqual(summary["sequence_drops"], 0)
            self.assertEqual(summary["sequence_out_of_order"], 2)
            self.assertEqual(summary["sequence_duplicates"], 1)

    def test_canonical_summary_reports_fixed_50hz_clock(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(
                Path(temp_dir), min_free_bytes=0, canonical_publish_hz=50.0
            )
            manager._canonical_rows = [
                {
                    "seq": index + 1,
                    "record_receive_monotonic_ns": 1_000_000_000 + index * 20_000_000,
                    "validity": {},
                    "calibrations": {},
                }
                for index in range(6)
            ]
            summary = manager._canonical_summary()
            self.assertEqual(summary["target_rate_hz"], 50.0)
            self.assertAlmostEqual(summary["measured_rate_hz"], 50.0)
            self.assertEqual(summary["interval_p95_ms"], 20.0)

    @mock.patch("control_ui.recording_service._http_json")
    def test_readiness_exposes_the_same_camera_health_used_for_preflight(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0, alignment_hz=20)
            now = __import__("time").time_ns()
            manager.observe_arm({"source_time_ns": now, "seq": 0})
            manager.observe_hand({"source_time_ns": now, "seq": 0})
            readiness = manager.readiness()
            self.assertTrue(readiness["left_camera"])
            self.assertTrue(readiness["right_camera"])
            self.assertEqual(readiness["camera_health"]["left"]["age_sec"], 0.02)

    @mock.patch("control_ui.recording_service._http_json")
    def test_session_streams_raw_data_and_builds_alignment(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0, alignment_hz=20)
            # Warm both required telemetry sources before strict preflight.
            now = __import__("time").time_ns()
            manager.observe_arm({"source_time_ns": now, "seq": 0})
            manager.observe_hand({"source_time_ns": now, "seq": 0})
            manager.set_browser_raw_connected(True)
            manager.observe_raw_input({
                "type": "raw_master_input", "schema_version": 1,
                "source": "pico_hand", "signal_form": "continuous",
                "value": {
                    "stream": "hand_skeleton", "frameId": 0, "rightTracked": True,
                    "rightPositions": [0.0] * 63, "rightRotations": [0.0] * 63,
                },
            })
            started = manager.start("keyboard+vr")
            self.assertTrue(started["ok"], started)
            session_dir = manager.session["dir"]
            session_id = manager.session["id"]
            start_ns = manager.session["started_at_ns"]
            manager.observe_raw_input({
                "type": "raw_master_input", "schema_version": 1,
                "source": "keyboard", "signal_form": "discrete",
                "value": {
                    "code": "KeyW", "key": "w", "event": "down", "repeat": False,
                    "modifiers": {"alt": False, "ctrl": False, "shift": False, "meta": False},
                    "pressed_keys": ["KeyW"], "client_time_ms": 1,
                },
            }, receive_utc_ns=start_ns + 1)
            manager.observe_raw_input({
                "type": "raw_master_input", "schema_version": 1,
                "source": "pico_hand", "signal_form": "continuous",
                "value": {
                    "stream": "hand_skeleton", "frameId": 1, "rightTracked": True,
                    "rightPositions": [0.1] * 63, "rightRotations": [0.2] * 63,
                },
            }, receive_utc_ns=start_ns + 1)
            for index in range(12):
                stamp = start_ns + index * 10_000_000
                manager.observe_arm({
                    "seq": index, "source_time_ns": stamp, "control_mode": "torque",
                    "current_valid_mask": 31, "velocity_valid_mask": 15,
                    "actual_q_rad": [index] * 5, "target_q_rad": [index + 1] * 5,
                    "actual_dq_rad_s": [1] * 5, "target_dq_rad_s": [2] * 5,
                    "actual_current_ma": [100] * 5, "commanded_torque_nm": [3] * 5,
                }, receive_utc_ns=stamp + 1_000_000, receive_monotonic_ns=stamp)
            for index in range(7):
                stamp = start_ns + index * 20_000_000
                manager.observe_hand({
                    "seq": index, "source_time_ns": stamp, "feedback_valid": True, "control_mode": "vr",
                    "channel_names": ["thumb_pitch", "thumb_yaw", "index", "middle", "ring", "pinky"],
                    "position_valid": [True] * 6, "current_valid": [True] * 6,
                    "position_zero_wrap_corrected": [True] + [False] * 5,
                    "actual_position_units": [1000] * 6, "target_position_units": [1100] * 6,
                    "command_position_units": [1050] * 6, "raw_current_ma": [200] * 6,
                    "filtered_current_ma": [190] * 6, "compliance_offset_units": [5] * 6,
                }, receive_utc_ns=stamp, receive_monotonic_ns=stamp)
                manager.observe_episode({
                    "timestamp_ns": stamp, "timestamp_source": "wa100_source_time_ns",
                    "decision_seq": index + 1, "frame_id": index, "state": "GRASP_CANDIDATE",
                    "closure_score": 0.7, "current_residual": [90.0] * 6,
                    "loaded_fingers": ["index", "middle"], "valid": 1,
                    "invalid_reason": "", "reason": "synthetic_joint_evidence",
                    "events": ["GRASP_CANDIDATE"] if index == 0 else [],
                    "monitor_only": True, "grasp_success_confirmed": False,
                    "ablation": {"current_only": True, "closure_only": True, "joint": True},
                }, receive_utc_ns=stamp, receive_monotonic_ns=stamp)
                manager.observe_master_fusion({
                    "type": "master_fusion_state",
                    "schema_version": 1,
                    "seq": index + 1,
                    "mode": "fused" if index < 5 else "controller_only",
                    "degradation_reasons": [] if index < 5 else ["glove_stale_or_invalid"],
                    "controller": {
                        "seq": index,
                        "age_sec": 0.01,
                        "position_m": [0.1, 0.2, 0.3],
                        "rotation_xyzw": [0, 0, 0, 1],
                    },
                    "glove": {
                        "frame_id": index,
                        "age_sec": 0.02,
                        "hand_index": 1,
                        "local_finger_rotations_deg": [0.0] * 57,
                        "mapped_target_units": [1000] * 6,
                    },
                    "calibrations": {
                        "time_and_extrinsic": "alignment-test",
                        "glove_mapping": "glove-test",
                    },
                }, receive_utc_ns=stamp, receive_monotonic_ns=stamp)
                manager.observe_canonical_goal({
                    "type": "canonical_goal", "schema_version": 1, "seq": index + 1,
                    "condition_id": "M1-PICO", "wrist_source": "pico_wrist", "hand_source": "pico_hand",
                    "validity": {"wrist_dof_mask": 15, "hand_node_mask": (1 << 21) - 1},
                    "calibrations": {
                        "wrist_mapping": "pico-test", "hand_model": "pico-direct",
                        "hand_mapping": "pico-hand-test",
                    },
                    "invalid_reasons": [],
                }, receive_utc_ns=stamp, receive_monotonic_ns=stamp)
            for side in ("left", "right"):
                (session_dir / f"{side}.mp4").write_bytes(b"synthetic-video")
                with (session_dir / f"{side}_frames.csv").open("w", newline="", encoding="utf-8") as fh:
                    writer = csv.DictWriter(fh, ["frame_index", "capture_utc_ns", "capture_monotonic_ns", "encoded_pts_ns", "drop_count"])
                    writer.writeheader()
                    for index in range(4):
                        stamp = start_ns + index * 33_333_333
                        writer.writerow({"frame_index": index, "capture_utc_ns": stamp, "capture_monotonic_ns": stamp, "encoded_pts_ns": index * 33_333_333, "drop_count": 0})
            stopped = manager.stop()
            self.assertTrue(stopped["ok"], stopped)
            manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertTrue(manifest["complete"])
            self.assertEqual(manifest["counts"]["arm"], 12)
            self.assertEqual(manifest["counts"]["hand"], 7)
            self.assertEqual(manifest["counts"]["episode_records"], 7)
            self.assertEqual(manifest["counts"]["master_fusion"], 7)
            self.assertEqual(manifest["counts"]["canonical_goal"], 7)
            self.assertEqual(manifest["counts"]["master_controller_input"], 7)
            self.assertNotIn("master_glove_input", manifest["counts"])
            self.assertFalse((session_dir / "master_glove_input.jsonl").exists())
            self.assertEqual(manifest["counts"]["raw_input"], 2)
            self.assertEqual(manifest["raw_input"]["counts"]["keyboard"], 1)
            self.assertEqual(manifest["raw_input"]["counts"]["pico_hand:hand_skeleton"], 1)
            self.assertEqual(
                manifest["hand_channel_names"],
                ["thumb_pitch", "thumb_yaw", "index", "middle", "ring", "pinky"],
            )
            self.assertEqual(manifest["alignment_hz"], 20.0)
            self.assertEqual(manifest["alignment_file"], "aligned_20hz.csv")
            self.assertTrue((session_dir / "aligned_20hz.csv").exists())
            self.assertFalse((session_dir / "manifest.inprogress.json").exists())
            self.assertEqual(manifest["task"], "general_recording")
            self.assertEqual(manifest["method"], "keyboard+vr")
            self.assertEqual(manifest["success"], "unknown")
            self.assertIn("p95_ms", manifest["alignment"]["frame_alignment"]["left"])
            with (session_dir / "aligned_20hz.csv").open(newline="", encoding="utf-8") as fh:
                aligned = list(csv.DictReader(fh))
            self.assertIn("left_valid", aligned[0])
            self.assertIn("arm_current_valid_mask", aligned[0])
            self.assertTrue(any(row["hand_position_valid_mask"] == "63" for row in aligned))
            self.assertTrue(any(row["hand_current_valid_mask"] == "63" for row in aligned))
            self.assertTrue(any(row["hand_position_zero_wrap_corrected_mask"] == "1" for row in aligned))
            with (session_dir / "episode_records.csv").open(newline="", encoding="utf-8") as fh:
                episode_records = list(csv.DictReader(fh))
            self.assertEqual(episode_records[0]["timestamp_source"], "wa100_source_time_ns")
            self.assertEqual(episode_records[0]["grasp_success_confirmed"], "0")
            self.assertIn("preset_name", episode_records[0])
            self.assertIn("preset_positions_json", episode_records[0])
            self.assertTrue((session_dir / "master_fusion.jsonl").exists())
            self.assertTrue((session_dir / "master_controller_input.jsonl").exists())
            self.assertTrue((session_dir / "raw_input.jsonl").exists())
            self.assertTrue((session_dir / "canonical_goal.jsonl").exists())
            fusion_rows = [json.loads(line) for line in (session_dir / "master_fusion.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertNotIn("glove", fusion_rows[0])
            self.assertEqual(set(fusion_rows[0]["glove_state"]), {"frame_id", "age_sec"})
            canonical_rows = [json.loads(line) for line in (session_dir / "canonical_goal.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertNotIn("glove_state", canonical_rows[0])
            raw_input = [json.loads(line) for line in (session_dir / "raw_input.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertEqual([row["source"] for row in raw_input], ["keyboard", "pico_hand"])
            self.assertLess(raw_input[0]["timestamp_ns"], raw_input[1]["timestamp_ns"])
            self.assertTrue(all(row["trial_id"] == session_id for row in raw_input))
            self.assertEqual(
                manifest["master_fusion"]["calibration_ids"]["time_and_extrinsic"],
                ["alignment-test"],
            )
            self.assertEqual(manifest["master_fusion"]["degradation_events"], 1)
            self.assertEqual(manifest["canonical_goal"]["valid_samples"], 7)

    @mock.patch("control_ui.recording_service._http_json")
    def test_canonical_logging_can_be_disabled_without_creating_a_file(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0, canonical_log_enabled=False)
            started = manager.start("keyboard+glove", require_ready=False)
            self.assertTrue(started["ok"])
            session_dir = manager.session["dir"]
            self.assertIsNone(manager._canonical_fh)
            self.assertFalse((session_dir / "canonical_goal.jsonl").exists())
            manager.stop("test cleanup")

    @mock.patch("control_ui.recording_service._http_json")
    def test_missing_required_modality_rejects_start(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0)
            result = manager.start()
            self.assertFalse(result["ok"])
            self.assertEqual(result["recording_state"], "idle")

    def test_daily_trial_increments_across_session_directories(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0)
            _day, first_trial, _first_id, first_dir = manager._new_identity("mode")
            first_dir.mkdir(parents=True)
            _day, second_trial, _second_id, _second_dir = manager._new_identity("mode")
            self.assertEqual(first_trial, 1)
            self.assertEqual(second_trial, 2)

    def test_inprogress_manifest_is_recovered_as_interrupted(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            session_dir = Path(temp_dir) / "20260721" / "20260721_120000_000_001"
            session_dir.mkdir(parents=True)
            (session_dir / "manifest.inprogress.json").write_text(
                json.dumps({"session_id": session_dir.name, "state": "recording"}), encoding="utf-8"
            )
            RecordingManager(Path(temp_dir), min_free_bytes=0)
            recovered = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(recovered["state"], "interrupted")
            self.assertFalse(recovered["complete"])
            self.assertFalse((session_dir / "manifest.inprogress.json").exists())

    @mock.patch("control_ui.recording_service.shutil.disk_usage")
    @mock.patch("control_ui.recording_service._http_json")
    def test_disk_preflight_enforces_reserved_space(self, camera_api, disk_usage):
        camera_api.side_effect = self.camera_api
        disk_usage.return_value = SimpleNamespace(free=1024)
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=2048)
            now = __import__("time").time_ns()
            manager.observe_arm({"source_time_ns": now})
            manager.observe_hand({"source_time_ns": now})
            result = manager.start()
            self.assertFalse(result["ok"])
            self.assertFalse(result["readiness"]["disk"])

    @mock.patch("control_ui.recording_service._http_json")
    def test_repeated_start_and_stop_are_idempotent(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0)
            first = manager.start(require_ready=False)
            self.assertTrue(first["ok"])
            self.assertFalse(manager.start(require_ready=False)["ok"])
            manager.stop()
            second_stop = manager.stop()
            self.assertTrue(second_stop["ok"])
            self.assertEqual(second_stop["message"], "no active recording")

    @mock.patch("control_ui.recording_service._http_json")
    def test_watchdog_keeps_latched_preset_hand_recording_alive(self, camera_api):
        camera_api.side_effect = self.camera_api
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0)
            now = time.time_ns()
            manager.state = "recording"
            manager.session = {"started_at_ns": now, "id": "preset-test"}
            manager._last_arm_receive_ns = now
            manager._last_hand_receive_ns = now
            manager._browser_raw_connected = True
            manager._raw_input_required = ["keyboard", "preset_hand:hand_skeleton"]
            manager._raw_input_last_seen_ns["preset_hand:hand_skeleton"] = now - 10_000_000_000

            self.assertIsNone(manager.watchdog(grace_ns=1_000_000_000))

    def test_alignment_helpers_do_not_extrapolate_across_gaps(self):
        rows = [{"aligned_utc_ns": 100, "value": 1}, {"aligned_utc_ns": 200, "value": 2}]
        self.assertEqual(RecordingManager._interpolate(rows, [100, 200], 100, ["value"])[1], "outside_range")
        frame, _delta, reason = RecordingManager._nearest(rows, [100, 200], 1_000_000_000)
        self.assertIsNone(frame)
        self.assertEqual(reason, "frame_gap")

    def test_alignment_rate_controls_output_filename_and_period(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manager = RecordingManager(Path(temp_dir), min_free_bytes=0, alignment_hz=20)
            self.assertEqual(manager.alignment_hz, 20.0)
            self.assertEqual(manager.alignment_filename, "aligned_20hz.csv")


if __name__ == "__main__":
    unittest.main()
