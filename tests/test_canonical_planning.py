import json
import math
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from control_ui.canonical_goal import CanonicalGoalBuilder
from control_ui.canonical_planning import CanonicalPlannerConfig, CanonicalWristPlanner
from control_ui.replay_canonical import verify_session


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "control_ui" / "wa100_kinematics_model.json"


def pose(x=0.5, y=0.0, z=0.0, roll=0.0):
    return {
        "position_m": [x, y, z],
        "orientation_xyzw": [math.sin(roll * 0.5), 0.0, 0.0, math.cos(roll * 0.5)],
    }


class CanonicalPlanningTests(unittest.TestCase):
    def test_jerk_limited_profile_respects_configured_state_limits(self):
        config = CanonicalPlannerConfig()
        planner = CanonicalWristPlanner(config)
        start = time.time_ns()
        planner.step(
            source="keyboard", target_pose=pose(), source_receive_ns=start, now_ns=start
        )
        previous_acceleration = planner.acceleration.copy()
        positions = []
        for tick in range(1, 101):
            now = start + tick * 20_000_000
            output, _, metadata = planner.step(
                source="keyboard", target_pose=pose(x=0.65, roll=0.5),
                source_receive_ns=now, now_ns=now,
            )
            positions.append(output["position_m"][0])
            self.assertTrue(metadata["valid_for_dispatch"])
            self.assertLessEqual(abs(planner.velocity[0]), config.translation_max_velocity_m_s + 1e-12)
            self.assertLessEqual(abs(planner.acceleration[0]), config.translation_max_acceleration_m_s2 + 1e-12)
            jerk = np.abs((planner.acceleration - previous_acceleration) / 0.02)
            self.assertLessEqual(jerk[0], config.translation_max_jerk_m_s3 + 1e-9)
            self.assertLessEqual(jerk[3], config.roll_max_jerk_rad_s3 + 1e-9)
            previous_acceleration = planner.acceleration.copy()
        self.assertTrue(all(current >= previous for previous, current in zip(positions, positions[1:])))
        self.assertGreater(positions[-1], 0.60)

    def test_one_euro_reduces_stationary_noise_without_nonfinite_output(self):
        planner = CanonicalWristPlanner(CanonicalPlannerConfig())
        rng = np.random.default_rng(7)
        start = time.time_ns()
        planner.step(
            source="pico_wrist", target_pose=pose(), source_receive_ns=start, now_ns=start
        )
        raw = []
        filtered = []
        for tick in range(1, 401):
            now = start + tick * 20_000_000
            noise = float(rng.normal(0.0, 0.003))
            output, _, metadata = planner.step(
                source="pico_wrist", target_pose=pose(x=0.5 + noise),
                source_receive_ns=now, now_ns=now,
            )
            raw.append(noise)
            filtered.append(output["position_m"][0] - 0.5)
            self.assertTrue(np.all(np.isfinite(output["position_m"])))
            self.assertLess(metadata["processing_time_ms"], 2.0)
        self.assertLess(np.std(filtered[50:]), np.std(raw[50:]) * 0.40)

    def test_one_euro_predictive_path_keeps_constant_velocity_lag_within_one_tick(self):
        planner = CanonicalWristPlanner(CanonicalPlannerConfig())
        start = time.time_ns()
        planner.step(
            source="pico_wrist", target_pose=pose(), source_receive_ns=start, now_ns=start
        )
        errors = []
        speed_m_s = 0.10
        for tick in range(1, 301):
            now = start + tick * 20_000_000
            target_x = 0.5 + speed_m_s * tick * 0.02
            output, _, _ = planner.step(
                source="pico_wrist", target_pose=pose(x=target_x),
                source_receive_ns=now, now_ns=now,
            )
            errors.append(target_x - output["position_m"][0])
        phase_lag_ms = float(np.median(errors[100:]) / speed_m_s * 1000.0)
        self.assertLessEqual(phase_lag_ms, 20.0)

    def test_source_switch_rebases_without_a_published_jump(self):
        planner = CanonicalWristPlanner(CanonicalPlannerConfig())
        start = time.time_ns()
        first, _, _ = planner.step(
            source="keyboard", target_pose=pose(), source_receive_ns=start, now_ns=start
        )
        now = start + 20_000_000
        switched, delta, metadata = planner.step(
            source="vr_controller", target_pose=pose(x=0.7), source_receive_ns=now, now_ns=now
        )
        np.testing.assert_allclose(switched["position_m"], first["position_m"], atol=1e-12)
        np.testing.assert_allclose(delta, np.zeros(6), atol=1e-12)
        self.assertEqual(metadata["reset_reason"], "source_or_profile_changed")

    def test_expired_source_holds_and_is_not_dispatchable(self):
        planner = CanonicalWristPlanner(CanonicalPlannerConfig())
        start = time.time_ns()
        initial, _, _ = planner.step(
            source="gamepad", target_pose=pose(), source_receive_ns=start, now_ns=start
        )
        output, delta, metadata = planner.step(
            source="gamepad", target_pose=pose(x=0.7), source_receive_ns=start,
            now_ns=start + 120_000_000,
        )
        np.testing.assert_allclose(output["position_m"], initial["position_m"], atol=1e-12)
        np.testing.assert_allclose(delta, np.zeros(6), atol=1e-12)
        self.assertTrue(metadata["hold_expired"])
        self.assertFalse(metadata["valid_for_dispatch"])

    def test_v4_session_replays_wa100_hand_deterministically(self):
        builder = CanonicalGoalBuilder(MODEL, planner_config=CanonicalPlannerConfig())
        start = time.time_ns()
        hand_value = {
            "stream": "hand_skeleton", "frameId": 7,
            "rightRotations": [0.0] * 63,
            "mapped_target_units": [1000.0] * 6,
            "glove_calibration_id": "replay-test",
        }
        builder.observe_hand_input("data_glove", hand_value, start)
        rows = []
        previous_x = 0.5
        for tick in range(8):
            now = start + tick * 20_000_000
            target_x = 0.5 + tick * 0.005
            builder.observe_wrist_adapter({
                "type": "canonical_wrist_adapter", "schema_version": 1,
                "wrist_source": "keyboard", "source_seq": tick,
                "source_time_ns": now,
                "adapter_wrist_delta": [target_x - previous_x, 0.0, 0.0, 0.0, 0.0, 0.0],
                "wrist_pose_C": pose(x=target_x),
                "arm_target_q_rad": [0.0] * 4,
                "no_send": True,
            }, now)
            row = builder.build("keyboard", "glove", now)
            layer = {
                "hand_position_units": row["hand_position_units"],
                "hand_skeleton_C": row["hand_skeleton_C"],
            }
            row["canonical_nominal"] = dict(layer)
            row["canonical_measured"] = dict(layer)
            row["canonical_corrected"] = dict(layer)
            rows.append(row)
            previous_x = target_x
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            (session / "raw_input.jsonl").write_text(
                json.dumps({"source": "data_glove", "value": hand_value}) + "\n",
                encoding="utf-8",
            )
            (session / "canonical_goal.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in rows),
                encoding="utf-8",
            )
            result = verify_session(session, MODEL, tolerance=1e-9)
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(result["schema_v4_samples"], len(rows))

    def test_schema_v3_record_remains_read_only_visible_without_old_model(self):
        legacy = {
            "type": "canonical_goal", "schema_version": 3, "seq": 1,
            "hand_skeleton_w": np.zeros((21, 3)).tolist(),
        }
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            (session / "canonical_goal.jsonl").write_text(json.dumps(legacy) + "\n", encoding="utf-8")
            result = verify_session(session, MODEL)
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(result["legacy_read_only_samples"], 1)
        self.assertEqual(result["fk_checked_layers"], 0)


if __name__ == "__main__":
    unittest.main()
