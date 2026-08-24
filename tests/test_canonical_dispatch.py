import struct
import time
import unittest
from pathlib import Path

from control_ui import bridge
from control_ui.canonical_goal import CanonicalGoalBuilder
from control_ui.canonical_planning import CanonicalPlannerConfig
from control_ui.semantic_admittance import arm_inverse_kinematics


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "control_ui" / "wa100_kinematics_model.json"


class _DecodedFrame:
    def __init__(self, emergency_stop=False):
        self.emergency_stop = emergency_stop


class CanonicalDispatchTests(unittest.TestCase):
    def setUp(self):
        with bridge.SLAVE_COMMAND_LOCK:
            self.previous = bridge.LATEST_SLAVE_COMMAND
        self.previous_enabled = bridge.CANONICAL_CONTROL_ENABLED
        bridge.CANONICAL_CONTROL_ENABLED = True

    def tearDown(self):
        with bridge.SLAVE_COMMAND_LOCK:
            bridge.LATEST_SLAVE_COMMAND = self.previous
        bridge.CANONICAL_CONTROL_ENABLED = self.previous_enabled

    def frame(self):
        return bytes(bridge.H5_FRAME_CRC_OFFSET + 4)

    def test_fresh_planned_reference_replaces_only_j1_through_j4(self):
        targets = [0.1, -0.2, 0.3, -0.4]
        with bridge.SLAVE_COMMAND_LOCK:
            bridge.LATEST_SLAVE_COMMAND = {
                "timestamp_ns": time.time_ns(),
                "valid_for_dispatch": True,
                "no_send": False,
                "arm_target_q_rad": targets,
            }
        result = bridge.apply_canonical_arm_reference(self.frame(), _DecodedFrame())
        self.assertIsInstance(result, bytes)
        actual = struct.unpack_from("<4f", result, 145)
        for observed, expected in zip(actual, targets):
            self.assertAlmostEqual(observed, expected, places=6)

    def test_stale_or_invalid_reference_suppresses_normal_frame(self):
        with bridge.SLAVE_COMMAND_LOCK:
            bridge.LATEST_SLAVE_COMMAND = {
                "timestamp_ns": time.time_ns() - 200_000_000,
                "valid_for_dispatch": True,
                "no_send": False,
                "arm_target_q_rad": [0.0] * 4,
            }
        self.assertIsNone(
            bridge.apply_canonical_arm_reference(self.frame(), _DecodedFrame())
        )

    def test_emergency_stop_is_never_suppressed_or_rewritten(self):
        original = self.frame()
        with bridge.SLAVE_COMMAND_LOCK:
            bridge.LATEST_SLAVE_COMMAND = None
        self.assertEqual(
            bridge.apply_canonical_arm_reference(original, _DecodedFrame(True)),
            original,
        )

    def test_bundle_recomputes_joint_target_from_planned_pose(self):
        config = CanonicalPlannerConfig(
            keyboard_profile="off", gamepad_profile="off",
            pico_wrist_profile="off", vr_controller_profile="off",
        )
        builder = CanonicalGoalBuilder(MODEL, planner_config=config)
        now = time.time_ns()
        builder.observe_wrist_adapter({
            "type": "canonical_wrist_adapter", "schema_version": 1,
            "wrist_source": "keyboard", "source_seq": 1,
            "source_time_ns": now, "adapter_wrist_delta": [0.0] * 6,
            "wrist_pose_C": {
                "position_m": [0.5, 0.1, 0.15],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "arm_target_q_rad": [9.0, 9.0, 9.0, 9.0],
        }, now)
        builder.observe_hand_input("data_glove", {
            "frameId": 1, "rightRotations": [0.0] * 63,
            "mapped_target_units": [1000.0] * 6,
            "glove_calibration_id": "test",
        }, now)
        goal = builder.build("keyboard", "glove", now)
        bundle = bridge._canonical_semantic_bundle(goal, now)
        expected = arm_inverse_kinematics([0.5, 0.1, 0.15], 0.0)
        actual = bundle["final_targets"]["arm_target_q_rad"]
        for observed, target in zip(actual, expected):
            self.assertAlmostEqual(observed, target, places=12)
        self.assertNotEqual(actual, [9.0, 9.0, 9.0, 9.0])

    def test_bundle_keeps_nominal_targets_separate_from_actual_encoders(self):
        builder = CanonicalGoalBuilder(MODEL, planner_config=CanonicalPlannerConfig(
            keyboard_profile="off", gamepad_profile="off",
            pico_wrist_profile="off", vr_controller_profile="off",
        ))
        now = time.time_ns()
        builder.observe_wrist_adapter({
            "type": "canonical_wrist_adapter", "schema_version": 1,
            "wrist_source": "keyboard", "source_seq": 2,
            "adapter_wrist_delta": [0.0] * 6,
            "wrist_pose_C": {"position_m": [0.5, 0.0, 0.15], "orientation_xyzw": [0, 0, 0, 1]},
            "arm_target_q_rad": [0.0] * 4,
        }, now)
        builder.observe_hand_input("data_glove", {"mapped_target_units": [1000.0] * 6}, now)
        with bridge.CANONICAL_MEASURED_LOCK:
            previous = dict(bridge.CANONICAL_MEASURED)
            bridge.CANONICAL_MEASURED.update({
                "arm": {"receive_utc_ns": now, "actual_q_rad": [0.0] * 5},
                "hand": {"receive_utc_ns": now, "position_valid_mask": 0x3F,
                         "actual_position_units": [1500.0] * 6},
                "current": {"timestamp_ns": now, "residual": [0.0] * 6},
            })
        try:
            bundle = bridge._canonical_semantic_bundle(builder.build("keyboard", "glove", now), now)
        finally:
            with bridge.CANONICAL_MEASURED_LOCK:
                bridge.CANONICAL_MEASURED.clear(); bridge.CANONICAL_MEASURED.update(previous)
        self.assertEqual(bundle["canonical_nominal"]["hand_position_units"], [1000.0] * 6)
        self.assertEqual(bundle["canonical_measured"]["hand_position_units"], [1500.0] * 6)
        self.assertEqual(bundle["final_targets"]["hand_target_units"], [1000.0] * 6)


if __name__ == "__main__":
    unittest.main()
