import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from Hand_Tracker.master_fusion import (
    HI5_FOREARM_INDEX,
    HI5_HAND_INDEX,
    MasterWristFusion,
    euler_deg_to_quaternion_xyzw,
    quaternion_angle_deg_xyzw,
)


class MasterWristFusionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.calibration_path = self.root / "alignment.json"
        self.calibration_path.write_text(
            json.dumps(
                {
                    "schema_version": 2,
                    "calibration_id": "alignment-test",
                    "conventions": {"glove_orientation_index": 1},
                    "quality": {"quality_pass": True},
                    "rotation_alignment": {
                        "mode": "rigid_hand_eye",
                        "controller_to_glove_hand_quaternion_wxyz": [1, 0, 0, 0],
                        "glove_world_from_controller_world_quaternion_wxyz": [1, 0, 0, 0],
                    },
                }
            ),
            encoding="utf-8",
        )
        self.settings = {
            "mode": "m3",
            "alignment_calibration_path": str(self.calibration_path),
            "glove_euler_order": "zyx",
            "glove_stale_timeout_sec": 0.15,
            "controller_stale_timeout_sec": 0.4,
            "controller_correction_tau_sec": 10.0,
            "weight_ramp_tau_sec": 0.0001,
            "innovation_soft_deg": 10.0,
            "innovation_reject_deg": 40.0,
            "glove_calibration_id": "glove-test",
        }

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def pose(deadman=True, tracked=True):
        return {
            "seq": 7,
            "device_time": 12.0,
            "tracked": tracked,
            "deadman_active": deadman,
            "position": np.array([0.1, 0.2, 0.3]),
            "rotation": np.array([0.0, 0.0, 0.0, 1.0]),
            "received_at": time.monotonic(),
            "wall_time": time.time(),
        }

    @staticmethod
    def glove(frame_id, hand_euler=(0.0, 0.0, 0.0), forearm_euler=(0.0, 0.0, 0.0)):
        rotations = [0.0] * 63
        rotations[HI5_FOREARM_INDEX * 3 : HI5_FOREARM_INDEX * 3 + 3] = forearm_euler
        rotations[HI5_HAND_INDEX * 3 : HI5_HAND_INDEX * 3 + 3] = hand_euler
        return {
            "type": "glove_sensor_frame",
            "schema_version": 2,
            "frameId": frame_id,
            "valid": True,
            "rightRotations": rotations,
            "mapped_target_units": [2000, 2000, 1800, 1700, 1600, 1500],
            "glove_calibration_id": "glove-test",
            "receiver_wall_time_ns": time.time_ns(),
        }

    def test_hand_slot_drives_orientation_and_forearm_slot_is_ignored(self):
        fusion = MasterWristFusion(self.settings, self.root)
        fusion.last_update_monotonic -= 0.05
        pose, state = fusion.update(self.pose(), self.glove(1), 0.01)
        self.assertEqual(state["mode"], "fused")
        self.assertTrue(state["validity"]["finger"])
        baseline = np.asarray(pose["rotation"])

        fusion.last_update_monotonic -= 0.05
        pose, _state = fusion.update(
            self.pose(), self.glove(2, forearm_euler=(90.0, 0.0, 0.0)), 0.01
        )
        self.assertLess(quaternion_angle_deg_xyzw(baseline, pose["rotation"]), 0.01)

        fusion.last_update_monotonic -= 0.05
        pose, state = fusion.update(self.pose(), self.glove(3, hand_euler=(0.0, 0.0, 20.0)), 0.01)
        self.assertGreater(quaternion_angle_deg_xyzw(baseline, pose["rotation"]), 5.0)
        self.assertEqual(state["controller"]["position_m"], [0.1, 0.2, 0.3])

    def test_m3_rejects_calibration_mismatch_and_stale_fingers(self):
        fusion = MasterWristFusion(self.settings, self.root)
        frame = self.glove(1)
        frame["glove_calibration_id"] = "wrong"
        _pose, state = fusion.update(self.pose(), frame, 0.01)
        self.assertEqual(state["mode"], "controller_only")
        self.assertIn("glove_calibration_id_mismatch", state["degradation_reasons"])

        _pose, state = fusion.update(self.pose(), self.glove(2), 1.0)
        self.assertFalse(state["validity"]["finger"])
        self.assertEqual(state["glove"]["mapped_target_units"], [])

    def test_m3_rejects_excessive_paired_time_gap(self):
        fusion = MasterWristFusion(self.settings, self.root)
        frame = self.glove(1)
        frame["receiver_wall_time_ns"] = time.time_ns() + 500_000_000
        _pose, state = fusion.update(self.pose(), frame, 0.01)
        self.assertEqual(state["mode"], "controller_only")
        self.assertIn("paired_time_gap_exceeded", state["degradation_reasons"])
        self.assertGreater(state["orientation"]["paired_time_gap_ms"], 60.0)

    def test_large_euler_jump_is_rejected_without_j4_input_jump(self):
        fusion = MasterWristFusion(self.settings, self.root)
        fusion.last_update_monotonic -= 0.05
        pose, _state = fusion.update(self.pose(), self.glove(10), 0.01)
        before = np.asarray(pose["rotation"])
        fusion.last_update_monotonic -= 0.05
        pose, state = fusion.update(
            self.pose(), self.glove(0, hand_euler=(0.0, 0.0, 120.0)), 0.01
        )
        self.assertEqual(state["mode"], "controller_only")
        self.assertIn("orientation_innovation_rejected", state["degradation_reasons"])
        self.assertLess(quaternion_angle_deg_xyzw(before, pose["rotation"]), 1.0)

    def test_deadman_and_tracking_remain_controller_authority(self):
        fusion = MasterWristFusion(self.settings, self.root)
        _pose, state = fusion.update(self.pose(deadman=False), self.glove(1), 0.01)
        self.assertEqual(state["mode"], "invalid")
        self.assertFalse(state["validity"]["position"])
        self.assertFalse(state["validity"]["orientation"])

        _pose, state = fusion.update(self.pose(tracked=False), self.glove(2), 0.01)
        self.assertEqual(state["mode"], "invalid")

    def test_euler_quaternion_convention(self):
        quarter_turn = euler_deg_to_quaternion_xyzw([90.0, 0.0, 0.0], "xyz")
        expected = np.array([2**-0.5, 0.0, 0.0, 2**-0.5])
        np.testing.assert_allclose(quarter_turn, expected, atol=1e-8)

    def test_legacy_proxy_calibration_is_rejected(self):
        legacy = json.loads(self.calibration_path.read_text(encoding="utf-8"))
        legacy["schema_version"] = 1
        legacy.pop("conventions")
        self.calibration_path.write_text(json.dumps(legacy), encoding="utf-8")
        fusion = MasterWristFusion(self.settings, self.root)
        self.assertEqual(fusion.calibration_id, "")
        _pose, state = fusion.update(self.pose(), self.glove(1), 0.01)
        self.assertEqual(state["mode"], "controller_only")
        self.assertIn("alignment_calibration_invalid", state["degradation_reasons"])


if __name__ == "__main__":
    unittest.main()
