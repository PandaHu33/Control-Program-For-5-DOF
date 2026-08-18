import json
import math
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from control_ui.canonical_goal import (
    CANONICAL_JOINT_NAMES,
    CANONICAL_PARENTS,
    HAND_ALL_VALID_MASK,
    WRIST_SUPPORTED_MASK,
    CanonicalGoalBuilder,
    FrozenHi5HandModel,
    integrate_wrist_pose,
    pico_skeleton_in_wrist_frame,
    validate_canonical_goal,
)
from control_ui.wa100_kinematics import WA100Kinematics


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "control_ui" / "hi5_hand_model_v1.json"
MODEL_V2 = ROOT / "control_ui" / "hi5_hand_model_v2.json"
MODEL_V3 = ROOT / "control_ui" / "hi5_hand_model_v3.json"
MODEL_V4 = ROOT / "control_ui" / "hi5_hand_model_v4.json"
MODEL_V5 = ROOT / "control_ui" / "hi5_hand_model_v5.json"
MODEL_V6 = ROOT / "control_ui" / "hi5_hand_model_v6.json"
MODEL_V7 = ROOT / "control_ui" / "hi5_hand_model_v7.json"
MODEL_V8 = ROOT / "control_ui" / "hi5_hand_model_v8.json"


def pico_points():
    model = FrozenHi5HandModel.load(MODEL)
    points = model.offsets.copy()
    for index in range(1, 21):
        points[index] += points[model.source_rotation_indices[index] or 0] * 0.0
    # Create non-degenerate world-space landmarks with the canonical index and
    # middle MCP defining the same palm plane expected by the runtime adapter.
    points[0] = [1.0, 2.0, 3.0]
    points[5] = [1.04, 2.01, 3.0]
    points[9] = [1.01, 2.04, 3.0]
    for index in range(1, 21):
        if index not in {5, 9}:
            points[index] = np.asarray([1.0, 2.0, 3.0]) + model.offsets[index]
    return points


class CanonicalGoalTests(unittest.TestCase):
    def test_v7_moves_only_wa100_thumb_chain_wristward(self):
        v6 = FrozenHi5HandModel.load(MODEL_V6)
        v7 = FrozenHi5HandModel.load(MODEL_V7)
        rotations = v7.neutral_source_rotations_deg.copy()
        targets = np.asarray([546, 254, 0, 484, 477, 1115], dtype=float)
        hand_v6 = v6.skeleton(rotations, targets)
        hand_v7 = v7.skeleton(rotations, targets)
        np.testing.assert_allclose(hand_v7[1:5], hand_v6[1:5] + [-0.025, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(hand_v7[5:], hand_v6[5:], atol=1e-12)
        self.assertLess(hand_v7[1, 0], hand_v7[5, 0])
        np.testing.assert_allclose(
            np.linalg.norm(np.diff(hand_v7[1:5], axis=0), axis=1),
            [0.017838791999460053, 0.03, 0.036],
            atol=1e-12,
        )

    def test_v8_scales_complete_hi5_skeleton_around_wrist_to_pico_size(self):
        v7 = FrozenHi5HandModel.load(MODEL_V7)
        v8 = FrozenHi5HandModel.load(MODEL_V8)
        rotations = v7.neutral_source_rotations_deg.copy()
        targets = np.asarray([1200, 800, 2000, 2000, 2000, 2000], dtype=float)
        hand_v7 = v7.skeleton(rotations, targets)
        hand_v8 = v8.skeleton(rotations, targets)
        np.testing.assert_allclose(hand_v8[0], np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(hand_v8, hand_v7 * v8.canonical_scale, atol=1e-12)
        middle_chain = sum(
            np.linalg.norm(hand_v8[joint] - hand_v8[CANONICAL_PARENTS[joint]])
            for joint in (9, 10, 11, 12)
        )
        self.assertAlmostEqual(middle_chain, 0.175672, places=6)

    def test_v6_thumb_is_exact_registered_wa100_reference(self):
        model = FrozenHi5HandModel.load(MODEL_V6)
        wa100 = WA100Kinematics.load(ROOT / "control_ui" / "wa100_kinematics_model.json")
        rotation = np.asarray([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
        rotations = model.neutral_source_rotations_deg.copy()
        for targets in (np.full(6, 2000.0), np.asarray([0, 0, 2000, 2000, 2000, 2000], dtype=float)):
            canonical = model.skeleton(rotations, targets)[1:5]
            expected = (rotation @ wa100.thumb_feature_points_from_motor_units(targets).T).T
            np.testing.assert_allclose(canonical, expected, atol=1e-12)
        self.assertEqual(
            model.wa100_thumb_kinematics.model["calibration"]["calibration_id"],
            "wa100_provisional_deadzone_v4",
        )

    def test_v5_is_right_hand_palm_down_rotation_of_v4(self):
        v4 = FrozenHi5HandModel.load(MODEL_V4)
        v5 = FrozenHi5HandModel.load(MODEL_V5)
        rotations = v5.neutral_source_rotations_deg.copy()
        for source_index in (6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20):
            rotations[source_index, 2] -= 35.0
        targets = np.asarray([500, 250, 400, 500, 600, 700], dtype=float)
        hand_v4 = v4.skeleton(rotations, targets)
        hand_v5 = v5.skeleton(rotations, targets)
        np.testing.assert_allclose(hand_v5, hand_v4 * [1.0, -1.0, -1.0], atol=1e-12)
        self.assertGreater(hand_v5[5, 1], hand_v5[17, 1])
        self.assertGreater(hand_v5[1, 1], 0.0)

    def test_pico_right_hand_is_laterally_mirrored_with_thumb_side_left(self):
        points = np.zeros((21, 3), dtype=float)
        points[5] = [1.0, 0.0, 0.0]
        points[9] = [1.0, 1.0, 0.0]
        points[17] = [1.0, 2.0, 0.0]
        points[4] = [1.0, -1.0, 0.0]
        local = pico_skeleton_in_wrist_frame(points)
        np.testing.assert_allclose(local[5], [1.0, 0.0, 0.0], atol=1e-12)
        self.assertLess(local[17, 1], local[5, 1])
        self.assertGreater(local[4, 1], local[5, 1])

    def test_v4_matches_pico_basis_and_separates_thumb_pitch_from_yaw(self):
        model = FrozenHi5HandModel.load(MODEL_V4)
        neutral = model.neutral_source_rotations_deg.copy()
        open_targets = np.full(6, 2000.0)
        open_hand = model.skeleton(neutral, open_targets)
        self.assertGreater(open_hand[5, 0], 0.0)
        self.assertLess(open_hand[5, 1], open_hand[17, 1])
        np.testing.assert_allclose(open_hand[:, 2], 0.0, atol=1e-12)

        thumb_lengths = np.linalg.norm(np.diff(open_hand[[1, 2, 3, 4]], axis=0), axis=1)
        np.testing.assert_allclose(thumb_lengths, [0.030, 0.022, 0.016], atol=1e-12)
        self.assertAlmostEqual(float(thumb_lengths.sum()), 0.068, places=12)

        yaw_targets = open_targets.copy()
        yaw_targets[1] = 0.0
        yaw_hand = model.skeleton(neutral, yaw_targets)
        self.assertGreater(yaw_hand[4, 1], open_hand[4, 1])
        self.assertAlmostEqual(float(yaw_hand[4, 2]), 0.0, places=12)

        pitch_targets = open_targets.copy()
        pitch_targets[0] = 0.0
        pitch_hand = model.skeleton(neutral, pitch_targets)
        self.assertGreater(pitch_hand[4, 2], open_hand[4, 2])
        self.assertAlmostEqual(float(pitch_hand[4, 1]), float(open_hand[4, 1]), places=12)

    def test_v3_four_finger_flexion_curls_opposite_to_v2(self):
        v2 = FrozenHi5HandModel.load(MODEL_V2)
        v3 = FrozenHi5HandModel.load(MODEL_V3)
        neutral = v3.neutral_source_rotations_deg.copy()
        flexed = neutral.copy()
        for source_index in (6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20):
            flexed[source_index, 2] -= 45.0

        np.testing.assert_allclose(v3.skeleton(neutral)[:, 2], 0.0, atol=1e-12)
        # Negative glove Z is flexion in the captured stream.  v3 deliberately
        # mirrors only that palm-plane bend while preserving link lengths.
        skeleton_v2 = v2.skeleton(flexed)
        skeleton_v3 = v3.skeleton(flexed)
        for base, tip in ((5, 8), (9, 12), (13, 16), (17, 20)):
            self.assertGreater(skeleton_v2[tip, 1] - skeleton_v2[base, 1], 0.0)
            self.assertLess(skeleton_v3[tip, 1] - skeleton_v3[base, 1], 0.0)
        lengths_v2 = np.linalg.norm(np.diff(skeleton_v2[[5, 6, 7, 8]], axis=0), axis=1)
        lengths_v3 = np.linalg.norm(np.diff(skeleton_v3[[5, 6, 7, 8]], axis=0), axis=1)
        np.testing.assert_allclose(lengths_v3, lengths_v2, atol=1e-12)

    def test_v2_glove_neutral_is_planar_and_thumb_yaw_uses_device_direction(self):
        model = FrozenHi5HandModel.load(MODEL_V2)
        neutral = model.neutral_source_rotations_deg.copy()
        open_skeleton = model.skeleton(neutral)
        np.testing.assert_allclose(open_skeleton[:, 2], 0.0, atol=1e-12)

        # On this Hi5 stream thumb opposition makes source Z more negative.
        # The corrected sign must move the tip from its open +Y side toward
        # the index/middle MCPs (decreasing wrist-frame Y).
        opposing = neutral.copy()
        opposing[2, 2] -= 20.0
        opposing[3, 2] -= 20.0
        opposed_skeleton = model.skeleton(opposing)
        self.assertLess(opposed_skeleton[4, 1], open_skeleton[4, 1])

    def test_world_translation_and_body_rotation_integrate_consistently(self):
        pose = {"position_m": [0.1, 0.2, 0.3], "orientation_xyzw": [0, 0, 0, 1]}
        delta = [0.01, -0.02, 0.03, math.pi / 2, 0, 0]
        updated = integrate_wrist_pose(pose, delta)
        np.testing.assert_allclose(updated["position_m"], [0.11, 0.18, 0.33], atol=1e-12)
        np.testing.assert_allclose(
            updated["orientation_xyzw"], [math.sqrt(0.5), 0, 0, math.sqrt(0.5)], atol=1e-12
        )

    def test_pico_and_hi5_share_21_node_contract(self):
        pico = pico_skeleton_in_wrist_frame(pico_points().reshape(-1))
        hi5 = FrozenHi5HandModel.load(MODEL).skeleton([0.0] * 63)
        self.assertEqual(pico.shape, (21, 3))
        self.assertEqual(hi5.shape, (21, 3))
        self.assertEqual(len(CANONICAL_JOINT_NAMES), 21)
        np.testing.assert_allclose(pico[0], [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(hi5[0], [0, 0, 0], atol=1e-12)

    def test_builder_emits_e1_condition_masks_calibrations_and_targets(self):
        builder = CanonicalGoalBuilder(MODEL, stale_timeout_sec=0.5)
        now = time.time_ns()
        ok, reason = builder.observe_wrist_adapter({
            "type": "canonical_wrist_adapter", "schema_version": 1,
            "wrist_source": "vr_controller", "source_seq": 3, "source_time_ns": now,
            "adapter_wrist_delta": [0.01, 0, 0, 0, 0, 0],
            "wrist_pose_C": {"position_m": [0.5, 0, 0.1], "orientation_xyzw": [0, 0, 0, -1]},
            "arm_target_q_rad": [0.1, 0.2, 0.3, 0.4], "wrist_dof_mask": WRIST_SUPPORTED_MASK,
            "mapping_id": "controller-test", "no_send": True,
        }, now)
        self.assertTrue(ok, reason)
        ok, reason = builder.observe_hand_input("data_glove", {
            "stream": "hand_skeleton", "frameId": 9, "rightRotations": [0.0] * 63,
            "mapped_target_units": [100, 200, 300, 400, 500, 600],
            "glove_calibration_id": "glove-test",
        }, now)
        self.assertTrue(ok, reason)
        goal = builder.build("controller_delta", "glove", now)
        self.assertEqual(goal["condition_id"], "M2-VR-GLOVE")
        self.assertEqual(goal["adapter_wrist_delta"], [0.01, 0, 0, 0, 0, 0])
        self.assertTrue(goal["no_send"])
        self.assertEqual(goal["validity"]["wrist_dof_mask"], WRIST_SUPPORTED_MASK)
        self.assertEqual(goal["validity"]["hand_node_mask"], HAND_ALL_VALID_MASK)
        self.assertEqual(goal["final_targets"]["hand_target_units"], [100, 200, 300, 400, 500, 600])
        self.assertEqual(goal["wrist_pose_C"]["orientation_xyzw"], [0.0, 0.0, 0.0, 1.0])
        self.assertTrue(validate_canonical_goal(goal)[0])
        repeated = builder.build("controller_delta", "glove", now + 20_000_000)
        self.assertEqual(repeated["seq"], goal["seq"] + 1)
        self.assertEqual(repeated["adapter_wrist_delta"], [0.0] * 6)
        self.assertEqual(repeated["wrist_pose_C"], goal["wrist_pose_C"])

    def test_stale_and_calibration_mismatch_fail_only_the_corresponding_subspace(self):
        builder = CanonicalGoalBuilder(MODEL, stale_timeout_sec=0.01)
        now = time.time_ns()
        builder.observe_wrist_adapter({
            "type": "canonical_wrist_adapter", "schema_version": 1,
            "wrist_source": "keyboard", "source_seq": 1,
            "adapter_wrist_delta": [0] * 6,
            "wrist_pose_C": {"position_m": [0.5, 0, 0], "orientation_xyzw": [0, 0, 0, 1]},
            "arm_target_q_rad": [0] * 4,
        }, now)
        ok, reason = builder.observe_hand_input("data_glove", {
            "rightRotations": [0.0] * 63,
            "hand_model_calibration_id": "wrong",
            "mapped_target_units": [1000] * 6,
        }, now)
        self.assertFalse(ok)
        self.assertEqual(reason, "hand_model_calibration_id_mismatch")

        builder.observe_hand_input("data_glove", {
            "rightRotations": [0.0] * 63,
            "mapped_target_units": [1000] * 6,
            "glove_calibration_id": "valid",
        }, now)
        goal = builder.build("keyboard", "glove", now + 20_000_000)
        self.assertEqual(goal["validity"], {"wrist_dof_mask": 0, "hand_node_mask": 0})
        self.assertIn("wrist_stale", goal["invalid_reasons"])
        self.assertIn("hand_stale", goal["invalid_reasons"])

    def test_browser_and_wa100_sources_contain_canonical_contract_hooks(self):
        html = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
        cpp = (ROOT / "wa100-sdk-publish" / "examples" / "udp_receiver_unity.cpp").read_text(encoding="utf-8")
        bridge = (ROOT / "control_ui" / "bridge.py").read_text(encoding="utf-8")
        config = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")
        self.assertIn('type: "canonical_wrist_adapter"', html)
        self.assertIn("solveArmKinematics(canonicalWristPose.position_m)", html)
        self.assertIn("MapCanonicalGoalToTargets", cpp)
        self.assertIn('native_payload["mapped_target_units"]', cpp)
        self.assertIn("CANONICAL_NATIVE_FALLBACK_TIMEOUT_MS 300", cpp)
        self.assertIn("Canonical recovered; native hand fallback disabled", cpp)
        self.assertIn("Control/preset packets always apply", cpp)
        self.assertIn("else if (native_hand_packet)", cpp)
        self.assertIn('id="e1NoSend"', html)
        self.assertIn('id="canonicalSkeletonCanvas"', html)
        self.assertIn('class="canonical-camera-row"', html)
        self.assertNotIn('id="canonicalValidity"', html)
        self.assertIn('class="metric-group"', html)
        self.assertIn('class="metric-group two"', html)
        self.assertIn('const CANONICAL_WS_URL = "ws://localhost:8080/canonical-goal"', html)
        self.assertIn("setInterval(commitLatestCanonicalGoal, 1000)", html)
        self.assertIn("rotateCanonicalPoint(local, quaternion)", html)
        self.assertIn("canonicalWristDelta", html)
        self.assertIn("const palmSurface = [0, 5, 9, 13, 17]", html)
        self.assertIn("applyPositionDelta(-axisY * step, lateralIntent * step, -zAxis * step)", html)
        self.assertIn('"ui-gamepad-mapping-v5-operator-facing-signs"', html)
        self.assertIn('"ui-keyboard-mapping-v4-operator-facing-signs"', html)
        self.assertIn('and not bool(goal.get("no_send"))', bridge)
        self.assertIn("publish_hz: 50.0", config)
        self.assertIn("def canonical_publish_loop()", bridge)
        self.assertEqual(bridge.count("publish_canonical_goal("), 2)


if __name__ == "__main__":
    unittest.main()
