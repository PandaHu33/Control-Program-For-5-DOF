import unittest
from pathlib import Path

import numpy as np

from control_ui.wa100_kinematics import WA100Kinematics, rotation_rpy, transform


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "control_ui" / "wa100_kinematics_model.json"


class WA100KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kinematics = WA100Kinematics.load(MODEL)

    def test_compiled_urdf_structure(self):
        self.assertEqual(self.kinematics.root_link, "base_link")
        self.assertEqual(len(self.kinematics.model["links"]), 12)
        self.assertEqual(len(self.kinematics.joints), 11)
        self.assertEqual(
            self.kinematics.model["source_urdf_sha256"],
            "0cb3ce5342f1d7647a592a41d1751283ed34b3293abb3f00d9bfcceb67e5ad5b",
        )

    def test_open_motor_units_produce_zero_joint_angles(self):
        angles = self.kinematics.motor_units_to_joint_angles([2000] * 6)
        self.assertTrue(all(abs(value) < 1e-12 for value in angles.values()))

    def test_half_close_interpolates_calibration_table(self):
        angles = self.kinematics.motor_units_to_joint_angles([1000] * 6)
        def deadzone_smoothstep(bent_ratio, open_deadzone, close_deadzone):
            middle = (bent_ratio - open_deadzone) / (1.0 - open_deadzone - close_deadzone)
            return middle * middle * (3.0 - 2.0 * middle)

        thumb_ratio = deadzone_smoothstep(0.5, 0.04, 0.06)
        finger_ratio = deadzone_smoothstep(0.5, 0.06, 0.12)
        self.assertAlmostEqual(angles["Thumb_J1"], -1.57 * thumb_ratio, places=12)
        self.assertAlmostEqual(angles["Thumb_J2"], 1.57 * thumb_ratio, places=12)
        self.assertAlmostEqual(angles["Thumb_J3"], -1.57 * thumb_ratio, places=12)
        self.assertAlmostEqual(angles["Index Finger_J1"], 1.57 * finger_ratio, places=12)

    def test_glove_endpoint_deadzones_hold_open_pose_at_zero_angle(self):
        angles = self.kinematics.motor_units_to_joint_angles(
            [1950, 1950, 1900, 1900, 1900, 1900]
        )
        self.assertTrue(all(abs(value) < 1e-12 for value in angles.values()))

        below_deadzone = self.kinematics.motor_units_to_joint_angles(
            [1919, 1919, 1879, 1879, 1879, 1879]
        )
        self.assertGreater(abs(below_deadzone["Thumb_J1"]), 0.0)
        self.assertGreater(abs(below_deadzone["Thumb_J2"]), 0.0)
        self.assertGreater(abs(below_deadzone["Index Finger_J1"]), 0.0)

        raw_angles = self.kinematics.motor_units_to_joint_angles(
            [1950, 1950, 1900, 1900, 1900, 1900], apply_deadzones=False
        )
        self.assertGreater(abs(raw_angles["Thumb_J1"]), 0.0)
        self.assertGreater(abs(raw_angles["Index Finger_J1"]), 0.0)

    def test_glove_close_deadzone_holds_fully_closed_pose(self):
        angles = self.kinematics.motor_units_to_joint_angles([100, 100, 200, 200, 200, 200])
        self.assertAlmostEqual(angles["Thumb_J1"], -1.57, places=12)
        self.assertAlmostEqual(angles["Thumb_J2"], 1.57, places=12)
        self.assertAlmostEqual(angles["Index Finger_J1"], 1.57, places=12)

    def test_skeleton_contains_all_feature_points_and_tips(self):
        skeleton = self.kinematics.skeleton_from_motor_units([2000] * 6)
        self.assertEqual(set(skeleton), {"thumb", "index", "middle", "ring", "little"})
        self.assertEqual(skeleton["thumb"].shape, (5, 3))
        for finger in ("index", "middle", "ring", "little"):
            self.assertEqual(skeleton[finger].shape, (4, 3))
        self.assertTrue(all(np.all(np.isfinite(points)) for points in skeleton.values()))

    def test_thumb_exposes_four_physical_feature_points(self):
        points = self.kinematics.thumb_feature_points_from_motor_units([1000] * 6)
        self.assertEqual(points.shape, (4, 3))
        np.testing.assert_allclose(
            points,
            self.kinematics.skeleton_from_motor_units([1000] * 6)["thumb"][1:],
            atol=1e-12,
        )

    def test_thumb_pitch_keeps_first_link_rigid_and_bends_j2_j3(self):
        pitch_open = self.kinematics.thumb_feature_points_from_motor_units(
            [2000, 1000, 2000, 2000, 2000, 2000]
        )
        pitch_closed = self.kinematics.thumb_feature_points_from_motor_units(
            [0, 1000, 2000, 2000, 2000, 2000]
        )
        # The first link is upstream of both flexion joints, so pitch cannot
        # change either of its endpoints or its direction.
        np.testing.assert_allclose(pitch_open[:2], pitch_closed[:2], atol=1e-12)
        # J2 moves the distal joint, and coupled J3 additionally changes the
        # final link direction and fingertip position.
        self.assertGreater(np.linalg.norm(pitch_open[2] - pitch_closed[2]), 0.01)
        self.assertGreater(np.linalg.norm(pitch_open[3] - pitch_closed[3]), 0.01)
        closed_angles = self.kinematics.motor_units_to_joint_angles(
            [0, 1000, 2000, 2000, 2000, 2000]
        )
        self.assertAlmostEqual(closed_angles["Thumb_J2"], 1.57, places=6)
        self.assertAlmostEqual(closed_angles["Thumb_J3"], -1.57, places=6)

    def test_thumb_three_links_are_collinear_when_flexion_is_open(self):
        for yaw_units in (0, 500, 1000, 1500, 2000):
            points = self.kinematics.thumb_feature_points_from_motor_units(
                [2000, yaw_units, 2000, 2000, 2000, 2000]
            )
            directions = np.diff(points, axis=0)
            directions /= np.linalg.norm(directions, axis=1)[:, None]
            np.testing.assert_allclose(directions[0], directions[1], atol=1e-12)
            np.testing.assert_allclose(directions[1], directions[2], atol=1e-12)

    def test_thumb_yaw_encoder_zero_is_normal_to_dorsum_plane(self):
        points = self.kinematics.thumb_feature_points_from_motor_units(
            [2000, 0, 2000, 2000, 2000, 2000]
        )
        directions = np.diff(points, axis=0)
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        expected = np.array([0.0, -1.0, 0.0])
        for direction in directions:
            np.testing.assert_allclose(direction, expected, atol=1e-12)
        dorsum_normal = np.asarray(
            self.kinematics.model["calibration"]["thumb_motion_model"]["dorsum_plane_normal"]
        )
        self.assertAlmostEqual(abs(float(np.dot(directions[0], dorsum_normal))), 1.0, places=12)

    def test_thumb_yaw_rotates_all_three_links_without_changing_lengths(self):
        yaw_open = self.kinematics.thumb_feature_points_from_motor_units(
            [1000, 2000, 2000, 2000, 2000, 2000]
        )
        yaw_closed = self.kinematics.thumb_feature_points_from_motor_units(
            [1000, 0, 2000, 2000, 2000, 2000]
        )
        np.testing.assert_allclose(
            np.linalg.norm(np.diff(yaw_open, axis=0), axis=1),
            np.linalg.norm(np.diff(yaw_closed, axis=0), axis=1),
            atol=1e-12,
        )
        self.assertGreater(np.linalg.norm(yaw_open[1:] - yaw_closed[1:]), 0.01)

    def test_thumb_yaw_is_orthogonal_to_both_flexion_axes(self):
        zero_angles = {name: 0.0 for name in self.kinematics.joints}
        links = self.kinematics.link_transforms(zero_angles)

        def axis_in_root(name):
            joint = self.kinematics.joints[name]
            axis = (
                links[joint.parent_link][:3, :3]
                @ rotation_rpy(joint.origin_rpy_rad)
                @ np.asarray(joint.axis)
            )
            return axis / np.linalg.norm(axis)

        yaw_axis = axis_in_root("Thumb_J1")
        j2_axis = axis_in_root("Thumb_J2")
        j3_axis = axis_in_root("Thumb_J3")
        self.assertAlmostEqual(float(np.dot(yaw_axis, j2_axis)), 0.0, places=5)
        self.assertAlmostEqual(float(np.dot(yaw_axis, j3_axis)), 0.0, places=5)
        self.assertAlmostEqual(abs(float(np.dot(j2_axis, j3_axis))), 1.0, places=5)

    def test_base_transform_places_model_in_world_frame(self):
        world_from_hand = transform(translation=[1.0, 2.0, 3.0])
        skeleton = self.kinematics.skeleton_from_motor_units([2000] * 6, world_from_hand)
        for points in skeleton.values():
            np.testing.assert_allclose(points[0], [1.0, 2.0, 3.0], atol=1e-12)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            self.kinematics.motor_units_to_joint_angles([2000] * 5)


if __name__ == "__main__":
    unittest.main()
