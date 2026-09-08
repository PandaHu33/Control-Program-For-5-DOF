"""Offline behavior checks: measurements enter canonical, never actuator filtering."""
import math
import unittest
from pathlib import Path
from dataclasses import replace

import numpy as np

from Hand_Tracker.unity_hand_udp_bridge import XrSkeletonOneEuroFilter
from control_ui.canonical_goal import WA100PinchDetector, CanonicalGoalBuilder
from control_ui.canonical_planning import CanonicalPlannerConfig, CanonicalWristPlanner


def pose(x=0.5, roll=0.0):
    return {"position_m": [x, 0.0, 0.0],
            "orientation_xyzw": [math.sin(roll / 2), 0, 0, math.cos(roll / 2)]}


def thumb_targets(points):
    """Same geometric calibration as the VR acquisition mapper (pitch, yaw)."""
    def unit(v):
        return v / np.linalg.norm(v)
    yaw = abs(np.dot(unit(points[4] - points[1]), unit(points[5] - points[0])))
    angle = math.degrees(math.acos(np.clip(np.dot(unit(points[1] - points[2]),
                                                           unit(points[4] - points[2])), -1, 1)))
    return np.clip([(165.31 - angle) / (165.31 - 125),
                    (0.988191 - yaw) / (0.988191 - 0.558984)], 0, 1) * 2000


class AcquisitionStabilityTests(unittest.TestCase):
    def test_wrist_noise_and_slow_motion(self):
        config = CanonicalPlannerConfig()
        planners = [CanonicalWristPlanner(replace(config, vr_stability_enabled=False)),
                    CanonicalWristPlanner(config)]
        rng = np.random.default_rng(81)
        outputs = [[], []]
        for tick in range(800):
            noise = rng.normal(0, 0.001, 2)
            for i, planner in enumerate(planners):
                now = 1_000_000_000 + tick * 20_000_000
                result, _, _ = planner.step(source="pico_wrist", target_pose=pose(0.5 + noise[0], noise[1] * 4),
                                            source_receive_ns=now, now_ns=now)
                outputs[i].append([result["position_m"][0], planner.roll])
        ratio = np.std(outputs[1][100:], axis=0) / np.std(outputs[0][100:], axis=0)
        print("Wrist position/roll noise reduction (%):", (100 * (1 - ratio)).round(1).tolist())
        self.assertTrue(np.all(ratio < 0.5), ratio)
        for tick in range(800, 1800):
            now = 1_000_000_000 + tick * 20_000_000
            target = 0.5 + (tick - 800) * 0.00002
            result, _, _ = planners[1].step(source="pico_wrist", target_pose=pose(target),
                                           source_receive_ns=now, now_ns=now)
        self.assertLess(abs(result["position_m"][0] - target), 0.006)

    def test_thumb_noise_reduction_against_previous_filter(self):
        points = np.zeros((21, 3))
        points[1] = [0.018, 0.012, 0]
        points[2] = [0.036, 0.023, 0]
        points[4] = [0.055, 0.057, 0.009]
        points[5] = [0.06, 0.015, 0]
        filters = [XrSkeletonOneEuroFilter(adaptive=False), XrSkeletonOneEuroFilter()]
        outputs = [[], []]
        rng = np.random.default_rng(8)
        for tick in range(1200):
            sample = points + rng.normal(0, 0.00035, points.shape)
            for i, filter_ in enumerate(filters):
                outputs[i].append(thumb_targets(np.array(filter_.apply(sample.ravel().tolist(), tick * .02)).reshape(21, 3)))
        ratio = np.std(outputs[1][200:], axis=0) / np.std(outputs[0][200:], axis=0)
        print("Thumb pitch/yaw noise reduction (%):", (100 * (1 - ratio)).round(1).tolist())
        self.assertTrue(np.all(ratio < 0.5), ratio)

    def test_translation_does_not_deform_hand_and_duplicate_does_not_advance(self):
        points = np.arange(63, dtype=float).reshape(21, 3) * 0.002
        filter_ = XrSkeletonOneEuroFilter()
        for tick in range(100):
            moved = points + [tick * .003, 0, 0]
            output = np.array(filter_.apply(moved.ravel().tolist(), tick * .02)).reshape(21, 3)
            np.testing.assert_allclose(output - output[0], points - points[0], atol=1e-12)
        duplicate = filter_.apply((points + 1).ravel().tolist(), 1.98)
        np.testing.assert_allclose(duplicate, output.ravel())
        recovered = filter_.apply((points + 1).ravel().tolist(), 2.5)
        np.testing.assert_allclose(recovered, (points + 1).ravel())

    def test_pinch_hold_release_and_gap(self):
        points = np.zeros((21, 3))
        points[5] = [1, 0, 0]
        points[8] = [.3, 0, 0]
        points[12] = [2, 0, 0]
        detector = WA100PinchDetector()
        for tick in range(500):
            points[8, 0] = .3 if tick < 10 else .59 + .05 * math.sin(tick)
            result = detector.update(points, tick * .02)
            if tick >= 4:
                self.assertEqual(result["active_anchor"], "thumb_index")
        points[8, 0] = 1
        for tick in range(500, 504):
            result = detector.update(points, tick * .02)
        self.assertEqual(result["active_anchor"], "none")
        points[8, 0] = .3
        result = detector.update(points, 11)
        self.assertEqual(result["active_anchor"], "none")

    def test_bad_parameters_and_nan_are_rejected(self):
        with self.assertRaises(ValueError):
            XrSkeletonOneEuroFilter(thumb_cutoff_hz=float("nan"))
        with self.assertRaises(ValueError):
            XrSkeletonOneEuroFilter().apply([float("nan")] * 63)
        with self.assertRaises(ValueError):
            CanonicalPlannerConfig.from_mapping({"planner_vr_still_cutoff_hz": 0})

    def test_finger_motion_and_spike_at_multiple_rates(self):
        for hz in (30, 50, 90):
            filter_ = XrSkeletonOneEuroFilter()
            sample = np.zeros(63)
            filter_.apply(sample.tolist(), 0)
            for tick in range(1, hz + 1):
                sample[12] = min(.025, tick / hz * .1)
                result = filter_.apply(sample.tolist(), tick / hz)
            self.assertLess(abs(result[12] - .025), .003)
            sample[12] += .2
            spike = filter_.apply(sample.tolist(), 1 + 1 / hz)
            self.assertLess(spike[12] - result[12], .015)

    def test_non_vr_sources_keep_old_planner_behavior(self):
        for source in ("keyboard", "gamepad", "vr_controller"):
            planners = [CanonicalWristPlanner(CanonicalPlannerConfig()),
                        CanonicalWristPlanner(CanonicalPlannerConfig(vr_stability_enabled=False))]
            for tick in range(100):
                now = 1_000_000_000 + tick * 20_000_000
                outputs = [p.step(source=source, target_pose=pose(.5 + .05 * math.sin(tick * .1)),
                                  source_receive_ns=now, now_ns=now)[0] for p in planners]
                self.assertEqual(outputs[0], outputs[1])

    def test_hand_reconnect_and_duplicate_lifecycle(self):
        builder = CanonicalGoalBuilder(Path(__file__).resolve().parents[1] / "control_ui/wa100_kinematics_model.json")
        value = {"frameId": 20, "mapped_target_units": [800] * 6,
                 "xrSkeletonFilter": {"right_generation": 1}}
        self.assertTrue(builder.observe_hand_input("pico_hand", value, 1_000_000_000)[0])
        self.assertFalse(builder.observe_hand_input("pico_hand", value, 1_020_000_000)[0])
        self.assertEqual(builder.hand["receive_utc_ns"], 1_000_000_000)
        value["frameId"] = 0
        value["xrSkeletonFilter"]["right_generation"] = 2
        self.assertTrue(builder.observe_hand_input("pico_hand", value, 1_040_000_000)[0])


if __name__ == "__main__":
    unittest.main()
