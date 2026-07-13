import math
import unittest

from control_ui.experiment_metrics import compute_trial_summary, normalize_raw_sample


def sample(elapsed_ms, desired, actual):
    return {
        "elapsed_ms": elapsed_ms,
        "desired_x_m": desired[0],
        "desired_y_m": desired[1],
        "desired_z_m": desired[2],
        "actual_x_m": actual[0],
        "actual_y_m": actual[1],
        "actual_z_m": actual[2],
        "actual_j1_rad": 0,
        "actual_j2_rad": 0,
        "actual_j3_rad": 0,
        "actual_j4_rad": 0,
        "actual_j5_rad": 0,
    }


class ExperimentMetricsTests(unittest.TestCase):
    def test_error_mm_is_distance_between_desired_and_actual(self):
        row = normalize_raw_sample(sample(0, [0.1, 0.0, 0.0], [0.0, 0.0, 0.0]))
        self.assertAlmostEqual(row["error_mm"], 100.0)

    def test_straight_path_efficiency_is_one(self):
        rows = [
            sample(0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            sample(100, [1.0, 0.0, 0.0], [0.5, 0.0, 0.0]),
            sample(200, [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ]
        summary = compute_trial_summary({"success": "yes"}, rows)
        self.assertAlmostEqual(summary["path_length_mm"], 1000.0)
        self.assertAlmostEqual(summary["path_efficiency"], 1.0)

    def test_detour_path_efficiency_is_less_than_one(self):
        rows = [
            sample(0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            sample(100, [1.0, 0.0, 0.0], [0.5, 0.5, 0.0]),
            sample(200, [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ]
        summary = compute_trial_summary({"success": "no"}, rows)
        self.assertLess(summary["path_efficiency"], 1.0)
        self.assertAlmostEqual(summary["path_length_mm"], math.sqrt(0.5) * 2000.0)


if __name__ == "__main__":
    unittest.main()
