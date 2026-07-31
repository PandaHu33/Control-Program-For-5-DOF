import unittest

from control_ui.perception_assist_monitor import PerceptionAssistMonitor


CHANNELS = ["thumb_pitch", "thumb_yaw", "index", "middle", "ring", "pinky"]


def commissioned_config():
    baseline = {
        "valid": True,
        "type": "affine_q_dq_direction",
        "intercept_ma": 100.0,
        "position_coeff_ma_per_unit": 0.0,
        "speed_coeff_ma_per_unit_s": 0.0,
        "direction_offset_ma": {"opening": 0.0, "stationary": 0.0, "closing": 0.0},
    }
    return {
        "channel_names": CHANNELS,
        "finger_names": ["thumb", "thumb", "index", "middle", "ring", "pinky"],
        "closure_open_units": [2000.0] * 6,
        "closure_close_units": [0.0] * 6,
        "closure_weights": [1.0] * 6,
        "current_baselines": [dict(baseline) for _ in CHANNELS],
        "load_residual_on_ma": [120.0] * 6,
        "load_residual_off_ma": [80.0] * 6,
        "overload_residual_on_ma": [350.0] * 6,
        "overload_residual_off_ma": [250.0] * 6,
        "grasp_closure_enter": 0.60,
        "grasp_closure_exit": 0.52,
        "min_grasp_loaded_fingers": 2,
        "candidate_hold_sec": 0.10,
        "supported_hold_sec": 0.30,
        "supported_stability_window_sec": 0.40,
        "closure_stability_range": 0.05,
        "residual_stability_range_ma": 50.0,
        "overload_hold_sec": 0.04,
        "overload_release_sec": 0.10,
        "direction_deadband_units_s": 10.0,
        "max_velocity_dt_sec": 0.20,
        "stale_timeout_sec": 0.25,
    }


def sample(frame, time_ns, closure, residuals=None, feedback_valid=True,
           position_valid=None, current_valid=None):
    residuals = residuals or [0.0] * 6
    position = 2000.0 * (1.0 - closure)
    payload = {
        "type": "wa100_recording_state",
        "schema_version": 1,
        "seq": frame,
        "source_time_ns": time_ns,
        "feedback_valid": feedback_valid,
        "actual_position_units": [position] * 6,
        "filtered_current_ma": [100.0 + value for value in residuals],
    }
    if position_valid is not None:
        payload["position_valid"] = position_valid
    if current_valid is not None:
        payload["current_valid"] = current_valid
    return payload


class PerceptionAssistMonitorTest(unittest.TestCase):
    def setUp(self):
        self.base_utc_ns = 2_000_000_000_000_000_000
        self.base_mono_ns = 50_000_000_000
        self.monitor = PerceptionAssistMonitor(commissioned_config())
        # First valid-position sample intentionally cannot produce a current
        # residual because dq/direction history does not yet exist.
        warm = sample(0, self.base_utc_ns, 0.0)
        decision = self.monitor.update(warm, self.base_utc_ns, self.base_mono_ns)
        self.assertEqual(decision["valid"], 0)
        self.assertIn("insufficient_velocity_history", decision["invalid_reason"])

    def update(self, frame, seconds, closure, residuals=None, utc_age_sec=0.0):
        receive_utc = self.base_utc_ns + int(seconds * 1e9)
        source_utc = receive_utc - int(utc_age_sec * 1e9)
        return self.monitor.update(
            sample(frame, source_utc, closure, residuals),
            receive_utc,
            self.base_mono_ns + int(seconds * 1e9),
        )

    def test_free_open_or_closed_without_load(self):
        opened = self.update(1, 0.05, 0.10)
        self.assertEqual((opened["state"], opened["valid"], opened["loaded_fingers"]), ("FREE", 1, []))
        closed = self.update(2, 0.10, 0.80)
        self.assertEqual(closed["state"], "FREE")
        self.assertTrue(closed["ablation"]["closure_only"])
        self.assertFalse(closed["ablation"]["joint"])

    def test_early_contact_and_current_only_false_positive_control(self):
        decision = self.update(1, 0.05, 0.30, [0, 0, 160, 150, 0, 0])
        self.assertEqual(decision["state"], "EARLY_CONTACT")
        self.assertEqual(decision["prompt_event"], "EARLY_CONTACT")
        self.assertTrue(decision["ablation"]["current_only"])
        self.assertFalse(decision["ablation"]["joint"])
        self.assertIn("index", decision["loaded_fingers"])

    def test_grasp_candidate_requires_joint_evidence_dwell(self):
        loads = [0, 0, 150, 145, 0, 0]
        pending = self.update(1, 0.05, 0.70, loads)
        candidate = self.update(2, 0.16, 0.70, loads)
        self.assertEqual(pending["state"], "FREE")
        self.assertEqual(candidate["state"], "GRASP_CANDIDATE")
        self.assertEqual(candidate["prompt_event"], "GRASP_CANDIDATE")
        self.assertFalse(candidate["grasp_success_confirmed"])

    def test_two_thumb_actuators_count_as_one_loaded_finger(self):
        thumb_only = [150, 145, 0, 0, 0, 0]
        self.update(1, 0.05, 0.70, thumb_only)
        decision = self.update(2, 0.16, 0.70, thumb_only)
        self.assertEqual(decision["loaded_fingers"], ["thumb"])
        self.assertEqual(decision["state"], "FREE")
        self.assertFalse(decision["ablation"]["current_only"])

    def test_stable_multifinger_load_becomes_supported_not_success(self):
        loads = [0, 0, 150, 145, 0, 0]
        decision = None
        events = []
        for frame, seconds in enumerate((0.05, 0.16, 0.26, 0.36, 0.46), 1):
            decision = self.update(frame, seconds, 0.70, loads)
            events.extend(decision["events"])
        self.assertEqual(decision["state"], "GRASP_SUPPORTED")
        self.assertIn("GRASP_SUPPORTED", events)
        self.assertFalse(decision["grasp_success_confirmed"])
        self.assertIn("stable_multichannel", decision["reason"])

    def test_overload_uses_residual_and_hysteretic_release(self):
        overload = [380, 0, 0, 0, 0, 0]
        self.update(1, 0.05, 0.40, overload)
        decision = self.update(2, 0.10, 0.40, overload)
        self.assertEqual(decision["state"], "OVERLOAD")
        self.assertEqual(decision["prompt_event"], "OVERLOAD")
        held = self.update(3, 0.15, 0.40, [270, 0, 0, 0, 0, 0])
        self.assertEqual(held["state"], "OVERLOAD")
        releasing = self.update(4, 0.20, 0.40, [200, 0, 0, 0, 0, 0])
        released = self.update(5, 0.31, 0.40, [0, 0, 0, 0, 0, 0])
        self.assertEqual(releasing["state"], "OVERLOAD")
        self.assertNotEqual(released["state"], "OVERLOAD")

    def test_stale_data_fails_closed(self):
        decision = self.update(1, 0.50, 0.70, [150, 150, 0, 0, 0, 0], utc_age_sec=0.30)
        self.assertEqual(decision["valid"], 0)
        self.assertEqual(decision["state"], "FREE")
        self.assertIn("data_stale", decision["invalid_reason"])

    def test_transient_invalid_position_uses_fallback_without_losing_current(self):
        payload = sample(
            1,
            self.base_utc_ns + 50_000_000,
            0.0,
            [0, 0, 160, 150, 0, 0],
            position_valid=[False] * 6,
            current_valid=[True] * 6,
        )
        decision = self.monitor.update(
            payload,
            self.base_utc_ns + 50_000_000,
            self.base_mono_ns + 50_000_000,
        )
        self.assertEqual(decision["valid"], 1)
        self.assertTrue(decision["signal_validity"]["current"])
        self.assertFalse(decision["signal_validity"]["position_current_frame"])
        self.assertTrue(decision["signal_validity"]["position_effective"])
        self.assertEqual(len(decision["signal_validity"]["position_fallback_channels"]), 6)
        self.assertTrue(all(value is not None for value in decision["current_residual"]))
        self.assertIn("position_fallback:index", decision["degraded_reason"])

    def test_bottom_layer_zero_wrap_correction_stays_position_valid(self):
        payload = sample(
            1,
            self.base_utc_ns + 50_000_000,
            0.0,
            position_valid=[True] * 6,
            current_valid=[True] * 6,
        )
        payload["position_zero_wrap_corrected"] = [True, False, False, False, False, False]
        decision = self.monitor.update(
            payload,
            self.base_utc_ns + 50_000_000,
            self.base_mono_ns + 50_000_000,
        )
        self.assertEqual(decision["valid"], 1)
        self.assertTrue(decision["signal_validity"]["position_current_frame"])
        self.assertEqual(decision["signal_validity"]["position_zero_wrap_corrected_channels"], ["thumb_pitch"])
        self.assertIn("position_zero_wrap_corrected:thumb_pitch", decision["degraded_reason"])

    def test_persistent_invalid_position_keeps_current_visible_but_disables_joint_inference(self):
        payload = sample(
            2,
            self.base_utc_ns + 400_000_000,
            0.0,
            [0, 0, 160, 150, 0, 0],
            position_valid=[False] * 6,
            current_valid=[True] * 6,
        )
        decision = self.monitor.update(
            payload,
            self.base_utc_ns + 400_000_000,
            self.base_mono_ns + 400_000_000,
        )
        self.assertEqual(decision["valid"], 0)
        self.assertTrue(decision["signal_validity"]["current"])
        self.assertFalse(decision["signal_validity"]["position_effective"])
        self.assertEqual(decision["filtered_current_ma"], payload["filtered_current_ma"])
        self.assertIn("actual_position_channel_invalid:index", decision["invalid_reason"])

    def test_missing_baseline_never_fabricates_residual(self):
        config = commissioned_config()
        config["current_baselines"][2] = None
        monitor = PerceptionAssistMonitor(config)
        monitor.update(sample(0, self.base_utc_ns, 0.0), self.base_utc_ns, self.base_mono_ns)
        decision = monitor.update(
            sample(1, self.base_utc_ns + 50_000_000, 0.70, [150] * 6),
            self.base_utc_ns + 50_000_000,
            self.base_mono_ns + 50_000_000,
        )
        self.assertEqual(decision["valid"], 0)
        self.assertEqual(decision["state"], "FREE")
        self.assertIsNone(decision["current_residual"][2])
        self.assertIn("missing_current_baseline:index", decision["invalid_reason"])

    def test_baseline_speed_clip_and_floor_are_applied(self):
        config = commissioned_config()
        model = config["current_baselines"][0]
        model.update({
            "intercept_ma": -10.0,
            "speed_coeff_ma_per_unit_s": 1.0,
            "speed_clip_units_s": 5.0,
            "baseline_floor_ma": 0.0,
        })
        monitor = PerceptionAssistMonitor(config)
        baseline, reason = monitor._baseline(0, 0.0, 100.0, "stationary")
        self.assertIsNone(reason)
        self.assertEqual(baseline, 0.0)


if __name__ == "__main__":
    unittest.main()
