import json
import unittest
from pathlib import Path
from unittest.mock import call, patch

from control_ui import bridge


ROOT = Path(__file__).resolve().parents[1]
UI = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
CONFIG = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")
HAND_RUNTIME = (ROOT / "wa100-sdk-publish" / "examples" / "udp_receiver_unity.cpp").read_text(encoding="utf-8")


class HandStartupModeTests(unittest.TestCase):
    def test_supervisor_and_ui_start_without_selected_hand_mode(self):
        self.assertEqual(bridge.DEFAULT_HAND_MODE, "idle")
        self.assertEqual(bridge.SYSTEM["hand_mode"], "idle")
        self.assertEqual(bridge.normalize_hand_mode("invalid"), "idle")
        self.assertIn('default_mode: "idle"', CONFIG)
        self.assertIn("start_glove_by_default: true", CONFIG)
        self.assertNotIn('name="handControlMode" value="glove" checked', UI)
        self.assertIn('let currentHandMode = "idle";', UI)

    def test_hand_runtime_rejects_inputs_until_a_mode_and_target_are_selected(self):
        self.assertIn("ControlMode::Idle", HAND_RUNTIME)
        self.assertIn("g_latest_actual_pos", HAND_RUNTIME)
        self.assertIn("control mode -> idle (forcing all encoders to 2000)", HAND_RUNTIME)
        self.assertIn("safe_open_targets[6] = {2000, 2000, 2000, 2000, 2000, 2000}", HAND_RUNTIME)
        self.assertIn("g_fast_driver.EnableAll(startup_feedback_position, startup_current)", HAND_RUNTIME)
        self.assertIn("memcpy(g_udp_target_pos, startup_open_target", HAND_RUNTIME)
        self.assertIn("memcpy(startup_actual, g_latest_actual_pos", HAND_RUNTIME)

    def test_idle_command_always_carries_six_safe_open_targets(self):
        with (
            patch.object(bridge, "send_udp_repeat", return_value=(True, "sent")) as send,
            patch.object(bridge, "refresh_hand_link_status"),
        ):
            self.assertEqual(bridge.send_hand_control("idle"), (True, "sent"))
        payload = json.loads(send.call_args.args[2])
        self.assertEqual(payload["mode"], "idle")
        self.assertEqual(payload["positions"], [2000] * 6)
        self.assertEqual(payload["name"], "idle_safe_open")

    def test_selecting_glove_mode_starts_hand_and_glove_before_switching(self):
        with (
            patch.object(bridge, "is_process_alive", return_value=False),
            patch.object(bridge, "start_program", return_value=(True, "ready")) as start,
            patch.object(bridge, "send_hand_control", return_value=(True, "sent")) as send,
        ):
            self.assertEqual(bridge.activate_hand_mode("glove"), (True, "sent"))
        self.assertEqual(start.call_args_list, [call("hand", "hand_exe"), call("glove", "unity_glove")])
        send.assert_called_once_with("glove", None, None)


if __name__ == "__main__":
    unittest.main()
