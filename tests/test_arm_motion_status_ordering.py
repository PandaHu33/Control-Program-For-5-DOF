import binascii
import struct
import unittest

from control_ui import bridge
from control_ui.bridge import arm_motion_transition_is_regressive


def motion_frame(ind, state, source="preset", progress=0.0, duration=1.0):
    frame = bytearray(293)
    struct.pack_into("<I", frame, 0, ind)
    struct.pack_into("<7f", frame, 12, *([0.0] * 7))
    struct.pack_into("<2f", frame, 145, progress, duration)
    note = f"motion:{state}:{source}".encode("utf-8")
    frame[225:225 + len(note)] = note
    struct.pack_into("<I", frame, 289, binascii.crc32(frame[:289]) & 0xFFFFFFFF)
    return bytes(frame)


class ArmMotionStatusOrderingTests(unittest.TestCase):
    def setUp(self):
        self.original_state = bridge.SYSTEM["state"]
        self.original_control = dict(bridge.SYSTEM["arm_control"])
        self.original_control["motion"] = dict(bridge.SYSTEM["arm_control"].get("motion") or {})

    def tearDown(self):
        with bridge.STATE_LOCK:
            bridge.SYSTEM["state"] = self.original_state
            bridge.SYSTEM["arm_control"] = self.original_control

    def test_terminal_state_cannot_be_relocked_by_delayed_udp_packet(self):
        for terminal in ("complete", "failed", "preempted"):
            for delayed in ("accepted", "running"):
                with self.subTest(terminal=terminal, delayed=delayed):
                    self.assertTrue(arm_motion_transition_is_regressive(terminal, delayed))

    def test_running_cannot_fall_back_to_accepted(self):
        self.assertTrue(arm_motion_transition_is_regressive("running", "accepted"))

    def test_forward_and_new_terminal_transitions_remain_valid(self):
        for previous, incoming in (
            ("idle", "accepted"),
            ("accepted", "running"),
            ("accepted", "complete"),
            ("running", "complete"),
            ("complete", "complete"),
        ):
            with self.subTest(previous=previous, incoming=incoming):
                self.assertFalse(arm_motion_transition_is_regressive(previous, incoming))

    def test_complete_releases_system_authority_and_delayed_accepted_is_ignored(self):
        with bridge.STATE_LOCK:
            bridge.SYSTEM["state"] = "MOVING"
            bridge.SYSTEM["arm_control"]["mode"] = "preset"
            bridge.SYSTEM["arm_control"]["owner_id"] = None
            bridge.SYSTEM["arm_control"]["motion"] = {
                "ind": 42,
                "source": "preset",
                "state": "running",
                "progress": 0.8,
                "duration_sec": 1.0,
            }

        self.assertTrue(bridge.update_arm_telemetry(motion_frame(42, "complete", progress=1.0)))
        with bridge.STATE_LOCK:
            self.assertEqual(bridge.SYSTEM["arm_control"]["mode"], "idle")
            self.assertEqual(bridge.SYSTEM["arm_control"]["motion"]["state"], "complete")
            self.assertEqual(bridge.SYSTEM["state"], "READY")

        self.assertTrue(bridge.update_arm_telemetry(motion_frame(42, "accepted")))
        with bridge.STATE_LOCK:
            self.assertEqual(bridge.SYSTEM["arm_control"]["mode"], "idle")
            self.assertEqual(bridge.SYSTEM["arm_control"]["motion"]["state"], "complete")
            self.assertEqual(bridge.SYSTEM["state"], "READY")


if __name__ == "__main__":
    unittest.main()
