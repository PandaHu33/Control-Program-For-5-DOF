import unittest

from control_ui import bridge


def envelope(source, value, signal_form="continuous"):
    return {
        "type": "raw_master_input",
        "schema_version": 1,
        "source": source,
        "signal_form": signal_form,
        "value": value,
    }


class RawInputValidationTests(unittest.TestCase):
    def set_modes(self, arm, hand):
        with bridge.STATE_LOCK:
            bridge.SYSTEM["arm_control"]["mode"] = arm
            bridge.SYSTEM["hand_mode"] = hand

    def test_keyboard_and_gamepad_keep_native_browser_values(self):
        self.set_modes("keyboard", "preset")
        valid, _ = bridge.validate_raw_master_input(envelope("keyboard", {
            "code": "KeyW", "key": "w", "event": "down", "repeat": False,
            "pressed_keys": ["KeyW"], "modifiers": {}, "client_time_ms": 1,
        }, "discrete"), browser_only=True)
        self.assertTrue(valid)

        self.set_modes("gamepad", "preset")
        valid, _ = bridge.validate_raw_master_input(envelope("gamepad", {
            "id": "pad", "index": 0, "timestamp": 10, "connected": True,
            "mapping": "standard", "axes": [0.25, -0.5],
            "buttons": [{"value": 0.7, "pressed": True, "touched": True}],
            "client_time_ms": 2,
        }), browser_only=True)
        self.assertTrue(valid)

    def test_pico_wrist_requires_raw_position_and_rotation(self):
        self.set_modes("hand_vision", "preset")
        payload = envelope("pico_hand", {
            "stream": "wrist_pose", "seq": 7, "device_time": 1.2,
            "hand": "right", "tracked": True,
            "position": [0.1, 0.2, 0.3], "rotation": [0.0, 0.0, 0.0, 1.0],
            "deadman_active": True,
        })
        self.assertTrue(bridge.validate_raw_master_input(payload)[0])
        payload["value"]["rotation"] = [0.0, 1.0]
        self.assertFalse(bridge.validate_raw_master_input(payload)[0])

    def test_same_native_arrays_are_classified_by_active_hand_mode(self):
        native = {
            "stream": "hand_skeleton", "frameId": 9, "rightTracked": True,
            "rightPositions": [0.1] * 63, "rightRotations": [0.2] * 63,
        }
        self.set_modes("idle", "vr")
        self.assertTrue(bridge.validate_raw_master_input(envelope("pico_hand", native))[0])
        self.assertFalse(bridge.validate_raw_master_input(envelope("data_glove", native))[0])

        self.set_modes("idle", "glove")
        self.assertTrue(bridge.validate_raw_master_input(envelope("data_glove", native))[0])
        self.assertFalse(bridge.validate_raw_master_input(envelope("pico_hand", native))[0])


if __name__ == "__main__":
    unittest.main()
