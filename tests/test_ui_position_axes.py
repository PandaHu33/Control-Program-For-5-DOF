import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
UI = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
CAMERA_CONFIG = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")


class UiPositionAxesTests(unittest.TestCase):
    def test_keyboard_position_axes_keep_operator_facing_left_right(self):
        self.assertIn(
            'const dx = (pressed.has("KeyS") ? step : 0) - (pressed.has("KeyW") ? step : 0);',
            UI,
        )
        self.assertIn(
            'return (pressed.has("KeyD") ? 1 : 0) - (pressed.has("KeyA") ? 1 : 0);',
            UI,
        )
        self.assertIn('const dy = keyboardLateralIntent() * step;', UI)
        self.assertIn(
            'const dz = (pressed.has("KeyE") ? step : 0) - (pressed.has("KeyQ") ? step : 0);',
            UI,
        )

    def test_keyboard_and_gamepad_share_normalized_lateral_intent(self):
        self.assertIn('function gamepadLateralIntent(gp)', UI)
        self.assertIn('return -applyGamepadDeadzone(gp?.axes?.[0] || 0);', UI)
        self.assertIn(
            'applyPositionDelta(axisY * step, lateralIntent * step, -zAxis * step);',
            UI,
        )
        self.assertIn('targetAngles[0] += keyboardLateralIntent() * step;', UI)
        self.assertIn('targetAngles[0] += lateralIntent * step; // A/D', UI)
        self.assertNotIn(
            'const dy = (pressed.has("KeyA") ? step : 0) - (pressed.has("KeyD") ? step : 0);',
            UI,
        )

    def test_vr_delta_left_right_is_not_inverted_again(self):
        self.assertIn('y: clampNumber(Number(axis.y) || 0, -1, 1)', UI)
        self.assertNotIn('y: clampNumber(-(Number(axis.y) || 0), -1, 1)', UI)

    def test_vr_absolute_left_right_matches_position_axis(self):
        mapping = """const baseYaw = splitRemapClamped(
        wristLeftRight,
        neutralLeftRight,
        -HAND_VISION_BASE_YAW_LIMIT,
        0,
        HAND_VISION_BASE_YAW_LIMIT
      );"""
        self.assertIn(mapping, UI)

    def test_vr_camera_uses_the_enabled_hls_path(self):
        self.assertIn('const DEFAULT_VIDEO_URL = "http://localhost:8081/usb_camera/index.m3u8";', UI)
        self.assertIn("hls_port: 8081", CAMERA_CONFIG)
        self.assertIn('id="monitorHlsPlayer"', UI)


if __name__ == "__main__":
    unittest.main()
