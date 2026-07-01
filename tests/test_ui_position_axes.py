import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
UI = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
CAMERA_CONFIG = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")


class UiPositionAxesTests(unittest.TestCase):
    def test_keyboard_position_a_is_negative_y_and_d_is_positive_y(self):
        self.assertIn(
            'const dy = (pressed.has("KeyD") ? POSITION_STEP : 0) - (pressed.has("KeyA") ? POSITION_STEP : 0);',
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
