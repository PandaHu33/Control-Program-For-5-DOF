import importlib.util
import os
import socket
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UI = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
CONFIG = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")
START_SCRIPT = (ROOT / "control_ui" / "one_click_start.ps1").read_text(encoding="utf-8")


class _ImportSocket:
    def setsockopt(self, *_args):
        pass

    def ioctl(self, *_args):
        pass

    def bind(self, *_args):
        pass


def load_bridge_without_network():
    original_socket = socket.socket
    original_mode = os.environ.get("UEM_CAMERA_ACTIVE_MODE")
    socket.socket = lambda *_args, **_kwargs: _ImportSocket()
    os.environ["UEM_CAMERA_ACTIVE_MODE"] = "display"
    try:
        spec = importlib.util.spec_from_file_location(
            "camera_startup_bridge_test",
            ROOT / "control_ui" / "bridge.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        socket.socket = original_socket
        if original_mode is None:
            os.environ.pop("UEM_CAMERA_ACTIVE_MODE", None)
        else:
            os.environ["UEM_CAMERA_ACTIVE_MODE"] = original_mode


class CameraStartupModeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge_without_network()

    def test_default_mode_is_browser_display(self):
        self.assertIn('startup_mode: "display"', CONFIG)
        self.assertEqual(self.bridge.normalize_camera_startup_mode("invalid"), "display")
        self.assertEqual(self.bridge.normalize_camera_startup_mode("STREAM"), "stream")

    def test_override_is_written_atomically_and_invalid_json_falls_back(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "camera_startup_mode.json"
            self.assertIsNone(self.bridge.read_camera_startup_override(path))
            self.assertEqual(self.bridge.save_camera_startup_override("stream", path), "stream")
            self.assertEqual(self.bridge.read_camera_startup_override(path), "stream")
            self.assertEqual(list(path.parent.glob("*.tmp")), [])
            path.write_text("not-json", encoding="utf-8")
            self.assertIsNone(self.bridge.read_camera_startup_override(path))
            with self.assertRaises(ValueError):
                self.bridge.save_camera_startup_override("other", path)

    def test_api_saves_next_mode_without_changing_active_mode(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            original_path = self.bridge.CAMERA_STARTUP_SETTINGS_PATH
            original_active = self.bridge.CAMERA_ACTIVE_MODE
            try:
                self.bridge.CAMERA_STARTUP_SETTINGS_PATH = Path(temp_dir) / "camera_startup_mode.json"
                self.bridge.CAMERA_ACTIVE_MODE = "display"
                result = self.bridge.do_command(
                    "POST", "/api/camera/startup-mode", {"mode": "stream"}
                )
                self.assertTrue(result["ok"])
                self.assertEqual(result["active_mode"], "display")
                self.assertEqual(result["next_mode"], "stream")
                self.assertTrue(result["restart_required"])
                invalid = self.bridge.do_command(
                    "POST", "/api/camera/startup-mode", {"mode": "invalid"}
                )
                self.assertFalse(invalid["ok"])
                self.assertEqual(invalid["next_mode"], "stream")
            finally:
                self.bridge.CAMERA_STARTUP_SETTINGS_PATH = original_path
                self.bridge.CAMERA_ACTIVE_MODE = original_active

    def test_start_script_always_starts_unified_camera_service(self):
        self.assertIn('$env:UEM_CAMERA_ACTIVE_MODE = $cameraStartupMode', START_SCRIPT)
        self.assertNotIn('if ($cameraStartupMode -eq "stream")', START_SCRIPT)
        self.assertIn('$rtspCameraReady = Start-RtspCameraStream', START_SCRIPT)
        self.assertIn('Stop-RtspCameraStream', START_SCRIPT)

    def test_ui_has_dual_stack_and_next_start_mode_setting(self):
        self.assertIn('id="leftCameraPreview"', UI)
        self.assertIn('id="rightCameraPreview"', UI)
        self.assertIn('id="cameraStartupModeGroup"', UI)
        self.assertIn('POST /api/camera/startup-mode', UI)
        self.assertIn('[cameraStreamPreview, cameraStreamHlsPlayer]', UI)
        self.assertNotIn('\n    applyVideoSource(DEFAULT_VIDEO_URL);', UI)

    def test_display_uses_service_without_competing_for_usb_cameras(self):
        self.assertNotIn("getUserMedia", UI)
        self.assertIn("left_preview_url", UI)
        self.assertIn("right_preview_url", UI)
        self.assertIn("left_mirror: true", CONFIG)
        self.assertIn("right_mirror: true", CONFIG)


if __name__ == "__main__":
    unittest.main()
