import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UI = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")


class UiTeleopControlTests(unittest.TestCase):
    def test_main_operator_selector_exposes_teleop(self):
        selector = re.search(
            r'<div class="keyboard-mode-selector" id="inputDeviceMode".*?</div>',
            UI,
            re.DOTALL,
        )
        self.assertIsNotNone(selector)
        self.assertIn('name="inputDeviceMode" value="teleop"', selector.group(0))
        self.assertIn("主手遥操", selector.group(0))

    def test_teleop_uses_existing_arm_authority_api(self):
        self.assertIn('"teleop", "imitation"', UI)
        self.assertIn('callApi("POST /api/arm/control"', UI)


if __name__ == "__main__":
    unittest.main()
