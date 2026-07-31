import json
import unittest
from pathlib import Path

from control_ui.episode_player import CHANNEL_NAMES


ROOT = Path(__file__).resolve().parents[1]
HARDWARE_CHANNELS = ["thumb_pitch", "thumb_yaw", "index", "middle", "ring", "pinky"]


class HandChannelLayoutTests(unittest.TestCase):
    def test_analysis_and_replay_use_physical_hardware_order(self):
        config = json.loads(
            (ROOT / "control_ui" / "perception_assist_config.json").read_text(encoding="utf-8")
        )
        self.assertEqual(config["channel_names"], HARDWARE_CHANNELS)
        self.assertEqual(CHANNEL_NAMES, HARDWARE_CHANNELS)

    def test_operator_ui_labels_id1_and_id2_by_physical_motion(self):
        html = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
        self.assertIn("ID1 拇指弯曲", html)
        self.assertIn("ID2 拇指横摇", html)


if __name__ == "__main__":
    unittest.main()
