import csv
import json
import tempfile
import unittest
from pathlib import Path

from control_ui.episode_player import PLAYER_HTML, find_latest_episode, load_episode


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class EpisodePlayerTests(unittest.TestCase):
    def make_episode(self, root, name, started_ns):
        episode = root / "20260722" / name
        episode.mkdir(parents=True)
        (episode / "manifest.json").write_text(
            json.dumps(
                {
                    "session_id": name,
                    "started_at_ns": started_ns,
                    "duration_sec": 1.0,
                    "alignment_hz": 30,
                    "complete": True,
                }
            ),
            encoding="utf-8",
        )
        return episode

    def test_latest_episode_uses_manifest_timestamp(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.make_episode(root, "newer_name_but_older_time", 100)
            expected = self.make_episode(root, "older_name_but_newer_time", 200)
            self.assertEqual(find_latest_episode(root), expected)

    def test_loads_aligned_samples_monitor_and_frame_mapping(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            episode = self.make_episode(Path(temp_dir), "episode_001", 1_000_000_000)
            aligned_fields = [
                "session_time_ns", "arm_valid", "hand_valid", "arm_control_mode", "hand_control_mode",
                *[f"arm_actual_q_rad_{i}" for i in range(1, 6)],
                *[f"arm_target_q_rad_{i}" for i in range(1, 6)],
                *[f"arm_actual_current_ma_{i}" for i in range(1, 6)],
                *[f"hand_actual_position_units_{i}" for i in range(1, 7)],
                *[f"hand_target_position_units_{i}" for i in range(1, 7)],
                *[f"hand_filtered_current_ma_{i}" for i in range(1, 7)],
            ]
            row = {field: "1" for field in aligned_fields}
            row.update({"session_time_ns": "100000000", "arm_control_mode": "torque", "hand_control_mode": "glove"})
            write_csv(episode / "aligned_30hz.csv", aligned_fields, [row])
            write_csv(
                episode / "episode_records.csv",
                ["timestamp_ns", "state", "valid", "closure_score", "current_residual_json", "loaded_fingers_json", "reason", "invalid_reason", "prompt_event"],
                [{"timestamp_ns": "1100000000", "state": "FREE", "valid": "1", "closure_score": "0.2", "current_residual_json": "[1,2,3,4,5,6]", "loaded_fingers_json": "[]"}],
            )
            for side in ("left", "right"):
                write_csv(
                    episode / f"{side}_frames.csv",
                    ["capture_utc_ns", "encoded_pts_ns"],
                    [{"capture_utc_ns": "1100000000", "encoded_pts_ns": "0"}],
                )
                (episode / f"{side}.mp4").write_bytes(b"video")

            data = load_episode(episode)
            self.assertEqual(data["meta"]["sessionId"], "episode_001")
            self.assertEqual(len(data["samples"]), 1)
            self.assertEqual(data["samples"][0]["monitor"]["state"], "FREE")
            self.assertEqual(data["frames"]["left"], [[0.1, 0.0]])
            self.assertIn('id="timeline"', PLAYER_HTML)


if __name__ == "__main__":
    unittest.main()
