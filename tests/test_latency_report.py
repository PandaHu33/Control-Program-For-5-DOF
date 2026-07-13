import csv
import tempfile
import unittest
from pathlib import Path

from control_ui.latency_report import merge_rows, percentile


def write_csv(path, fields, rows):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class LatencyReportTests(unittest.TestCase):
    def test_merge_and_percentile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bridge = root / "bridge.csv"
            h5 = root / "h5.csv"
            node = root / "node.csv"
            can = root / "can.log"
            write_csv(bridge, ["date", "ind", "source", "source_time_ms", "ws_rx_ns", "udp_tx_ns", "bridge_us"], [
                {"date": "x", "ind": 7, "source": "keyboard", "source_time_ms": 1000,
                 "ws_rx_ns": 1001000000, "udp_tx_ns": 1002000000, "bridge_us": 1000},
            ])
            write_csv(h5, ["ind", "source", "source_time_ms", "udp_rx_ns", "ros_publish_ns"], [
                {"ind": 7, "source": "keyboard", "source_time_ms": 1000,
                 "udp_rx_ns": 1003000000, "ros_publish_ns": 1004000000},
            ])
            write_csv(node, ["ind", "source", "source_stamp", "callback_stamp", "control_apply_stamp"], [
                {"ind": 7, "source": "keyboard", "source_stamp": 1.0,
                 "callback_stamp": 1.005, "control_apply_stamp": 1.010},
            ])
            can.write_text("(1.011000) can0 123#00\n", encoding="utf-8")
            total, rows = merge_rows(bridge, h5, node, can)
            self.assertEqual(len(total), 1)
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(rows[0]["source_to_can"], 11.0)
            self.assertEqual(percentile([1.0, 2.0, 3.0], 0.5), 2.0)


if __name__ == "__main__":
    unittest.main()
