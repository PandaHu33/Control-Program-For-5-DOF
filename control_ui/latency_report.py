#!/usr/bin/env python3
"""Merge arm command latency traces and print segment percentiles.

The bridge, h5_udp_bridge and test_node CSV files share the command ind,
source and source timestamp.  candump should be captured with absolute
timestamps, for example: candump -t a can0 > can_trace.log
"""

import argparse
import bisect
import csv
import math
import re
from pathlib import Path


CAN_TIMESTAMP_RE = re.compile(r"\((\d+(?:\.\d+)?)\)")


def read_csv(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def command_key(ind, source, source_time_ms):
    return int(ind), str(source), int(round(float(source_time_ms)))


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_candump_timestamps(path):
    if not path:
        return []
    result = []
    with Path(path).open("r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            match = CAN_TIMESTAMP_RE.search(line)
            if match:
                result.append(float(match.group(1)))
    return sorted(result)


def first_can_after(timestamps, apply_stamp, timeout_sec=0.1):
    if not timestamps:
        return None
    index = bisect.bisect_left(timestamps, apply_stamp)
    if index >= len(timestamps) or timestamps[index] - apply_stamp > timeout_sec:
        return None
    return timestamps[index]


def merge_rows(bridge_path, h5_path, node_path, can_path=None):
    bridge_rows = read_csv(bridge_path)
    h5_rows = {
        command_key(row["ind"], row["source"], row["source_time_ms"]): row
        for row in read_csv(h5_path)
    }
    node_rows = {
        command_key(row["ind"], row["source"], float(row["source_stamp"]) * 1000.0): row
        for row in read_csv(node_path)
    }
    can_timestamps = read_candump_timestamps(can_path)
    merged = []
    for bridge in bridge_rows:
        key = command_key(bridge["ind"], bridge["source"], bridge["source_time_ms"])
        h5 = h5_rows.get(key)
        node = node_rows.get(key)
        if not h5 or not node:
            continue
        source = float(bridge["source_time_ms"]) / 1000.0
        ws_rx = float(bridge["ws_rx_ns"]) / 1e9
        udp_tx = float(bridge["udp_tx_ns"]) / 1e9
        udp_rx = float(h5["udp_rx_ns"]) / 1e9
        ros_pub = float(h5["ros_publish_ns"]) / 1e9
        callback = float(node["callback_stamp"])
        apply_stamp = float(node["control_apply_stamp"])
        can_stamp = first_can_after(can_timestamps, apply_stamp)
        row = {
            "ui_to_ws": (ws_rx - source) * 1000.0,
            "bridge": (udp_tx - ws_rx) * 1000.0,
            "network": (udp_rx - udp_tx) * 1000.0,
            "h5_to_ros": (ros_pub - udp_rx) * 1000.0,
            "ros_to_callback": (callback - ros_pub) * 1000.0,
            "callback_to_apply": (apply_stamp - callback) * 1000.0,
            "source_to_apply": (apply_stamp - source) * 1000.0,
        }
        if can_stamp is not None:
            row["apply_to_can"] = (can_stamp - apply_stamp) * 1000.0
            row["source_to_can"] = (can_stamp - source) * 1000.0
        merged.append(row)
    return bridge_rows, merged


def print_report(total_rows, rows, clock_uncertainty_ms):
    print("samples_source=%d samples_merged=%d loss_or_unmatched=%d" %
          (len(total_rows), len(rows), len(total_rows) - len(rows)))
    print("clock_uncertainty_ms=%.3f" % clock_uncertainty_ms)
    if clock_uncertainty_ms > 1.0:
        print("WARNING: cross-host one-way values exceed the 1 ms clock uncertainty requirement")
    metrics = [
        "ui_to_ws", "bridge", "network", "h5_to_ros", "ros_to_callback",
        "callback_to_apply", "apply_to_can", "source_to_apply", "source_to_can",
    ]
    print("metric_ms,count,p50,p95,p99,max")
    for name in metrics:
        values = [row[name] for row in rows if name in row and math.isfinite(row[name])]
        if not values:
            continue
        print("%s,%d,%.3f,%.3f,%.3f,%.3f" % (
            name, len(values), percentile(values, 0.50), percentile(values, 0.95),
            percentile(values, 0.99), max(values),
        ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", required=True, help="control_ui bridge latency CSV")
    parser.add_argument("--h5", required=True, help="Jetson h5_udp_bridge latency CSV")
    parser.add_argument("--node", required=True, help="Jetson test_node latency CSV")
    parser.add_argument("--candump", help="candump absolute timestamp log")
    parser.add_argument("--clock-uncertainty-ms", type=float, default=0.0)
    args = parser.parse_args()
    total, rows = merge_rows(args.bridge, args.h5, args.node, args.candump)
    print_report(total, rows, max(0.0, args.clock_uncertainty_ms))
    return 0 if rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
