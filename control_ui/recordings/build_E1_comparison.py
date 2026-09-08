#!/usr/bin/env python3
"""E1 comparison figures and repeated-trial statistics.

Fig_E1_raw_inputs            — (a) arm-side raw input magnitude, (b) hand-side
                               raw input magnitude, four conditions overlaid.
Fig_E1_canonical_wrist_3d    — 2x2 per-condition canonical wrist trajectory
                               with start / grasp / release events.
Fig_E1_grasp_hand_skeleton   — 1x4 21-node hand skeleton at the grasp moment.
Fig_E1_rmse                  — (a) arm joint RMSE, (b) hand channel RMSE.

Data loading, time alignment and record parsing follow
build_recording_visualizations.py.  All outputs are saved as png/pdf/svg.

With --data-root, the script scans repeated keyboard/gamepad/VR-controller
trials that share the data-glove hand source, selects one robust group-centre
trial per arm source, and writes a 3-D overlay plus CSV/JSON statistics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Conditions (mapped from manifest method strings)
# ---------------------------------------------------------------------------
CONDITION_NAMES = {
    "keyboard+preset": "Keyboard + Preset",
    "keyboard+glove": "Keyboard + Glove",
    "gamepad+glove": "Gamepad + Glove",
    "controller_delta+glove": "VR Controller + Glove",
    "hand_vision+vr": "Pure VR Hand Tracking",
}
CONDITION_ORDER = [
    "Keyboard + Glove",
    "Gamepad + Glove",
    "VR Controller + Glove",
    "Pure VR Hand Tracking",
]
CONDITION_COLORS = {
    "Keyboard + Preset": "#7c3aed",
    "Keyboard + Glove": "#1f77b4",
    "Gamepad + Glove": "#ff7f0e",
    "VR Controller + Glove": "#2ca02c",
    "Pure VR Hand Tracking": "#d62728",
}

# Hand skeleton topology / colors (matches plot_canonical_wrist_hand_animation.py)
FINGERS = {
    "thumb": ("#D55E00", (1, 2, 3, 4)),
    "index": ("#0072B2", (5, 6, 7, 8)),
    "middle": ("#009E73", (9, 10, 11, 12)),
    "ring": ("#CC79A7", (13, 14, 15, 16)),
    "little": ("#E69F00", (17, 18, 19, 20)),
}
PALM_INDICES = (0, 5, 9, 13, 17, 0, 1)
PALM_COLOR = "#64748B"
WRIST_COLOR = "#111827"
FINGER_BEND_NODES = {"thumb": 3, "index": 6, "middle": 10, "ring": 14, "pinky": 18}
FINGER_TIP_NODES = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
GAMEPAD_DEADZONE = 0.08
GRASP_WINDOW_S = 3.0

REPEATED_CONDITIONS = {
    "keyboard+glove": "Keyboard + Glove",
    "gamepad+glove": "Gamepad + Glove",
    "controller_delta+glove": "VR Controller + Glove",
}
REPEATED_CONDITION_LABELS_ZH = {
    "Keyboard + Preset": "键盘 + 预编程",
    "Keyboard + Glove": "键盘 + 数据手套",
    "Gamepad + Glove": "手柄 + 数据手套",
    "VR Controller + Glove": "VR手柄 + 数据手套",
}
SMOOTHNESS_HZ = 50.0
SMOOTHNESS_WINDOW_S = 0.4
ARM_LINK_3_M = 0.424
ARM_LINK_5_M = 0.424
RUCKIG_MAX_JOINT_VELOCITY_RAD_S = 1.0
REPRESENTATIVE_METRICS = (
    "duration_s",
    "wrist_path_length_m",
    "wrist_log10_dimensionless_jerk",
    "overall_arm_rmse_rad",
)


def configure_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "DejaVu Sans"],
        "font.size": 8.5,
        "axes.labelsize": 9,
        "axes.titlesize": 9.5,
        "legend.fontsize": 7.6,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.linewidth": 0.8,
        "lines.linewidth": 1.4,
        "figure.dpi": 140,
        "savefig.dpi": 300,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def save_figure(fig, stem: Path) -> None:
    for suffix in ("png", "pdf", "svg"):
        fig.savefig(stem.with_suffix(f".{suffix}"), bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Data loading (kept consistent with build_recording_visualizations.py)
# ---------------------------------------------------------------------------

def truthy(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def numeric_vector(value, size: int) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) < size:
        return None
    try:
        values = [float(item) for item in value[:size]]
    except (TypeError, ValueError):
        return None
    return values if all(math.isfinite(v) for v in values) else None


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def load_raw_input(path: Path, start_ns: int, end_s: float) -> dict[str, list[dict]]:
    sources: dict[str, list[dict]] = defaultdict(list)
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            source = str(row.get("source", "unknown"))
            stream_name = str((row.get("value") or {}).get("stream") or "")
            key = f"{source}:{stream_name}" if stream_name else source
            t = (int(row.get("timestamp_ns", 0)) - start_ns) / 1e9
            if t < 0 or t > end_s:
                continue
            row["t_s"] = t
            sources[key].append(row)
    for key in sources:
        sources[key].sort(key=lambda row: int(row.get("timestamp_ns", 0)))
    return dict(sources)


def load_canonical(path: Path, start_ns: int, end_s: float) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                goal = json.loads(line)
            except json.JSONDecodeError:
                continue
            if goal.get("invalid_reasons"):
                continue
            wrist_pose = goal.get("wrist_pose_C") or {}
            position = numeric_vector(wrist_pose.get("position_m"), 3)
            quaternion = numeric_vector(wrist_pose.get("orientation_xyzw"), 4)
            # Schema v4 recordings use hand_skeleton_C; older E1 recordings
            # used hand_skeleton_w for the same wrist-local 21-node geometry.
            skeleton_raw = goal.get("hand_skeleton_C") or goal.get("hand_skeleton_w") or []
            skeleton = [numeric_vector(node, 3) for node in skeleton_raw]
            if not position or not quaternion or len(skeleton) != 21 or any(node is None for node in skeleton):
                continue
            timestamp = goal.get("record_receive_utc_ns") or goal.get("receive_utc_ns") or goal.get("source_time_ns")
            t = (int(timestamp) - start_ns) / 1e9
            if t < 0 or t > end_s:
                continue
            rows.append({
                "t": t,
                "position": np.asarray(position, dtype=float),
                "quaternion": np.asarray(quaternion, dtype=float),
                "skeleton": np.asarray(skeleton, dtype=float),
            })
    rows.sort(key=lambda row: row["t"])
    return rows


def rotate_xyzw(point, quaternion) -> np.ndarray:
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not norm > 1e-12:
        return np.asarray(point, dtype=float)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = (float(v) for v in point)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return np.asarray([
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ], dtype=float)


def world_hand_nodes(skeleton: np.ndarray, quaternion: np.ndarray, position: np.ndarray) -> np.ndarray:
    world = np.empty_like(skeleton)
    for index, local in enumerate(skeleton):
        world[index] = rotate_xyzw(local, quaternion) + position
    return world


# ---------------------------------------------------------------------------
# Native raw-signal extractors (Fig. E1-1)
# ---------------------------------------------------------------------------

KEYBOARD_LANES = {
    "W/S": ("KeyW", "KeyS"),
    "A/D": ("KeyA", "KeyD"),
    "Q/E": ("KeyQ", "KeyE"),
    "J4": ("ArrowLeft", "ArrowRight"),
}


def keyboard_event_intervals(rows: list[dict], duration: float) -> dict[str, list[tuple[float, float]]]:
    """Key press intervals per lane (native discrete events, no interpolation)."""
    starts: dict[str, float] = {}
    per_key: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for row in rows:
        value = row.get("value") or {}
        code = str(value.get("code") or "")
        event = str(value.get("event") or "")
        if not code:
            continue
        t = float(row["t_s"])
        if event == "down":
            starts.setdefault(code, t)
        elif code in starts:
            per_key[code].append((starts.pop(code), t))
    for code, t0 in starts.items():
        per_key[code].append((t0, duration))
    lanes: dict[str, list[tuple[float, float]]] = {}
    for lane, codes in KEYBOARD_LANES.items():
        intervals: list[tuple[float, float]] = []
        for code in codes:
            intervals.extend(per_key.get(code, []))
        intervals.sort()
        lanes[lane] = intervals
    return lanes


def gamepad_intent_series(rows: list[dict]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Continuous gamepad intent in [-1, 1]: lateral / depth / z / J4.
    Rules follow build_recording_visualizations.py (LX+ = right, LY- = forward)."""
    t = np.asarray([row["t_s"] for row in rows], dtype=float)
    lateral = np.zeros_like(t)
    depth = np.zeros_like(t)
    z_intent = np.zeros_like(t)
    j4_intent = np.zeros_like(t)
    for i, row in enumerate(rows):
        value = row.get("value") or {}
        axes = numeric_vector(value.get("axes"), 4) or [0.0, 0.0, 0.0, 0.0]
        ax = axes[0] if abs(axes[0]) >= GAMEPAD_DEADZONE else 0.0
        ay = axes[1] if abs(axes[1]) >= GAMEPAD_DEADZONE else 0.0
        buttons = value.get("buttons") or []
        btn = lambda index: float((buttons[index].get("value", 0)) if index < len(buttons) else 0.0)
        lateral[i] = ax
        depth[i] = -ay
        z_intent[i] = btn(6) - btn(4)
        j4_intent[i] = btn(15) - btn(14)
    return t, lateral, depth, z_intent, j4_intent


def wrist_pose_series(rows: list[dict]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Native wrist pose: time, position (N,3) in m and roll angle in deg."""
    t: list[float] = []
    positions: list[np.ndarray] = []
    rolls: list[float] = []
    for row in rows:
        value = row.get("value") or {}
        position = numeric_vector(value.get("position_m") or value.get("position"), 3)
        rotation = numeric_vector(value.get("rotation_xyzw") or value.get("rotation"), 4)
        if position is None:
            continue
        t.append(float(row["t_s"]))
        positions.append(np.asarray(position, dtype=float))
        if rotation is not None:
            x, y, z, w = (float(v) for v in rotation)
            rolls.append(math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))))
        else:
            rolls.append(math.nan)
    return (np.asarray(t, dtype=float),
            np.asarray(positions, dtype=float) if positions else np.empty((0, 3)),
            np.asarray(rolls, dtype=float))


def thumb_index_distance(skeleton: np.ndarray) -> float:
    return float(np.linalg.norm(skeleton[4] - skeleton[8]))


def find_grasp_event(rows: list[dict], window_s: float = GRASP_WINDOW_S) -> int:
    positions = np.asarray([row["position"] for row in rows], dtype=float)
    lowest_index = int(np.argmin(positions[:, 2]))
    t_lowest = rows[lowest_index]["t"]
    candidates = [i for i, row in enumerate(rows) if abs(row["t"] - t_lowest) <= window_s]
    if not candidates:
        candidates = list(range(len(rows)))
    return min(candidates, key=lambda i: thumb_index_distance(rows[i]["skeleton"]))


def find_release_event(rows: list[dict], grasp_index: int) -> int:
    return max(range(grasp_index, len(rows)), key=lambda i: thumb_index_distance(rows[i]["skeleton"]))


def fingertip_distances(skeleton: np.ndarray) -> list[float]:
    wrist = skeleton[0]
    return [float(np.linalg.norm(skeleton[node] - wrist)) for node in FINGER_TIP_NODES.values()]


def compute_arm_rmse(aligned_rows: list[dict[str, str]]) -> tuple[np.ndarray, float]:
    errors = [
        np.asarray([float(row.get(f"arm_actual_q_rad_{i}") or "nan") - float(row.get(f"arm_target_q_rad_{i}") or "nan")
                    for i in range(1, 5)], dtype=float)
        for row in aligned_rows
        if truthy(row.get("arm_valid", ""))
        and all(row.get(f"arm_actual_q_rad_{i}") for i in range(1, 5))
        and all(row.get(f"arm_target_q_rad_{i}") for i in range(1, 5))
    ]
    if not errors:
        return np.full(4, math.nan), math.nan
    matrix = np.stack(errors, axis=0)
    per_joint = np.sqrt(np.mean(matrix * matrix, axis=0))
    return per_joint, float(np.sqrt(np.mean(matrix * matrix)))


def compute_hand_rmse(aligned_rows: list[dict[str, str]]) -> tuple[np.ndarray, float]:
    errors = []
    for row in aligned_rows:
        if not truthy(row.get("hand_valid", "")):
            continue
        target = np.asarray([float(row.get(f"hand_target_position_units_{i}") or "nan") for i in range(1, 7)], dtype=float)
        actual = np.asarray([float(row.get(f"hand_actual_position_units_{i}") or "nan") for i in range(1, 7)], dtype=float)
        if not (np.all(np.isfinite(target)) and np.all(np.isfinite(actual))):
            continue
        # 命令与反馈同序（ID1=thumb pitch、ID2=thumb yaw），无需交换。
        # 旧版录音（2026-08-21 驱动响应槽位修复前）的 hand_actual 拇指两列
        # 是反的，旧数据不适用本脚本，需重新录制/生成。
        errors.append(actual - target)
    if not errors:
        return np.full(6, math.nan), math.nan
    matrix = np.stack(errors, axis=0)
    per_channel = np.sqrt(np.mean(matrix * matrix, axis=0))
    return per_channel, float(np.sqrt(np.mean(matrix * matrix)))


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def _mark_events(axis, data: dict, duration: float) -> None:
    """Grasp / release vertical lines for this condition only (no time
    realignment — events are drawn at their trial-relative times)."""
    _, grasp_i, release_i = data["event_indices"]
    t_grasp = data["rows"][grasp_i]["t"]
    t_release = data["rows"][release_i]["t"]
    axis.axvline(t_grasp, color="#166534", linewidth=1.0, linestyle="-", alpha=0.9)
    axis.axvline(t_release, color="#B45309", linewidth=1.0, linestyle="--", alpha=0.9)
    axis.set_xlim(0.0, duration)


def build_fig_raw_inputs(datasets: list[dict], output_stem: Path) -> None:
    """4x1 native arm-side signal panels (one row per condition).  Each
    condition keeps its own raw form and units; no cross-modality
    normalization, no shared y scale; each panel marks only its own
    grasp / release."""
    duration = max(d["duration_s"] for d in datasets)
    by_condition = {data["condition"]: data for data in datasets}
    fig = plt.figure(figsize=(9.8, 11.6), constrained_layout=True)
    grid = fig.add_gridspec(4, 1)

    # (a) Keyboard + Glove — key press intervals (event raster)
    ax_a = fig.add_subplot(grid[0, 0])
    data = by_condition["Keyboard + Glove"]
    lanes = keyboard_event_intervals(data["raw_rows"].get("keyboard", []), duration)
    for lane_index, (lane, intervals) in enumerate(lanes.items()):
        for begin, end in intervals:
            ax_a.broken_barh([(begin, max(end - begin, 0.008))], (lane_index - 0.34, 0.68),
                             facecolors=CONDITION_COLORS[data["condition"]], alpha=0.85)
    ax_a.set_yticks(range(len(lanes)), list(lanes.keys()))
    ax_a.set_ylim(-0.7, len(lanes) - 0.3)
    ax_a.set_ylabel("key press (event)")
    ax_a.set_title("(a) Keyboard + Glove", loc="left")
    _mark_events(ax_a, data, duration)

    # (b) Gamepad + Glove — continuous intent channels in [-1, 1]
    ax_b = fig.add_subplot(grid[1, 0])
    data = by_condition["Gamepad + Glove"]
    t, lateral, depth, z_intent, j4_intent = gamepad_intent_series(data["raw_rows"].get("gamepad", []))
    color = CONDITION_COLORS[data["condition"]]
    for series, label, linestyle, lw, series_color in (
        (lateral, "lateral", "-", 1.4, color),
        (depth, "depth", "--", 1.2, color),
        (z_intent, "z", "-", 1.0, "0.45"),
        (j4_intent, "J4", "--", 1.0, "0.45"),
    ):
        ax_b.plot(t, series, linewidth=lw, linestyle=linestyle, color=series_color, label=label)
    ax_b.axhline(0.0, color="0.35", linewidth=0.6)
    ax_b.set_ylim(-1.05, 1.05)
    ax_b.set_ylabel("normalized command (-1..1)")
    ax_b.set_title("(b) Gamepad + Glove", loc="left")
    ax_b.legend(ncol=4, frameon=False, loc="upper right", fontsize=6.6)
    _mark_events(ax_b, data, duration)

    # (c) VR Controller + Glove — native wrist position (+ roll, dashed)
    ax_c = fig.add_subplot(grid[2, 0])
    data = by_condition["VR Controller + Glove"]
    t, positions, rolls = wrist_pose_series(data["raw_rows"].get("vr_controller:controller_pose", []))
    if positions.size:
        for index, label in enumerate(("x", "y", "z")):
            ax_c.plot(t, positions[:, index], linewidth=1.2, label=label)
        if np.any(np.isfinite(rolls)):
            ax_c_roll = ax_c.twinx()
            ax_c_roll.plot(t, rolls, linewidth=1.0, linestyle="--", color="0.45", label="wrist roll")
            ax_c_roll.set_ylabel("roll (deg)", fontsize=8)
            ax_c_roll.spines[["top"]].set_visible(False)
    ax_c.set_ylabel("wrist position (m)")
    ax_c.set_title("(c) VR Controller + Glove", loc="left")
    ax_c.legend(ncol=4, frameon=False, loc="upper left", fontsize=6.6)
    _mark_events(ax_c, data, duration)

    # (d) Pure VR Hand Tracking — native wrist position (+ roll, dashed)
    ax_d = fig.add_subplot(grid[3, 0])
    data = by_condition["Pure VR Hand Tracking"]
    t, positions, rolls = wrist_pose_series(data["raw_rows"].get("pico_hand:wrist_pose", []))
    if positions.size:
        for index, label in enumerate(("x", "y", "z")):
            ax_d.plot(t, positions[:, index], linewidth=1.2, label=label)
        if np.any(np.isfinite(rolls)):
            ax_d_roll = ax_d.twinx()
            ax_d_roll.plot(t, rolls, linewidth=1.0, linestyle="--", color="0.45", label="wrist roll")
            ax_d_roll.set_ylabel("roll (deg)", fontsize=8)
            ax_d_roll.spines[["top"]].set_visible(False)
    ax_d.set_ylabel("wrist position (m)")
    ax_d.set_title("(d) Pure VR Hand Tracking", loc="left")
    ax_d.legend(ncol=4, frameon=False, loc="upper left", fontsize=6.6)
    _mark_events(ax_d, data, duration)

    for axis in (ax_a, ax_b, ax_c, ax_d):
        axis.grid(True, linewidth=0.45, alpha=0.25)
        axis.spines[["top"]].set_visible(False)
        axis.set_xlabel("Trial-relative time (s)")
    fig.suptitle("Fig. E1-1  Native master signals before semantic adaptation", fontsize=12)
    fig.text(
        0.5, 0.005,
        "Signals are shown in their native source-specific forms and are not amplitude-comparable across modalities.  "
        "Green solid / orange dashed lines mark this condition's grasp / release.",
        ha="center", fontsize=8, color="0.3",
    )
    save_figure(fig, output_stem)


def build_fig_canonical_wrist_3d(datasets: list[dict], output_stem: Path) -> None:
    all_points = np.concatenate([
        np.asarray([row["position"] for row in data["rows"]], dtype=float)
        for data in datasets if data["rows"]
    ], axis=0)
    lo = all_points.min(axis=0)
    hi = all_points.max(axis=0)
    span = hi - lo
    span[span <= 1e-9] = 1.0
    lo -= 0.05 * span
    hi += 0.05 * span

    fig = plt.figure(figsize=(11.2, 9.4), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    for index, data in enumerate(datasets):
        axis = fig.add_subplot(grid[index // 2, index % 2], projection="3d")
        positions = np.asarray([row["position"] for row in data["rows"]], dtype=float)
        color = CONDITION_COLORS[data["condition"]]
        axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color=color, linewidth=1.6)
        start_i, grasp_i, release_i = data["event_indices"]
        for frame_index, marker, size, zorder in (
            (start_i, "o", 46, 6),
            (grasp_i, "*", 130, 8),
            (release_i, "^", 60, 7),
        ):
            axis.scatter(*positions[frame_index], color=color, marker=marker, s=size,
                         edgecolor="white", linewidth=0.5, zorder=zorder)
        axis.set_xlim(lo[0], hi[0])
        axis.set_ylim(lo[1], hi[1])
        axis.set_zlim(lo[2], hi[2])
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_zlabel("z (m)")
        axis.set_title(
            f"({chr(ord('a') + index)}) {data['condition']}\n"
            f"start / grasp / release", loc="left", fontsize=8.8,
        )
        axis.view_init(elev=24, azim=-58)
        axis.set_box_aspect((1.0, 0.9, 0.75))
    fig.suptitle("Fig. E1-2  Canonical wrist trajectories (common view & scale)", fontsize=12)
    save_figure(fig, output_stem)


def draw_skeleton(axis, skeleton: np.ndarray) -> None:
    palm = skeleton[list(PALM_INDICES)]
    axis.plot(palm[:, 0], palm[:, 1], palm[:, 2], color=PALM_COLOR, linewidth=2.1, zorder=18)
    for name, (color, indices) in FINGERS.items():
        points = skeleton[list(indices)]
        axis.plot(points[:, 0], points[:, 1], points[:, 2], color=color, linewidth=2.4, zorder=20)
        axis.scatter(points[:, 0], points[:, 1], points[:, 2],
                     facecolor=color, edgecolor="white", linewidth=0.75,
                     s=26 if name == "thumb" else 20, depthshade=False, zorder=31)
    axis.scatter(*skeleton[0], facecolor=WRIST_COLOR, edgecolor="white", linewidth=0.75,
                 marker="s", s=30, depthshade=False, zorder=30)


def build_fig_grasp_hand_skeleton(datasets: list[dict], output_stem: Path) -> None:
    skeletons = [data["rows"][data["event_indices"][1]]["skeleton"] for data in datasets]
    all_points = np.concatenate(skeletons, axis=0)
    lo = all_points.min(axis=0)
    hi = all_points.max(axis=0)
    span = hi - lo
    span[span <= 1e-9] = 1.0
    lo -= 0.06 * span
    hi += 0.06 * span

    fig = plt.figure(figsize=(13.2, 3.9), constrained_layout=True)
    for index, data in enumerate(datasets):
        axis = fig.add_subplot(1, 4, index + 1, projection="3d")
        row = data["rows"][data["event_indices"][1]]
        draw_skeleton(axis, row["skeleton"])
        mean_tip = float(np.mean(fingertip_distances(row["skeleton"])))
        axis.set_xlim(lo[0], hi[0])
        axis.set_ylim(lo[1], hi[1])
        axis.set_zlim(lo[2], hi[2])
        axis.set_title(
            f"{data['condition']}\nt = {row['t']:.1f} s, mean fingertip distance = {mean_tip * 1000:.0f} mm",
            fontsize=8.2, loc="left",
        )
        axis.view_init(elev=24, azim=-58)
        axis.set_box_aspect((1.0, 1.0, 0.9))
        axis.set_axis_off()
    fig.suptitle("Fig. E1-3  Hand skeleton at the grasp moment", fontsize=12)
    save_figure(fig, output_stem)


def build_fig_rmse(datasets: list[dict], output_stem: Path) -> None:
    fig, (ax_arm, ax_hand) = plt.subplots(1, 2, figsize=(11.6, 4.6), constrained_layout=True)
    arm_joints = np.arange(4)
    hand_channels = np.arange(6)
    width = 0.2
    overall_arm, overall_hand = [], []
    for index, data in enumerate(datasets):
        color = CONDITION_COLORS[data["condition"]]
        offset = (index - 1.5) * width
        per_joint, total_arm = data["arm_rmse"]
        per_channel, total_hand = data["hand_rmse"]
        overall_arm.append(total_arm)
        overall_hand.append(total_hand)
        ax_arm.bar(arm_joints + offset, per_joint, width, color=color, label=data["condition"])
        ax_hand.bar(hand_channels + offset, per_channel, width, color=color, label=data["condition"])
    for axis, title, ylabel, ticks, tick_labels in (
        (ax_arm, "(a) Arm joint RMSE", "RMSE (rad)", arm_joints, ["J1", "J2", "J3", "J4"]),
        (ax_hand, "(b) Hand channel RMSE", "RMSE (encoder unit)", hand_channels, ["H1", "H2", "H3", "H4", "H5", "H6"]),
    ):
        axis.set_xticks(ticks, tick_labels)
        axis.set_ylabel(ylabel)
        axis.set_title(title, loc="left")
        axis.grid(True, linewidth=0.45, alpha=0.3, axis="y")
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(ncol=2, frameon=False, loc="upper right", fontsize=7.2)
    mean_arm = float(np.nanmean(overall_arm)) if overall_arm else math.nan
    mean_hand = float(np.nanmean(overall_hand)) if overall_hand else math.nan
    fig.suptitle(
        f"Fig. E1-4  Tracking RMSE  ·  Overall arm RMSE {mean_arm:.4f} rad  ·  Overall hand RMSE {mean_hand:.1f} unit",
        fontsize=12,
    )
    save_figure(fig, output_stem)


# ---------------------------------------------------------------------------
# Repeated-trial E1 analysis (three arm sources, common data-glove hand source)
# ---------------------------------------------------------------------------

def _finite_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _savgol_smooth(values: np.ndarray, sample_hz: float, window_s: float) -> np.ndarray:
    """Savitzky-Golay smoothing without a SciPy runtime dependency."""
    if len(values) < 5:
        return values.copy()
    requested = max(5, int(round(sample_hz * window_s)))
    if requested % 2 == 0:
        requested += 1
    window = min(requested, len(values) if len(values) % 2 else len(values) - 1)
    if window < 5:
        return values.copy()
    half = window // 2
    x = np.arange(-half, half + 1, dtype=float)
    design = np.vander(x, 4, increasing=True)
    coefficients = np.linalg.pinv(design)[0]
    smoothed = np.empty_like(values, dtype=float)
    for axis in range(values.shape[1]):
        padded = np.pad(values[:, axis], (half, half), mode="reflect")
        smoothed[:, axis] = np.convolve(padded, coefficients[::-1], mode="valid")
    return smoothed


def compute_wrist_motion_metrics(rows: list[dict]) -> dict[str, float]:
    times = np.asarray([float(row["t"]) for row in rows], dtype=float)
    positions = np.asarray([row["position"] for row in rows], dtype=float)
    unique_times, indices = np.unique(times, return_index=True)
    positions = positions[indices]
    if len(unique_times) < 5 or unique_times[-1] <= unique_times[0]:
        raise ValueError("insufficient unique canonical wrist samples")
    dt = 1.0 / SMOOTHNESS_HZ
    uniform_t = np.arange(unique_times[0], unique_times[-1] + 0.5 * dt, dt)
    uniform_position = np.column_stack([
        np.interp(uniform_t, unique_times, positions[:, axis]) for axis in range(3)
    ])
    smooth_position = _savgol_smooth(uniform_position, SMOOTHNESS_HZ, SMOOTHNESS_WINDOW_S)
    steps = np.linalg.norm(np.diff(smooth_position, axis=0), axis=1)
    path_length = float(np.sum(steps))
    duration = float(uniform_t[-1] - uniform_t[0])
    if duration <= 0.0 or path_length <= 1e-9:
        raise ValueError("canonical wrist trajectory has negligible duration or path length")
    velocity = np.gradient(smooth_position, dt, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, dt, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, dt, axis=0, edge_order=2)
    integrated_squared_jerk = float(np.trapezoid(np.sum(jerk * jerk, axis=1), dx=dt))
    dimensionless_jerk = (duration ** 5 / path_length ** 2) * integrated_squared_jerk
    if not math.isfinite(dimensionless_jerk) or dimensionless_jerk <= 0.0:
        raise ValueError("dimensionless jerk is not finite and positive")
    return {
        "wrist_path_length_m": path_length,
        "wrist_mean_speed_m_s": path_length / duration,
        "wrist_dimensionless_jerk": dimensionless_jerk,
        "wrist_log10_dimensionless_jerk": math.log10(dimensionless_jerk),
        "smoothness_resampled_duration_s": duration,
        "smoothness_uniform_samples": int(len(uniform_t)),
    }


def _canonical_line_count(path: Path) -> int:
    with path.open("r", encoding="utf-8") as stream:
        return sum(1 for line in stream if line.strip())


def load_repeated_trial(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if "fail" in path.name.lower():
        return None, "directory_marked_fail"
    required = ("manifest.json", "canonical_goal.jsonl", "aligned_20hz.csv")
    missing = [name for name in required if not (path / name).exists()]
    if missing:
        return None, "missing:" + ",".join(missing)
    try:
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"invalid_manifest:{exc}"
    method = str(manifest.get("method", "unknown"))
    condition = REPEATED_CONDITIONS.get(method) or ("Keyboard + Preset" if method == "keyboard+preset" else None)
    if condition is None:
        return None, f"unsupported_method:{method}"
    if not bool(manifest.get("complete")) or str(manifest.get("state", "")) != "complete":
        return None, "manifest_incomplete"
    if manifest.get("errors"):
        return None, "manifest_errors"
    duration_s = _finite_number(manifest.get("duration_sec"))
    start_ns = int(manifest.get("started_at_ns", 0) or 0)
    if duration_s is None or duration_s <= 0.0 or start_ns <= 0:
        return None, "invalid_manifest_timing"
    try:
        canonical_path = path / "canonical_goal.jsonl"
        total_canonical = _canonical_line_count(canonical_path)
        canonical_rows = load_canonical(canonical_path, start_ns, duration_s * 1.02)
        aligned_rows = read_csv(path / "aligned_20hz.csv")
        if len(canonical_rows) < 5 or not aligned_rows:
            return None, "insufficient_data"
        motion = compute_wrist_motion_metrics(canonical_rows)
        arm_rmse = compute_arm_rmse(aligned_rows)[1]
        hand_rmse = compute_hand_rmse(aligned_rows)[1]
    except (OSError, ValueError, csv.Error) as exc:
        return None, f"analysis_error:{exc}"
    if not all(math.isfinite(value) for value in (arm_rmse, hand_rmse)):
        return None, "nonfinite_tracking_rmse"
    session_id = str(manifest.get("session_id") or path.name)
    record: dict[str, Any] = {
        "condition": condition,
        "condition_zh": REPEATED_CONDITION_LABELS_ZH[condition],
        "method": method,
        "session_id": session_id,
        "folder_name": path.name,
        "session_path": str(path.resolve()),
        "trial": manifest.get("trial"),
        "included": True,
        "exclusion_reason": "",
        "duration_s": duration_s,
        "canonical_valid_samples": len(canonical_rows),
        "canonical_total_samples": total_canonical,
        "canonical_valid_fraction": len(canonical_rows) / total_canonical if total_canonical else math.nan,
        "overall_arm_rmse_rad": float(arm_rmse),
        "overall_hand_rmse_unit": float(hand_rmse),
        "representative_score": math.nan,
        "is_representative": False,
        "canonical_rows": canonical_rows,
    }
    record.update(motion)
    return record, None


def select_representatives(records: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    selected: dict[str, dict[str, Any]] = {}
    for condition in REPEATED_CONDITIONS.values():
        group = [row for row in records if row["condition"] == condition]
        if not group:
            raise ValueError(f"no valid trials for {condition}")
        matrix = np.asarray([[float(row[key]) for key in REPRESENTATIVE_METRICS] for row in group])
        medians = np.median(matrix, axis=0)
        mad = np.median(np.abs(matrix - medians), axis=0)
        scale = 1.4826 * mad
        active = scale > 1e-12
        for row, values in zip(group, matrix):
            row["representative_score"] = float(np.sum(np.abs(values[active] - medians[active]) / scale[active]))
        group.sort(key=lambda row: (
            row["representative_score"],
            abs(float(row["duration_s"]) - float(medians[0])),
            row["session_id"],
        ))
        group[0]["is_representative"] = True
        selected[condition] = group[0]
    return selected


def _mean_sd(values: list[float]) -> tuple[float, float]:
    array = np.asarray(values, dtype=float)
    return float(np.mean(array)), float(np.std(array, ddof=1)) if len(array) > 1 else 0.0


def build_repeated_group_summary(
    records: list[dict[str, Any]], all_trials: list[dict[str, Any]], selected: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for condition in REPEATED_CONDITIONS.values():
        group = [row for row in records if row["condition"] == condition]
        scanned = [row for row in all_trials if row.get("condition") == condition]
        summary: dict[str, Any] = {
            "condition": condition,
            "condition_zh": REPEATED_CONDITION_LABELS_ZH[condition],
            "scanned_n": len(scanned),
            "valid_n": len(group),
            "failed_n": sum(row.get("exclusion_reason") == "directory_marked_fail" for row in scanned),
            "representative_session_id": selected[condition]["session_id"],
        }
        for key in (
            "duration_s", "wrist_path_length_m", "overall_arm_rmse_rad", "overall_hand_rmse_unit",
            "endpoint_position_rmse_mm", "endpoint_orientation_rmse_deg",
            "canonical_cartesian_speed_max_m_s",
            "canonical_cartesian_speed_p99_m_s", "canonical_joint_speed_max_rad_s",
            "canonical_joint_speed_p99_rad_s", "canonical_intervals_over_ruckig_velocity_fraction",
            "equivalent_045_joint_speed_median_rad_s", "equivalent_045_joint_speed_p95_rad_s",
            "equivalent_045_over_ruckig_velocity_fraction",
        ):
            mean, sd = _mean_sd([float(row[key]) for row in group])
            summary[f"{key}_mean"] = mean
            summary[f"{key}_sd"] = sd
        smooth = np.asarray([float(row["wrist_dimensionless_jerk"]) for row in group])
        summary["wrist_dimensionless_jerk_median"] = float(np.median(smooth))
        summary["wrist_dimensionless_jerk_q1"] = float(np.percentile(smooth, 25))
        summary["wrist_dimensionless_jerk_q3"] = float(np.percentile(smooth, 75))
        summary["canonical_valid_fraction_mean"] = float(np.mean([
            row["canonical_valid_fraction"] for row in group
        ]))
        summaries.append(summary)
    return summaries


def write_csv_records(path: Path, rows: list[dict[str, Any]], columns: list[str]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in columns})


def build_fig_representative_wrist_overlay(selected: dict[str, dict[str, Any]], output_stem: Path) -> None:
    fig = plt.figure(figsize=(8.8, 7.2), constrained_layout=True)
    axis = fig.add_subplot(111, projection="3d")
    all_positions: list[np.ndarray] = []
    for condition in REPEATED_CONDITIONS.values():
        data = selected[condition]
        positions = np.asarray([row["position"] for row in data["canonical_rows"]], dtype=float)
        all_positions.append(positions)
        color = CONDITION_COLORS[condition]
        label = (
            f"{REPEATED_CONDITION_LABELS_ZH[condition]} | {data['session_id']} | "
            f"{data['duration_s']:.1f} s | log10 jerk {data['wrist_log10_dimensionless_jerk']:.2f}"
        )
        axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color=color, linewidth=1.7, label=label)
        axis.scatter(*positions[0], color=color, marker="o", s=42, edgecolor="white", linewidth=0.7)
        axis.scatter(*positions[-1], color=color, marker="X", s=56, edgecolor="white", linewidth=0.7)
    points = np.concatenate(all_positions, axis=0)
    lo, hi = points.min(axis=0), points.max(axis=0)
    span = hi - lo
    span[span <= 1e-9] = 1.0
    center = (lo + hi) / 2.0
    half = float(np.max(span)) * 0.55
    axis.set_xlim(center[0] - half, center[0] + half)
    axis.set_ylim(center[1] - half, center[1] + half)
    axis.set_zlim(center[2] - half, center[2] + half)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.set_title("E1 三类臂源主端的典型 canonical 腕部期望轨迹", fontsize=12)
    axis.view_init(elev=24, azim=-58)
    axis.set_box_aspect((1.0, 1.0, 1.0))
    axis.grid(True, linewidth=0.45, alpha=0.3)
    axis.legend(loc="upper left", bbox_to_anchor=(0.0, 0.98), fontsize=7.2, frameon=True)
    fig.text(0.5, 0.012, "圆点：起点；叉号：终点。三条轨迹使用相同坐标范围与视角。", ha="center", fontsize=8)
    save_figure(fig, output_stem)


def representative_event_indices(rows: list[dict[str, Any]]) -> tuple[int, int]:
    """User-defined E1 event proxies: lowest z = grasp, leftmost y = release."""
    positions = np.asarray([row["position"] for row in rows], dtype=float)
    return int(np.argmin(positions[:, 2])), int(np.argmin(positions[:, 1]))


def build_fig1_arm_source_raw_signals(selected: dict[str, dict[str, Any]], output_stem: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 9.2), constrained_layout=True)
    for row_index, condition in enumerate(REPEATED_CONDITIONS.values()):
        data = selected[condition]
        session_path = Path(data["session_path"])
        manifest = json.loads((session_path / "manifest.json").read_text(encoding="utf-8"))
        raw = load_raw_input(
            session_path / "raw_input.jsonl",
            int(manifest["started_at_ns"]),
            float(manifest["duration_sec"]) * 1.02,
        )
        axis = axes[row_index]
        color = CONDITION_COLORS[condition]
        title = f"({chr(ord('a') + row_index)}) {REPEATED_CONDITION_LABELS_ZH[condition]} · {data['session_id']}"
        if condition == "Keyboard + Glove":
            lanes = keyboard_event_intervals(raw.get("keyboard", []), float(data["duration_s"]))
            for lane_index, (lane, intervals) in enumerate(lanes.items()):
                for begin, end in intervals:
                    axis.broken_barh([(begin, max(end - begin, 0.008))], (lane_index - 0.34, 0.68),
                                     facecolors=color, alpha=0.88)
            axis.set_yticks(range(len(lanes)), list(lanes.keys()))
            axis.set_ylim(-0.7, len(lanes) - 0.3)
            axis.set_ylabel("按键事件")
        elif condition == "Gamepad + Glove":
            t, lateral, depth, z_intent, j4_intent = gamepad_intent_series(raw.get("gamepad", []))
            for values, label, style in (
                (lateral, "横向", "-"), (depth, "纵向", "--"),
                (z_intent, "升降", "-"), (j4_intent, "腕转", "--"),
            ):
                axis.plot(t, values, linestyle=style, linewidth=1.15, label=label)
            axis.axhline(0.0, color="0.4", linewidth=0.6)
            axis.set_ylim(-1.05, 1.05)
            axis.set_ylabel("归一化指令")
            axis.legend(ncol=4, frameon=False, loc="upper right")
        else:
            t, positions, rolls = wrist_pose_series(raw.get("vr_controller:controller_pose", []))
            for column, label in enumerate(("x", "y", "z")):
                axis.plot(t, positions[:, column], linewidth=1.15, label=label)
            axis.set_ylabel("手柄位置 (m)")
            axis.legend(ncol=3, frameon=False, loc="upper left")
            if np.any(np.isfinite(rolls)):
                roll_axis = axis.twinx()
                roll_axis.plot(t, rolls, color="0.35", linestyle=":", linewidth=1.0, label="roll")
                roll_axis.set_ylabel("roll (°)", color="0.35")
                roll_axis.tick_params(axis="y", colors="0.35")
                roll_axis.spines[["top"]].set_visible(False)
        axis.set_title(title, loc="left")
        axis.set_xlim(0.0, float(data["duration_s"]))
        axis.set_xlabel("试次相对时间 (s)")
        axis.grid(True, linewidth=0.45, alpha=0.28)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Fig. 1  三类臂源主端的原始输入信号", fontsize=13)
    fig.text(0.5, 0.005, "三种输入保留各自原始信号形式与量纲，幅值不可直接横向比较。", ha="center", fontsize=8)
    fig.get_layout_engine().set(rect=(0.0, 0.035, 1.0, 0.945))
    save_figure(fig, output_stem)


def build_fig2_representative_wrist_rows(
    selected: dict[str, dict[str, Any]], output_stems: list[Path]
) -> dict[str, dict[str, float]]:
    all_positions = np.concatenate([
        np.asarray([row["position"] for row in selected[condition]["canonical_rows"]], dtype=float)
        for condition in REPEATED_CONDITIONS.values()
    ], axis=0)
    lo, hi = all_positions.min(axis=0), all_positions.max(axis=0)
    span = hi - lo
    span[span <= 1e-9] = 1.0
    lo -= 0.05 * span
    hi += 0.05 * span
    event_summary: dict[str, dict[str, float]] = {}
    fig = plt.figure(figsize=(8.6, 15.2), constrained_layout=True)
    for row_index, condition in enumerate(REPEATED_CONDITIONS.values()):
        data = selected[condition]
        rows = data["canonical_rows"]
        positions = np.asarray([row["position"] for row in rows], dtype=float)
        grasp_index, release_index = representative_event_indices(rows)
        color = CONDITION_COLORS[condition]
        axis = fig.add_subplot(3, 1, row_index + 1, projection="3d")
        axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color=color, linewidth=1.65)
        axis.scatter(*positions[0], color="#111827", marker="o", s=34, label="起点", zorder=8)
        axis.scatter(*positions[grasp_index], color="#166534", marker="*", s=130,
                     edgecolor="white", linewidth=0.6, label="抓取：z 最低", zorder=10)
        axis.scatter(*positions[release_index], color="#B45309", marker="^", s=66,
                     edgecolor="white", linewidth=0.6, label="释放：y 最小", zorder=9)
        axis.set_xlim(lo[0], hi[0])
        axis.set_ylim(lo[1], hi[1])
        axis.set_zlim(lo[2], hi[2])
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_zlabel("z (m)")
        axis.set_title(
            f"({chr(ord('a') + row_index)}) {REPEATED_CONDITION_LABELS_ZH[condition]} · {data['session_id']}\n"
            f"抓取 t={rows[grasp_index]['t']:.2f} s；释放 t={rows[release_index]['t']:.2f} s",
            loc="left", fontsize=9.4,
        )
        axis.view_init(elev=24, azim=-58)
        axis.set_box_aspect((1.0, 0.9, 0.75))
        axis.legend(loc="upper left", fontsize=7.5, frameon=True)
        event_summary[condition] = {
            "grasp_time_s": float(rows[grasp_index]["t"]),
            "grasp_x_m": float(positions[grasp_index, 0]),
            "grasp_y_m": float(positions[grasp_index, 1]),
            "grasp_z_m": float(positions[grasp_index, 2]),
            "release_time_s": float(rows[release_index]["t"]),
            "release_x_m": float(positions[release_index, 0]),
            "release_y_m": float(positions[release_index, 1]),
            "release_z_m": float(positions[release_index, 2]),
        }
    fig.suptitle("Fig. 2  三类臂源主端的典型 canonical 腕部期望轨迹", fontsize=13)
    fig.text(0.5, 0.004, "绿色星号：全局 z 最低处（抓取时刻）；橙色三角：全局 y 最小处（释放时刻）。三行采用相同坐标范围与视角。",
             ha="center", fontsize=8)
    fig.get_layout_engine().set(rect=(0.0, 0.018, 1.0, 0.985))
    for stem in output_stems:
        for suffix in ("png", "pdf", "svg"):
            fig.savefig(stem.with_suffix(f".{suffix}"), bbox_inches="tight")
    plt.close(fig)
    return event_summary


def arm_fk_pose_4dof(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    q1, q2, q3, q4 = (float(q[index]) for index in range(4))
    radial = ARM_LINK_3_M * math.cos(q2) + ARM_LINK_5_M * math.cos(q2 - q3)
    position = np.asarray([
        radial * math.cos(q1), radial * math.sin(q1),
        ARM_LINK_3_M * math.sin(q2) + ARM_LINK_5_M * math.sin(q2 - q3),
    ])
    quaternion = np.asarray([math.sin(q4 * 0.5), 0.0, 0.0, math.cos(q4 * 0.5)])
    return position, quaternion


def arm_position_jacobian_3dof(q: np.ndarray) -> np.ndarray:
    q1, q2, q3 = (float(q[index]) for index in range(3))
    radial = ARM_LINK_3_M * math.cos(q2) + ARM_LINK_5_M * math.cos(q2 - q3)
    height = ARM_LINK_3_M * math.sin(q2) + ARM_LINK_5_M * math.sin(q2 - q3)
    distal_sin = ARM_LINK_5_M * math.sin(q2 - q3)
    distal_cos = ARM_LINK_5_M * math.cos(q2 - q3)
    return np.asarray([
        [-radial * math.sin(q1), -height * math.cos(q1), distal_sin * math.cos(q1)],
        [ radial * math.cos(q1), -height * math.sin(q1), distal_sin * math.sin(q1)],
        [0.0, radial, -distal_cos],
    ])


def load_endpoint_tracking_errors(session_path: Path) -> dict[str, np.ndarray]:
    rows = read_csv(session_path / "aligned_20hz.csv")
    times: list[float] = []
    position_errors_mm: list[float] = []
    orientation_errors_deg: list[float] = []
    for row in rows:
        if not truthy(row.get("arm_valid", "")):
            continue
        try:
            actual_q = np.asarray([float(row[f"arm_actual_q_rad_{index}"]) for index in range(1, 5)])
            target_q = np.asarray([float(row[f"arm_target_q_rad_{index}"]) for index in range(1, 5)])
            time_s = float(row["session_time_ns"]) / 1e9
        except (KeyError, TypeError, ValueError):
            continue
        if not (np.all(np.isfinite(actual_q)) and np.all(np.isfinite(target_q)) and math.isfinite(time_s)):
            continue
        actual_position, actual_quaternion = arm_fk_pose_4dof(actual_q)
        target_position, target_quaternion = arm_fk_pose_4dof(target_q)
        dot = float(np.clip(abs(np.dot(actual_quaternion, target_quaternion)), 0.0, 1.0))
        times.append(time_s)
        position_errors_mm.append(float(np.linalg.norm(actual_position - target_position) * 1000.0))
        orientation_errors_deg.append(math.degrees(2.0 * math.acos(dot)))
    if len(times) < 2:
        raise ValueError(f"insufficient arm tracking data in {session_path}")
    return {
        "time_s": np.asarray(times),
        "position_error_mm": np.asarray(position_errors_mm),
        "orientation_error_deg": np.asarray(orientation_errors_deg),
    }


def audit_canonical_speed_against_ruckig(session_path: Path) -> dict[str, Any]:
    """Compare 50 Hz canonical IK increments with the downstream joint limit."""
    manifest = json.loads((session_path / "manifest.json").read_text(encoding="utf-8"))
    start_ns = int(manifest["started_at_ns"])
    samples: list[tuple[float, np.ndarray, np.ndarray]] = []
    with (session_path / "canonical_goal.jsonl").open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                goal = json.loads(line)
                target = goal.get("final_targets") or goal.get("slave_kinematic_reference_cmd") or {}
                q = numeric_vector(target.get("arm_target_q_rad"), 4)
                position = numeric_vector((goal.get("wrist_pose_C") or {}).get("position_m"), 3)
                timestamp_ns = int(goal.get("record_receive_utc_ns") or goal.get("receive_utc_ns") or 0)
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
            if q is not None and position is not None and timestamp_ns > 0:
                samples.append(((timestamp_ns - start_ns) / 1e9, np.asarray(q), np.asarray(position)))
    if len(samples) < 5:
        raise ValueError(f"insufficient canonical IK data in {session_path}")
    samples.sort(key=lambda item: item[0])
    times = np.asarray([item[0] for item in samples])
    q = np.unwrap(np.asarray([item[1] for item in samples]), axis=0)
    position = np.asarray([item[2] for item in samples])
    dt = np.diff(times)
    valid = (dt >= 0.005) & (dt <= 0.100)
    if np.count_nonzero(valid) < 4:
        raise ValueError(f"insufficient valid canonical intervals in {session_path}")
    joint_speed = np.abs(np.diff(q, axis=0)[valid] / dt[valid, None])
    cartesian_speed = np.linalg.norm(np.diff(position, axis=0)[valid] / dt[valid, None], axis=1)
    any_joint_speed = np.max(joint_speed, axis=1)
    high_cartesian = cartesian_speed >= 0.40
    moving = cartesian_speed >= 0.05
    equivalent_joint_speed_045: list[float] = []
    valid_indices = np.flatnonzero(valid)
    for local_index in np.flatnonzero(moving):
        source_index = valid_indices[local_index]
        displacement = position[source_index + 1] - position[source_index]
        direction = displacement / np.linalg.norm(displacement)
        jacobian = arm_position_jacobian_3dof(q[source_index])
        qdot_at_045 = np.linalg.pinv(jacobian, rcond=1e-4) @ (0.45 * direction)
        equivalent_joint_speed_045.append(float(np.max(np.abs(qdot_at_045))))
    equivalent_045 = np.asarray(equivalent_joint_speed_045)
    return {
        "canonical_speed_intervals": int(len(cartesian_speed)),
        "canonical_cartesian_speed_max_m_s": float(np.max(cartesian_speed)),
        "canonical_cartesian_speed_p99_m_s": float(np.percentile(cartesian_speed, 99)),
        "canonical_joint_speed_max_rad_s": float(np.max(any_joint_speed)),
        "canonical_joint_speed_p99_rad_s": float(np.percentile(any_joint_speed, 99)),
        "canonical_joint_speed_p99_per_joint_rad_s": [
            float(np.percentile(joint_speed[:, joint], 99)) for joint in range(4)
        ],
        "canonical_intervals_over_ruckig_velocity_fraction": float(np.mean(
            any_joint_speed > RUCKIG_MAX_JOINT_VELOCITY_RAD_S
        )),
        "canonical_high_cartesian_speed_intervals": int(np.count_nonzero(high_cartesian)),
        "canonical_high_cartesian_any_joint_over_limit_fraction": float(np.mean(
            any_joint_speed[high_cartesian] > RUCKIG_MAX_JOINT_VELOCITY_RAD_S
        )) if np.any(high_cartesian) else 0.0,
        "equivalent_045_speed_samples": int(len(equivalent_045)),
        "equivalent_045_joint_speed_median_rad_s": float(np.median(equivalent_045)),
        "equivalent_045_joint_speed_p95_rad_s": float(np.percentile(equivalent_045, 95)),
        "equivalent_045_over_ruckig_velocity_fraction": float(np.mean(
            equivalent_045 > RUCKIG_MAX_JOINT_VELOCITY_RAD_S
        )),
    }


def add_speed_metrics(records: list[dict[str, Any]]) -> None:
    for record in records:
        speed = audit_canonical_speed_against_ruckig(Path(record["session_path"]))
        record.update(speed)


def _distribution_summary(values: list[float]) -> dict[str, float | int]:
    array = np.asarray(values, dtype=float)
    return {
        "n": int(len(array)),
        "mean": float(np.mean(array)),
        "sd": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
        "median": float(np.median(array)),
        "q1": float(np.percentile(array, 25)),
        "q3": float(np.percentile(array, 75)),
    }


def add_endpoint_trial_metrics(records: list[dict[str, Any]]) -> None:
    """Reduce every trial's endpoint error time series to one RMSE point."""
    for record in records:
        errors = load_endpoint_tracking_errors(Path(record["session_path"]))
        position_error = errors["position_error_mm"]
        orientation_error = errors["orientation_error_deg"]
        record["endpoint_position_rmse_mm"] = float(np.sqrt(np.mean(position_error ** 2)))
        record["endpoint_orientation_rmse_deg"] = float(np.sqrt(np.mean(orientation_error ** 2)))
        record["endpoint_tracking_samples"] = int(len(position_error))


def build_fig3_endpoint_tracking_errors(
    records: list[dict[str, Any]], output_stem: Path
) -> dict[str, dict[str, dict[str, float | int]]]:
    conditions = list(REPEATED_CONDITIONS.values())
    labels = [REPEATED_CONDITION_LABELS_ZH[condition].replace(" + ", "\n+ ") for condition in conditions]
    metric_specs = (
        ("endpoint_position_rmse_mm", "末端位置跟踪误差", "每试次位置 RMSE (mm)"),
        ("endpoint_orientation_rmse_deg", "末端姿态跟踪误差", "每试次姿态 RMSE (°)"),
    )
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 5.3), constrained_layout=True)
    summary: dict[str, dict[str, dict[str, float | int]]] = {condition: {} for condition in conditions}
    rng = np.random.default_rng(20260824)
    for panel_index, (axis, (metric, title, ylabel)) in enumerate(zip(axes, metric_specs)):
        grouped = [[float(row[metric]) for row in records if row["condition"] == condition]
                   for condition in conditions]
        box = axis.boxplot(
            grouped, positions=np.arange(1, len(conditions) + 1), widths=0.52,
            patch_artist=True, showfliers=False, medianprops={"color": "#111827", "linewidth": 1.4},
            whiskerprops={"color": "#64748B"}, capprops={"color": "#64748B"},
        )
        for patch, condition in zip(box["boxes"], conditions):
            patch.set_facecolor(CONDITION_COLORS[condition])
            patch.set_alpha(0.22)
            patch.set_edgecolor(CONDITION_COLORS[condition])
            patch.set_linewidth(1.2)
        for group_index, (condition, values) in enumerate(zip(conditions, grouped), start=1):
            stats = _distribution_summary(values)
            summary[condition]["position" if panel_index == 0 else "orientation"] = stats
            jitter = rng.uniform(-0.105, 0.105, size=len(values))
            axis.scatter(
                group_index + jitter, values, s=31, color=CONDITION_COLORS[condition],
                edgecolor="white", linewidth=0.55, alpha=0.88, zorder=3,
            )
            axis.scatter(group_index, stats["mean"], marker="D", s=34, color="#111827",
                         edgecolor="white", linewidth=0.6, zorder=4)
            annotation = (
                f"n={stats['n']}\n"
                f"均值±SD {stats['mean']:.2f}±{stats['sd']:.2f}\n"
                f"中位数[IQR] {stats['median']:.2f} "
                f"[{stats['q1']:.2f}, {stats['q3']:.2f}]"
            )
            axis.text(group_index, 0.985, annotation, transform=axis.get_xaxis_transform(),
                      ha="center", va="top", fontsize=7.1, color="#374151")
        axis.set_title(f"({chr(ord('a') + panel_index)}) {title}", loc="left")
        axis.set_ylabel(ylabel)
        axis.set_xticks(np.arange(1, len(conditions) + 1), labels)
        axis.set_xlim(0.45, len(conditions) + 0.55)
        axis.set_ylim(bottom=0.0)
        axis.grid(axis="y", linewidth=0.5, alpha=0.3)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Fig. 3  全部有效试次的末端跟踪误差分布", fontsize=13)
    fig.text(0.5, 0.004, "每个圆点代表一个 trial 的全时段 RMSE；箱体为 IQR，中线为中位数，菱形为均值。",
             ha="center", fontsize=8)
    fig.get_layout_engine().set(rect=(0.0, 0.035, 1.0, 0.95))
    save_figure(fig, output_stem)
    return summary


def run_repeated_analysis(data_root: Path, output_dir: Path) -> None:
    if not data_root.is_dir():
        raise ValueError(f"data root does not exist: {data_root}")
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    valid_records: list[dict[str, Any]] = []
    all_trials: list[dict[str, Any]] = []
    session_dirs = sorted(path for group in data_root.iterdir() if group.is_dir() for path in group.iterdir() if path.is_dir())
    for path in session_dirs:
        manifest_path = path / "manifest.json"
        method = "unknown"
        condition = None
        if manifest_path.exists():
            try:
                method = str(json.loads(manifest_path.read_text(encoding="utf-8")).get("method", "unknown"))
                condition = REPEATED_CONDITIONS.get(method)
            except (OSError, json.JSONDecodeError):
                pass
        record, reason = load_repeated_trial(path)
        if record is not None:
            valid_records.append(record)
            all_trials.append(record)
        else:
            all_trials.append({
                "condition": condition,
                "condition_zh": REPEATED_CONDITION_LABELS_ZH.get(condition, ""),
                "method": method,
                "session_id": path.name,
                "folder_name": path.name,
                "session_path": str(path.resolve()),
                "included": False,
                "exclusion_reason": reason or "unknown",
                "is_representative": False,
            })
    selected = select_representatives(valid_records)
    add_endpoint_trial_metrics(valid_records)
    add_speed_metrics(valid_records)
    summaries = build_repeated_group_summary(valid_records, all_trials, selected)
    build_fig1_arm_source_raw_signals(selected, output_dir / "Fig1_E1_arm_source_raw_signals")
    event_summary = build_fig2_representative_wrist_rows(selected, [
        output_dir / "Fig2_E1_canonical_wrist_3d",
        output_dir / "Fig_E1_20260824_wrist_3d_overlay",
    ])
    endpoint_error_summary = build_fig3_endpoint_tracking_errors(
        valid_records, output_dir / "Fig3_E1_endpoint_tracking_errors"
    )
    detail_columns = [
        "condition_zh", "condition", "method", "session_id", "folder_name", "included", "exclusion_reason",
        "is_representative", "representative_score", "duration_s", "canonical_valid_samples",
        "canonical_total_samples", "canonical_valid_fraction", "wrist_path_length_m", "wrist_mean_speed_m_s",
        "wrist_dimensionless_jerk", "wrist_log10_dimensionless_jerk", "overall_arm_rmse_rad",
        "overall_hand_rmse_unit", "endpoint_position_rmse_mm", "endpoint_orientation_rmse_deg",
        "endpoint_tracking_samples", "canonical_speed_intervals", "canonical_cartesian_speed_max_m_s",
        "canonical_cartesian_speed_p99_m_s", "canonical_joint_speed_max_rad_s",
        "canonical_joint_speed_p99_rad_s", "canonical_joint_speed_p99_per_joint_rad_s",
        "canonical_intervals_over_ruckig_velocity_fraction", "canonical_high_cartesian_speed_intervals",
        "canonical_high_cartesian_any_joint_over_limit_fraction", "equivalent_045_speed_samples",
        "equivalent_045_joint_speed_median_rad_s", "equivalent_045_joint_speed_p95_rad_s",
        "equivalent_045_over_ruckig_velocity_fraction", "session_path",
    ]
    summary_columns = [
        "condition_zh", "condition", "scanned_n", "valid_n", "failed_n", "representative_session_id",
        "duration_s_mean", "duration_s_sd", "wrist_path_length_m_mean", "wrist_path_length_m_sd",
        "wrist_dimensionless_jerk_median", "wrist_dimensionless_jerk_q1", "wrist_dimensionless_jerk_q3",
        "overall_arm_rmse_rad_mean", "overall_arm_rmse_rad_sd", "overall_hand_rmse_unit_mean",
        "overall_hand_rmse_unit_sd", "canonical_valid_fraction_mean",
        "endpoint_position_rmse_mm_mean", "endpoint_position_rmse_mm_sd",
        "endpoint_orientation_rmse_deg_mean", "endpoint_orientation_rmse_deg_sd",
        "canonical_cartesian_speed_max_m_s_mean", "canonical_cartesian_speed_max_m_s_sd",
        "canonical_cartesian_speed_p99_m_s_mean", "canonical_cartesian_speed_p99_m_s_sd",
        "canonical_joint_speed_max_rad_s_mean", "canonical_joint_speed_max_rad_s_sd",
        "canonical_joint_speed_p99_rad_s_mean", "canonical_joint_speed_p99_rad_s_sd",
        "canonical_intervals_over_ruckig_velocity_fraction_mean",
        "canonical_intervals_over_ruckig_velocity_fraction_sd",
        "equivalent_045_joint_speed_median_rad_s_mean", "equivalent_045_joint_speed_median_rad_s_sd",
        "equivalent_045_joint_speed_p95_rad_s_mean", "equivalent_045_joint_speed_p95_rad_s_sd",
        "equivalent_045_over_ruckig_velocity_fraction_mean",
        "equivalent_045_over_ruckig_velocity_fraction_sd",
    ]
    write_csv_records(output_dir / "E1_20260824_trial_metrics.csv", all_trials, detail_columns)
    write_csv_records(output_dir / "E1_20260824_group_summary.csv", summaries, summary_columns)
    json_records = [{key: value for key, value in row.items() if key != "canonical_rows"} for row in all_trials]
    report = {
        "data_root": str(data_root.resolve()),
        "output_dir": str(output_dir.resolve()),
        "method": {
            "failure_rule": "folder name contains 'fail' (case-insensitive)",
            "duration": "manifest.duration_sec; recording start-to-stop interval",
            "wrist_motion": "canonical wrist position resampled to 50 Hz and Savitzky-Golay smoothed (~0.4 s, polynomial order 3)",
            "dimensionless_jerk": "T^5 / L^2 * integral(||d3p/dt3||^2 dt); lower is smoother",
            "representative_metrics": list(REPRESENTATIVE_METRICS),
            "representative_rule": "minimum equal-weight sum of absolute robust z distances to group medians; scale=1.4826*MAD",
            "event_proxies": "global minimum z = grasp; global minimum y = release",
            "endpoint_error": "target/actual arm q1-q4 mapped through the registered 0.424 m + 0.424 m FK; position norm in mm and quaternion geodesic angle in degrees",
            "ruckig_velocity_audit": "canonical 50 Hz IK target finite differences compared with the verified downstream per-joint velocity limit of 1.0 rad/s; observed motion directions are also normalized to 0.45 m/s through the analytic position Jacobian",
        },
        "counts": {
            "scanned": len(all_trials),
            "valid": len(valid_records),
            "excluded": len(all_trials) - len(valid_records),
        },
        "representatives": {
            condition: {
                "session_id": row["session_id"],
                "session_path": row["session_path"],
                "representative_score": row["representative_score"],
                **{key: row[key] for key in REPRESENTATIVE_METRICS},
            }
            for condition, row in selected.items()
        },
        "group_summary": summaries,
        "representative_events": event_summary,
        "endpoint_error_group_statistics": endpoint_error_summary,
        "trials": json_records,
        "outputs": {
            "fig1_png": str((output_dir / "Fig1_E1_arm_source_raw_signals.png").resolve()),
            "fig1_pdf": str((output_dir / "Fig1_E1_arm_source_raw_signals.pdf").resolve()),
            "fig1_svg": str((output_dir / "Fig1_E1_arm_source_raw_signals.svg").resolve()),
            "fig2_png": str((output_dir / "Fig2_E1_canonical_wrist_3d.png").resolve()),
            "fig2_pdf": str((output_dir / "Fig2_E1_canonical_wrist_3d.pdf").resolve()),
            "fig2_svg": str((output_dir / "Fig2_E1_canonical_wrist_3d.svg").resolve()),
            "fig3_png": str((output_dir / "Fig3_E1_endpoint_tracking_errors.png").resolve()),
            "fig3_pdf": str((output_dir / "Fig3_E1_endpoint_tracking_errors.pdf").resolve()),
            "fig3_svg": str((output_dir / "Fig3_E1_endpoint_tracking_errors.svg").resolve()),
            "trial_csv": str((output_dir / "E1_20260824_trial_metrics.csv").resolve()),
            "summary_csv": str((output_dir / "E1_20260824_group_summary.csv").resolve()),
            "workbook_xlsx": str((output_dir / "E1_20260824_summary.xlsx").resolve()),
        },
    }
    report_path = output_dir / "E1_20260824_analysis.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_single_condition_analysis(data_root: Path, output_dir: Path, method: str,
                                  excluded_sessions: list[str], expected_trials: int | None) -> None:
    """Single-condition E1 metrics using the original motion/tracking/speed functions."""
    records, excluded = [], []
    for manifest_path in sorted(data_root.rglob("manifest.json")):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("method") != method:
            continue
        path = manifest_path.parent
        session_id = str(manifest.get("session_id") or path.name)
        if session_id in excluded_sessions or path.name in excluded_sessions:
            excluded.append({"session_id": session_id, "reason": "explicit_user_exclusion"})
            continue
        record, reason = load_repeated_trial(path)
        if record is None:
            excluded.append({"session_id": session_id, "reason": reason})
            continue
        record["started_at_ns"] = int(manifest["started_at_ns"])
        record["success_annotation"] = manifest.get("success", "unknown")
        aligned = read_csv(path / "aligned_20hz.csv")
        record["aligned_rows"] = len(aligned)
        for key in ("arm", "hand"):
            record[f"{key}_valid_rows"] = sum(truthy(row.get(f"{key}_valid", "")) for row in aligned)
            record[f"{key}_valid_fraction"] = record[f"{key}_valid_rows"] / len(aligned)
        record["arm_rmse_per_joint_rad"] = compute_arm_rmse(aligned)[0].tolist()
        record["hand_rmse_per_channel_unit"] = compute_hand_rmse(aligned)[0].tolist()
        record["hand_target_span_units"] = []
        for channel in range(1, 7):
            values = [_finite_number(row.get(f"hand_target_position_units_{channel}")) for row in aligned]
            finite_values = [v for v in values if v is not None]
            record["hand_target_span_units"].append(max(finite_values) - min(finite_values) if finite_values else None)
            record[f"hand_h{channel}_rmse_unit"] = record["hand_rmse_per_channel_unit"][channel - 1]
        for channel in range(1, 5):
            record[f"arm_j{channel}_rmse_rad"] = record["arm_rmse_per_joint_rad"][channel - 1]
        records.append(record)
    records.sort(key=lambda row: (row["started_at_ns"], row["session_id"]))
    if not records or (expected_trials is not None and len(records) != expected_trials):
        raise ValueError(f"expected {expected_trials} valid trials; found {len(records)}; exclusions={excluded}")
    add_endpoint_trial_metrics(records)
    add_speed_metrics(records)
    for index, record in enumerate(records, 1):
        record["analysis_sequence"] = index
        errors = load_endpoint_tracking_errors(Path(record["session_path"]))
        for label, key, unit in (("position", "position_error_mm", "mm"), ("orientation", "orientation_error_deg", "deg")):
            record[f"endpoint_{label}_p95_{unit}"] = float(np.percentile(errors[key], 95))
            record[f"endpoint_{label}_peak_{unit}"] = float(np.max(errors[key]))
        for key in ("canonical_rows", "representative_score", "is_representative"):
            record.pop(key)
    metric_keys = [key for key in records[0] if (
        key.startswith(("overall_", "wrist_", "endpoint_", "canonical_", "equivalent_"))
        or key in ("duration_s", "arm_valid_fraction", "hand_valid_fraction"))
        and isinstance(records[0][key], (float, int))]
    summary = {key: _distribution_summary([row[key] for row in records]) for key in metric_keys}
    for key, count in (("arm_rmse_per_joint_rad", 4), ("hand_rmse_per_channel_unit", 6)):
        summary[key] = [_distribution_summary([row[key][i] for row in records]) for i in range(count)]
    for key in ("endpoint_position_rmse_mm", "endpoint_orientation_rmse_deg"):
        summary[key]["pooled_rmse"] = float(np.sqrt(np.average(
            [row[key] ** 2 for row in records], weights=[row["endpoint_tracking_samples"] for row in records])))
    report = {"method": method, "condition_zh": records[0]["condition_zh"], "n": len(records),
              "data_root": str(data_root.resolve()), "excluded": excluded, "summary": summary, "trials": records}
    def finite_json(value):
        if isinstance(value, dict): return {k: finite_json(v) for k, v in value.items()}
        if isinstance(value, list): return [finite_json(v) for v in value]
        if isinstance(value, float) and not math.isfinite(value): return None
        return value
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv_records(output_dir / "E1_trial_metrics.csv", records, list(records[0]))
    write_csv_records(output_dir / "E1_group_summary.csv",
                      [{"metric": k, **v} for k, v in summary.items() if isinstance(v, dict)],
                      ["metric", "n", "mean", "sd", "median", "q1", "q3", "pooled_rmse"])
    configure_style()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis, (key, title, unit) in zip(axes.flat, (
        ("endpoint_position_rmse_mm", "末端位置跟踪", "RMSE (mm)"),
        ("endpoint_orientation_rmse_deg", "末端姿态跟踪", "RMSE (°)"),
        ("overall_hand_rmse_unit", "手部六通道跟踪", "RMSE (编码器单位)"),
        ("wrist_log10_dimensionless_jerk", "canonical 轨迹平滑性", "log10(无量纲 jerk)，越低越平滑"),
    )):
        axis.bar(range(1, len(records) + 1), [row[key] for row in records], color="#7c3aed")
        axis.set(title=title, xlabel="分析序号（按时间排列）", ylabel=unit, xticks=range(1, len(records) + 1))
        axis.grid(axis="y", alpha=.25)
    fig.suptitle(f"{records[0]['condition_zh']}：{len(records)} 组 E1 指标")
    save_figure(fig, output_dir / "E1_single_condition_metrics")
    lines = [f"# {records[0]['condition_zh']}：{len(records)} 组独立 E1 分析", "",
             "排除：" + "、".join(row["session_id"] for row in excluded), "",
             "组均值与样本标准差按每组等权计算；pooled_rmse 另按有效样本数加权。", "",
             "| 指标 | 均值 ± 标准差 | 中位数 |", "| --- | ---: | ---: |"]
    labels = {
        "duration_s": "录制时长 (s)", "wrist_path_length_m": "canonical 路径长度 (m)",
        "wrist_mean_speed_m_s": "canonical 平均速度 (m/s)",
        "wrist_log10_dimensionless_jerk": "log10 无量纲 jerk（越低越平滑）",
        "overall_arm_rmse_rad": "四关节整体 RMSE (rad)",
        "overall_hand_rmse_unit": "手部六通道整体 RMSE (编码器单位)",
        "endpoint_position_rmse_mm": "末端位置 RMSE (mm)",
        "endpoint_orientation_rmse_deg": "腕部姿态/J4 RMSE (°)",
        "endpoint_position_p95_mm": "各组位置误差 P95 (mm)",
        "endpoint_position_peak_mm": "各组位置误差峰值 (mm)",
        "canonical_cartesian_speed_max_m_s": "各组 canonical 笛卡尔速度最大值 (m/s)",
        "canonical_cartesian_speed_p99_m_s": "各组 canonical 笛卡尔速度 P99 (m/s)",
        "canonical_joint_speed_p99_rad_s": "各组 canonical 最大关节速度 P99 (rad/s)",
        "canonical_intervals_over_ruckig_velocity_fraction": "关节速度超过参考 1rad/s 的比例（0–1）",
        "canonical_valid_fraction": "canonical 有效比例（0–1）",
        "arm_valid_fraction": "臂对齐有效比例（0–1）", "hand_valid_fraction": "手对齐有效比例（0–1）",
    }
    for key, label in labels.items():
        s = summary[key]
        lines.append(f"| {label} | {s['mean']:.5g} ± {s['sd']:.5g} | {s['median']:.5g} |")
    worst = max(records, key=lambda row: row["endpoint_orientation_rmse_deg"])
    position_values = [row["endpoint_position_rmse_mm"] for row in records]
    other_orientation = [row["endpoint_orientation_rmse_deg"] for row in records if row is not worst]
    lines += ["", "## 结果解读", "",
              f"- 总录制时长 {sum(r['duration_s'] for r in records):.2f} s；有效末端跟踪样本 {sum(r['endpoint_tracking_samples'] for r in records)} 个。",
              f"- 各组位置 RMSE 范围 {min(position_values):.2f}–{max(position_values):.2f} mm；合并样本 RMSE {summary['endpoint_position_rmse_mm']['pooled_rmse']:.2f} mm。",
              f"- 姿态误差最高为 {worst['session_id']}，RMSE {worst['endpoint_orientation_rmse_deg']:.3f}°；该组保留在全部统计中。",
              f"- 其余组姿态 RMSE 范围 {min(other_orientation):.3f}–{max(other_orientation):.3f}°，仅作为分布说明，不改变所选数据。" if other_orientation else "",
              "", "## 关节与手指通道", "", "| 通道 | 各组 RMSE 均值 ± 标准差 |", "| --- | ---: |"]
    for i, s in enumerate(summary["arm_rmse_per_joint_rad"], 1):
        lines.append(f"| J{i} (rad) | {s['mean']:.6f} ± {s['sd']:.6f} |")
    for label, s in zip(("拇指弯曲", "拇指横摇", "食指", "中指", "无名指", "小指"), summary["hand_rmse_per_channel_unit"]):
        lines.append(f"| {label} (编码器单位) | {s['mean']:.3f} ± {s['sd']:.3f} |")
    static_channels = [str(i + 1) for i in range(6) if all(row["hand_target_span_units"][i] == 0 for row in records)]
    if static_channels:
        lines += ["", "所有试次中目标固定的手部通道：H" + "、H".join(static_channels) + "。这些通道的低误差仅反映保持目标时的表现，不能评价动态跟踪。"]
    lines += ["", "## 逐组结果", "", "| 序号 | 原始会话 | 时长 s | 位置 RMSE mm | 姿态 RMSE ° | 手部 RMSE unit | log10 jerk |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for r in records:
        lines.append(f"| {r['analysis_sequence']:02d} | {r['session_id']} | {r['duration_s']:.2f} | {r['endpoint_position_rmse_mm']:.2f} | {r['endpoint_orientation_rmse_deg']:.2f} | {r['overall_hand_rmse_unit']:.2f} | {r['wrist_log10_dimensionless_jerk']:.3f} |")
    notes = [
        "沿用 build_E1_comparison.py 原有计算函数；位置误差来自同一对齐时刻目标／实际关节的 FK，无时间平移，不等同于外部测量的绝对精度。原脚本姿态四元数仅由 J4 生成，该项表示腕部转动误差，不是完整三维姿态误差。",
        "无量纲 jerk 使用原脚本 50Hz 重采样、约 0.4 秒三阶 Savitzky-Golay 平滑及 T^5/L^2 积分归一化，越低越平滑。",
        "速度超过 1rad/s 是相对于脚本参考值的统计，不代表实机超限；equivalent_045 是沿记录方向以假设 0.45m/s 运动的雅可比估计，不是实际速度。",
        "手部单位为编码器单位，通道依次为拇指弯曲、拇指横摇、食指、中指、无名指、小指；各通道详细结果保存在 JSON 与逐组 CSV。",
        "complete 仅表示录制完成；success 未标注，不能计算任务成功率。没有其他组别对照，不作跨输入方式优劣判断。",
    ]
    report["definitions"] = notes
    lines += ["", "## 口径与限制", "", *["- " + n for n in notes], "", "![逐组指标](E1_single_condition_metrics.png)", ""]
    (output_dir / "E1_report.md").write_text("\n".join(lines), encoding="utf-8")
    (output_dir / "E1_single_condition_analysis.json").write_text(
        json.dumps(finite_json(report), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(finite_json({"n": len(records), "excluded": excluded, "summary": summary}), ensure_ascii=False, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", action="append", type=Path,
                        help="recording session directory (4 conditions, any order)")
    parser.add_argument("--data-root", type=Path,
                        help="repeated-trial root containing three arm-source condition folders")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).with_name("outputs"))
    parser.add_argument("--method", choices=[*REPEATED_CONDITIONS, "keyboard+preset"], help="analyze only one condition")
    parser.add_argument("--exclude-session", action="append", default=[], help="explicit original session ID to omit")
    parser.add_argument("--expected-trials", type=int, help="require exactly this many valid selected trials")
    args = parser.parse_args()
    if (args.method or args.exclude_session or args.expected_trials is not None) and args.data_root is None:
        parser.error("single-condition options require --data-root")
    if (args.exclude_session or args.expected_trials is not None) and not args.method:
        parser.error("--exclude-session/--expected-trials require --method")
    if args.data_root is not None:
        if args.session:
            parser.error("--data-root and --session cannot be used together")
        try:
            if args.method:
                run_single_condition_analysis(args.data_root, args.output_dir, args.method,
                                              args.exclude_session, args.expected_trials)
            else:
                run_repeated_analysis(args.data_root, args.output_dir)
        except ValueError as exc:
            parser.error(str(exc))
        return
    if not args.session:
        parser.error("provide either --data-root or four --session arguments")
    if len(args.session) != 4:
        parser.error("exactly 4 sessions required (keyboard / gamepad / VR controller / VR hand)")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()

    datasets: list[dict] = []
    for path in args.session:
        manifest = json.loads((path / "manifest.json").read_text(encoding="utf-8"))
        method = str(manifest.get("method", "unknown"))
        condition = CONDITION_NAMES.get(method)
        if condition is None:
            parser.error(f"unknown method {method!r} in {path}")
        start_ns = int(manifest.get("started_at_ns", 0))
        duration_s = float(manifest.get("duration_sec", 0.0)) or 60.0
        end_s = duration_s * 1.02
        raw_rows = load_raw_input(path / "raw_input.jsonl", start_ns, end_s)
        canonical_rows = load_canonical(path / "canonical_goal.jsonl", start_ns, end_s)
        aligned_path = path / "aligned_20hz.csv"
        aligned_rows = read_csv(aligned_path) if aligned_path.exists() else []
        if len(canonical_rows) < 2:
            parser.error(f"insufficient canonical rows in {path}")

        grasp_index = find_grasp_event(canonical_rows)
        release_index = find_release_event(canonical_rows, grasp_index)
        arm_rmse = compute_arm_rmse(aligned_rows)
        hand_rmse = compute_hand_rmse(aligned_rows)

        datasets.append({
            "condition": condition,
            "method": method,
            "session_id": str(manifest.get("session_id") or path.name),
            "duration_s": duration_s,
            "rows": canonical_rows,
            "raw_rows": raw_rows,
            "event_indices": (0, grasp_index, release_index),
            "arm_rmse": arm_rmse,
            "hand_rmse": hand_rmse,
        })
    datasets.sort(key=lambda data: CONDITION_ORDER.index(data["condition"]))

    build_fig_raw_inputs(datasets, args.output_dir / "Fig_E1_raw_inputs")
    build_fig_canonical_wrist_3d(datasets, args.output_dir / "Fig_E1_canonical_wrist_3d")
    build_fig_grasp_hand_skeleton(datasets, args.output_dir / "Fig_E1_grasp_hand_skeleton")
    build_fig_rmse(datasets, args.output_dir / "Fig_E1_rmse")

    summary = {
        "figures": {
            "raw_inputs": str((args.output_dir / "Fig_E1_raw_inputs.png").resolve()),
            "canonical_wrist_3d": str((args.output_dir / "Fig_E1_canonical_wrist_3d.png").resolve()),
            "grasp_hand_skeleton": str((args.output_dir / "Fig_E1_grasp_hand_skeleton.png").resolve()),
            "rmse": str((args.output_dir / "Fig_E1_rmse.png").resolve()),
        },
        "conditions": {
            data["condition"]: {
                "method": data["method"],
                "session_id": data["session_id"],
                "events_s": {
                    name: round(data["rows"][data["event_indices"][i]]["t"], 3)
                    for i, name in enumerate(("start", "grasp", "release"))
                },
                "arm_rmse_per_joint_rad": [None if math.isnan(v) else round(float(v), 4) for v in data["arm_rmse"][0]],
                "overall_arm_rmse_rad": None if math.isnan(data["arm_rmse"][1]) else round(data["arm_rmse"][1], 4),
                "hand_rmse_per_channel_unit": [None if math.isnan(v) else round(float(v), 2) for v in data["hand_rmse"][0]],
                "overall_hand_rmse_unit": None if math.isnan(data["hand_rmse"][1]) else round(data["hand_rmse"][1], 2),
            }
            for data in datasets
        },
        "notes": (
            "Fig. E1-1 shows native master signals per condition (keyboard key "
            "events; gamepad normalized intent in -1..1; VR/PICO wrist position "
            "in m with optional roll in deg) — not amplitude-comparable across "
            "modalities, no cross-device normalization. Grasp/release marked by "
            "vertical lines without time realignment. "
            "Hand RMSE channels H1-H6 follow the slave (ch1=thumb pitch, ch2=thumb yaw, "
            "ch3-ch6 = index-pinky).  Thumb channels are only meaningful for recordings "
            "made after the 2026-08-21 driver response-slot fix."
        ),
    }
    summary_path = args.output_dir / "Fig_E1_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
