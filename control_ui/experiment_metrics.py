import math
import re


RAW_COLUMNS = [
    "elapsed_ms",
    "desired_x_m",
    "desired_y_m",
    "desired_z_m",
    "actual_x_m",
    "actual_y_m",
    "actual_z_m",
    "actual_j1_rad",
    "actual_j2_rad",
    "actual_j3_rad",
    "actual_j4_rad",
    "actual_j5_rad",
    "error_mm",
]

SUMMARY_COLUMNS = [
    "date",
    "task",
    "method",
    "trial",
    "success",
    "duration_s",
    "final_desired_x_m",
    "final_desired_y_m",
    "final_desired_z_m",
    "final_actual_x_m",
    "final_actual_y_m",
    "final_actual_z_m",
    "final_error_mm",
    "path_length_mm",
    "path_efficiency",
]


def safe_slug(value, default="trial"):
    text = str(value or default).strip()
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("._-")
    return text or default


def to_float(value, default=0.0):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def normalize_success(value):
    text = str(value or "").strip().lower()
    if text in {"1", "true", "yes", "y", "success", "succeeded", "ok"}:
        return "yes"
    if text in {"0", "false", "no", "n", "fail", "failed"}:
        return "no"
    return "unknown"


def normalize_raw_sample(sample):
    row = {}
    for column in RAW_COLUMNS:
        row[column] = to_float((sample or {}).get(column), 0.0)

    if not (sample or {}).get("error_mm"):
        desired = [row["desired_x_m"], row["desired_y_m"], row["desired_z_m"]]
        actual = [row["actual_x_m"], row["actual_y_m"], row["actual_z_m"]]
        row["error_mm"] = distance_m(desired, actual) * 1000.0
    return row


def distance_m(a, b):
    return math.sqrt(
        (to_float(a[0]) - to_float(b[0])) ** 2
        + (to_float(a[1]) - to_float(b[1])) ** 2
        + (to_float(a[2]) - to_float(b[2])) ** 2
    )


def point_from_row(row, prefix):
    return [
        to_float(row.get(f"{prefix}_x_m")),
        to_float(row.get(f"{prefix}_y_m")),
        to_float(row.get(f"{prefix}_z_m")),
    ]


def path_length_mm(rows):
    total_m = 0.0
    last_point = None
    for row in rows:
        point = point_from_row(row, "actual")
        if last_point is not None:
            total_m += distance_m(last_point, point)
        last_point = point
    return total_m * 1000.0


def compute_trial_summary(metadata, rows):
    rows = [normalize_raw_sample(row) for row in rows]
    if not rows:
        raise ValueError("trial has no samples")

    first = rows[0]
    final = rows[-1]
    path_mm = path_length_mm(rows)
    straight_mm = distance_m(point_from_row(first, "actual"), point_from_row(final, "desired")) * 1000.0
    if path_mm <= 1e-9:
        efficiency = 1.0 if straight_mm <= 1e-9 else 0.0
    else:
        efficiency = straight_mm / path_mm

    return {
        "date": str((metadata or {}).get("date") or ""),
        "task": str((metadata or {}).get("task") or ""),
        "method": str((metadata or {}).get("method") or ""),
        "trial": str((metadata or {}).get("trial") or ""),
        "success": normalize_success((metadata or {}).get("success")),
        "duration_s": to_float(final.get("elapsed_ms")) / 1000.0,
        "final_desired_x_m": final["desired_x_m"],
        "final_desired_y_m": final["desired_y_m"],
        "final_desired_z_m": final["desired_z_m"],
        "final_actual_x_m": final["actual_x_m"],
        "final_actual_y_m": final["actual_y_m"],
        "final_actual_z_m": final["actual_z_m"],
        "final_error_mm": final["error_mm"],
        "path_length_mm": path_mm,
        "path_efficiency": efficiency,
    }
