import base64
import binascii
import csv
import hashlib
import json
import math
import os
import shlex
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    from control_arbitration import (
        SELECTABLE_MODES,
        arbitrate_ws_frame,
        normalize_arm_mode,
        normalize_client_id,
    )
except ImportError:
    from control_ui.control_arbitration import (
        SELECTABLE_MODES,
        arbitrate_ws_frame,
        normalize_arm_mode,
        normalize_client_id,
    )

try:
    from experiment_metrics import (
        RAW_COLUMNS,
        SUMMARY_COLUMNS,
        compute_trial_summary,
        normalize_raw_sample,
        normalize_success,
        safe_slug,
    )
except ImportError:
    from control_ui.experiment_metrics import (
        RAW_COLUMNS,
        SUMMARY_COLUMNS,
        compute_trial_summary,
        normalize_raw_sample,
        normalize_success,
        safe_slug,
    )

try:
    from recording_service import RecordingManager
except ImportError:
    from control_ui.recording_service import RecordingManager

try:
    from canonical_goal import CanonicalGoalBuilder
except ImportError:
    from control_ui.canonical_goal import CanonicalGoalBuilder

try:
    from semantic_admittance import (
        SemanticAdmittance, SemanticLimits, arm_forward_kinematics,
        arm_inverse_kinematics, canonical_snapshot, normalize_semantic_mode,
    )
except ImportError:
    from control_ui.semantic_admittance import (
        SemanticAdmittance, SemanticLimits, arm_forward_kinematics,
        arm_inverse_kinematics, canonical_snapshot, normalize_semantic_mode,
    )

try:
    from perception_assist_monitor import PerceptionAssistMonitor, load_monitor_config
except ImportError:
    from control_ui.perception_assist_monitor import PerceptionAssistMonitor, load_monitor_config


def unique_paths(paths):
    seen = set()
    result = []
    for path in paths:
        try:
            resolved = Path(path).resolve()
        except Exception:
            continue
        key = str(resolved).lower()
        if key in seen:
            continue
        seen.add(key)
        result.append(resolved)
    return result


def runtime_candidate_dirs():
    cwd = Path(os.getcwd())
    file_dir = Path(__file__).resolve().parent
    candidates = [cwd, file_dir, file_dir.parent, cwd.parent]
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.extend([exe_dir, exe_dir.parent])
        mei_pass = getattr(sys, "_MEIPASS", None)
        if mei_pass:
            mei_dir = Path(mei_pass)
            candidates.extend([mei_dir, mei_dir.parent])
    return unique_paths(candidates)


def find_control_ui_dir():
    for directory in runtime_candidate_dirs():
        if (directory / "config.yaml").exists():
            return directory
    return Path(os.getcwd()).resolve()


BASE_DIR = find_control_ui_dir()
PROJECT_ROOT = BASE_DIR.parent


def load_config():
    cfg_path = BASE_DIR / "config.yaml"
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config file not found: {cfg_path}")
    text = cfg_path.read_text(encoding="utf-8")
    try:
        import yaml

        return yaml.safe_load(text) or {}
    except ModuleNotFoundError as e:
        try:
            return json.loads(text)
        except Exception:
            try:
                return parse_simple_yaml(text)
            except Exception as yaml_error:
                raise RuntimeError("config.yaml parse failed. Install pyyaml or keep config.yaml as simple key/value YAML.") from yaml_error


def parse_config_value(value):
    value = value.strip()
    if not value:
        return ""
    if value[0:1] in {"[", "{"}:
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            pass
    if value[0:1] in ["'", '"'] and value[-1:] == value[0]:
        return value[1:-1]
    low = value.lower()
    if low in ["true", "yes", "on"]:
        return True
    if low in ["false", "no", "off"]:
        return False
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def parse_simple_yaml(text):
    """Tiny parser for this repo's simple two-level config.yaml."""
    root = {}
    current = None
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith((" ", "\t")):
            if not line.endswith(":"):
                raise ValueError(f"Expected section header: {raw_line}")
            current = line[:-1].strip()
            root[current] = {}
            continue
        if current is None or ":" not in line:
            raise ValueError(f"Expected indented key/value: {raw_line}")
        key, value = line.strip().split(":", 1)
        root[current][key.strip()] = parse_config_value(value)
    return root

cfg = load_config()
udp_cfg = cfg.get("udp", {})
ws_cfg = cfg.get("websocket", {})
api_cfg = cfg.get("api", {})
jetson_cfg = cfg.get("jetson", {})
program_cfg = cfg.get("programs", {})
local_check_cfg = cfg.get("local_checks", {})
hand_control_cfg = cfg.get("hand_control", {})
experiment_cfg = cfg.get("experiment", {})
recording_cfg = cfg.get("recording", {})
arm_control_cfg = cfg.get("arm_control", {})
rtsp_camera_cfg = cfg.get("rtsp_dual_camera", {})
canonical_cfg = cfg.get("canonical", {})
semantic_cfg = cfg.get("semantic_admittance", {})

SSH_COMMON_OPTIONS = [
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=5",
    "-o", "UserKnownHostsFile=NUL",
    "-o", "StrictHostKeyChecking=no",
    "-o", "LogLevel=ERROR",
]

WS_HOST = ws_cfg.get("host", "0.0.0.0")
WS_PORT = int(ws_cfg.get("port", 8080))
UDP_PATH = ws_cfg.get("path", "/udp")
STATUS_PATH = ws_cfg.get("status_path", "/status")
RAW_INPUT_PATH = ws_cfg.get("raw_input_path", "/raw-input")
CANONICAL_PATH = ws_cfg.get("canonical_path", "/canonical-goal")
API_HOST = api_cfg.get("host", "0.0.0.0")
API_PORT = int(api_cfg.get("port", 8090))

UDP_HOST = udp_cfg.get("host", "127.0.0.1")
UDP_RECV_PORT = int(udp_cfg.get("recv_port", 14550))
UDP_SEND_PORT = int(udp_cfg.get("send_port", 14550))
MATLAB_HOST = udp_cfg.get("matlab_host", "127.0.0.1")
MATLAB_PORT = int(udp_cfg.get("matlab_port", 50010))
HAND_HOST = udp_cfg.get("hand_host", "127.0.0.1")
HAND_PORT = int(udp_cfg.get("hand_port", 50020))
HAND_UDP_LISTEN_HOST = local_check_cfg.get("hand_udp_host", "127.0.0.1")
HAND_UDP_LISTEN_PORT = int(local_check_cfg.get("hand_udp_port", 25001))
LOCAL_PROGRAM_DELAY_SEC = float(local_check_cfg.get("local_program_delay_sec", 5))
HAND_READY_TIMEOUT_SEC = float(local_check_cfg.get("hand_ready_timeout_sec", 10))
GLOVE_READY_TIMEOUT_SEC = float(local_check_cfg.get("glove_ready_timeout_sec", 15))
HAND_CONTROL_REPEAT_COUNT = max(1, int(local_check_cfg.get("hand_control_repeat_count", 6)))
HAND_CONTROL_REPEAT_INTERVAL_SEC = max(0.0, float(local_check_cfg.get("hand_control_repeat_interval_sec", 0.025)))
LOOP_FILTER = str(udp_cfg.get("loop_filter", 1)).lower() not in ["0", "false", "no"]
LOOP_FILTER_WINDOW = float(udp_cfg.get("loop_filter_window", 0.5))
EXPERIMENT_LOG_DIR = BASE_DIR / str(experiment_cfg.get("log_dir", "experiment_logs"))
RECORDING_ROOT = BASE_DIR / str(recording_cfg.get("root_dir", "recordings"))
CAMERA_SERVICE_HOST = str(recording_cfg.get("camera_service_host", "127.0.0.1"))
CAMERA_SERVICE_PORT = int(recording_cfg.get("camera_service_port", 8092))
HAND_TELEMETRY_HOST = str(recording_cfg.get("hand_telemetry_host", "127.0.0.1"))
HAND_TELEMETRY_PORT = int(recording_cfg.get("hand_telemetry_port", 25003))
MASTER_FUSION_HOST = str(recording_cfg.get("master_fusion_host", "127.0.0.1"))
MASTER_FUSION_PORT = int(recording_cfg.get("master_fusion_port", 25007))
RAW_MASTER_HOST = str(recording_cfg.get("raw_master_host", "127.0.0.1"))
RAW_MASTER_PORT = int(recording_cfg.get("raw_master_port", 25009))
RECORDING_MIN_FREE_BYTES = int(float(recording_cfg.get("min_free_gib", 2.0)) * 1024 ** 3)
RECORDING_MAX_DURATION_SEC = float(recording_cfg.get("max_duration_minutes", 10.0)) * 60.0
RECORDING_ALIGNMENT_HZ = float(recording_cfg.get("alignment_hz", 20.0))
CANONICAL_CONTROL_ENABLED = str(canonical_cfg.get("control_path_enabled", True)).strip().lower() in {"1", "true", "yes", "on"}
CANONICAL_LOG_ENABLED = str(canonical_cfg.get("log_enabled", True)).strip().lower() in {"1", "true", "yes", "on"}
CANONICAL_STALE_TIMEOUT_SEC = float(canonical_cfg.get("stale_timeout_sec", 0.5))
CANONICAL_PUBLISH_HZ = max(1.0, float(canonical_cfg.get("publish_hz", 50.0)))
CANONICAL_PUBLISH_PERIOD_NS = int(round(1e9 / CANONICAL_PUBLISH_HZ))
CANONICAL_HAND_MODEL_PATH = BASE_DIR / str(canonical_cfg.get("wa100_kinematics_model_path", "wa100_kinematics_model.json"))
CANONICAL_HAND_HOST = str(canonical_cfg.get("hand_host", HAND_UDP_LISTEN_HOST))
CANONICAL_HAND_PORT = int(canonical_cfg.get("hand_port", HAND_UDP_LISTEN_PORT))
SEMANTIC_DEFAULT_MODE = normalize_semantic_mode(semantic_cfg.get("mode", "OFF")) or "OFF"
RECORDING = RecordingManager(
    RECORDING_ROOT,
    f"http://{CAMERA_SERVICE_HOST}:{CAMERA_SERVICE_PORT}",
    RECORDING_MIN_FREE_BYTES,
    RECORDING_MAX_DURATION_SEC,
    alignment_hz=RECORDING_ALIGNMENT_HZ,
    canonical_log_enabled=CANONICAL_LOG_ENABLED,
    canonical_publish_hz=CANONICAL_PUBLISH_HZ,
    canonical_queue_capacity=int(canonical_cfg.get("record_queue_capacity", 4096)),
)
CANONICAL = CanonicalGoalBuilder(
    CANONICAL_HAND_MODEL_PATH,
    CANONICAL_STALE_TIMEOUT_SEC,
    planner_config=canonical_cfg,
)
SEMANTIC = SemanticAdmittance(SemanticLimits(
    measured_timeout_sec=float(semantic_cfg.get("measured_timeout_sec", 0.25)),
    current_timeout_sec=float(semantic_cfg.get("current_timeout_sec", 0.25)),
    trigger_ma=float(semantic_cfg.get("trigger_ma", 80.0)),
    wrist_max_displacement_m=float(semantic_cfg.get("wrist_max_displacement_m", 0.025)),
    wrist_max_speed_m_s=float(semantic_cfg.get("wrist_max_speed_m_s", 0.02)),
    hand_max_unload_ratio=float(semantic_cfg.get("hand_max_unload_ratio", 0.25)),
    registered_retreat_direction_C=tuple(semantic_cfg.get("registered_retreat_direction_C", [-1.0, 0.0, 0.0])),
), SEMANTIC_DEFAULT_MODE, hand_kinematics=CANONICAL.hand_kinematics)
CANONICAL_MEASURED_LOCK = threading.RLock()
CANONICAL_MEASURED = {"arm": None, "hand": None, "current": None}
SLAVE_COMMAND_LOCK = threading.RLock()
LATEST_SLAVE_COMMAND = None
CANONICAL_HAND_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CANONICAL_ASYNC_CONDITION = threading.Condition()
CANONICAL_LATEST_ENVELOPE = None
CANONICAL_LATEST_VERSION = 0
CANONICAL_PERF_LOCK = threading.RLock()
CANONICAL_PERF = {
    "publish_starts_ns": deque(maxlen=4096),
    "critical_ms": deque(maxlen=4096),
    "build_ms": deque(maxlen=4096),
    "semantic_ms": deque(maxlen=4096),
    "measured_fk_ms": deque(maxlen=4096),
    "admittance_ms": deque(maxlen=4096),
    "ik_bundle_ms": deque(maxlen=4096),
    "record_enqueue_ms": deque(maxlen=4096),
    "deadline_misses": 0,
    "ws_sent": 0,
    "ws_errors": 0,
    "ws_overwritten": 0,
    "hand_udp_sent": 0,
    "hand_udp_errors": 0,
    "hand_udp_overwritten": 0,
}
PERCEPTION_CONFIG_PATH = BASE_DIR / "perception_assist_config.json"
PERCEPTION_CONFIG = load_monitor_config(PERCEPTION_CONFIG_PATH)
PERCEPTION_MONITOR = PerceptionAssistMonitor(PERCEPTION_CONFIG)
PERCEPTION_UNITY_TARGET = (
    str(PERCEPTION_CONFIG.get("unity_udp_host", "127.0.0.1")),
    int(PERCEPTION_CONFIG.get("unity_udp_port", 25004)),
)
PERCEPTION_UNITY_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
PERCEPTION_LAST_PAYLOAD = None
PERCEPTION_LAST_RECEIVE_MONOTONIC_NS = 0
PERCEPTION_LAST_STATUS_PUSH = 0.0
ADMITTANCE_ASSIST_TIMEOUT_SEC = 0.25
RUNTIME_MOTION_DURATION_SEC = max(0.001, float(arm_control_cfg.get("runtime_motion_duration_sec", 5.0)))
ARM_LEGACY_HOME_FALLBACK = str(arm_control_cfg.get("legacy_home_fallback", False)).strip().lower() in {"1", "true", "yes", "on"}


def config_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


CAMERA_STARTUP_MODES = {"display", "stream"}
CAMERA_STARTUP_SETTINGS_PATH = BASE_DIR / "logs" / "camera_startup_mode.json"


def normalize_camera_startup_mode(value, default="display"):
    mode = str(value or "").strip().lower()
    if mode in CAMERA_STARTUP_MODES:
        return mode
    return default


def read_camera_startup_override(path=None):
    settings_path = Path(path or CAMERA_STARTUP_SETTINGS_PATH)
    try:
        data = json.loads(settings_path.read_text(encoding="utf-8-sig"))
    except (FileNotFoundError, OSError, ValueError, TypeError):
        return None
    return normalize_camera_startup_mode(data.get("mode"), None) if isinstance(data, dict) else None


def save_camera_startup_override(mode, path=None):
    normalized = normalize_camera_startup_mode(mode, None)
    if normalized is None:
        raise ValueError("mode must be 'display' or 'stream'")

    settings_path = Path(path or CAMERA_STARTUP_SETTINGS_PATH)
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = settings_path.with_name(
        f"{settings_path.name}.{os.getpid()}.{threading.get_ident()}.tmp"
    )
    payload = {
        "mode": normalized,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
    }
    try:
        temp_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temp_path, settings_path)
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass
    return normalized


CAMERA_DEFAULT_STARTUP_MODE = normalize_camera_startup_mode(
    rtsp_camera_cfg.get("startup_mode"), "display"
)
CAMERA_ACTIVE_MODE = normalize_camera_startup_mode(
    os.environ.get("UEM_CAMERA_ACTIVE_MODE"),
    read_camera_startup_override() or CAMERA_DEFAULT_STARTUP_MODE,
)


def camera_video_size():
    raw = str(rtsp_camera_cfg.get("video_size", "640x480")).lower().split("x", 1)
    try:
        width, height = int(raw[0]), int(raw[1])
    except (ValueError, IndexError):
        return 640, 480
    return max(1, width), max(1, height)


def camera_startup_settings():
    next_mode = read_camera_startup_override() or CAMERA_DEFAULT_STARTUP_MODE
    width, height = camera_video_size()
    hls_port = int(rtsp_camera_cfg.get("hls_port", 8081))
    stream_path = str(rtsp_camera_cfg.get("stream_path", "usb_camera")).strip("/") or "usb_camera"
    cameras = [
        {
            "slot": "left",
            "name": str(rtsp_camera_cfg.get("left_video_device", "") or ""),
            "index": int(rtsp_camera_cfg.get("left_camera_index", 0)),
            "mirror": config_bool(rtsp_camera_cfg.get("left_mirror"), True),
            "vertical_flip": config_bool(rtsp_camera_cfg.get("left_vertical_flip"), False),
        },
        {
            "slot": "right",
            "name": str(rtsp_camera_cfg.get("right_video_device", "") or ""),
            "index": int(rtsp_camera_cfg.get("right_camera_index", 1)),
            "mirror": config_bool(rtsp_camera_cfg.get("right_mirror"), True),
            "vertical_flip": config_bool(rtsp_camera_cfg.get("right_vertical_flip"), False),
        },
    ]
    return {
        "active_mode": CAMERA_ACTIVE_MODE,
        "next_mode": next_mode,
        "default_mode": CAMERA_DEFAULT_STARTUP_MODE,
        "restart_required": next_mode != CAMERA_ACTIVE_MODE,
        "cameras": cameras,
        "video": {
            "width": width,
            "height": height,
            "framerate": max(1, int(rtsp_camera_cfg.get("framerate", 30))),
            "layout": str(rtsp_camera_cfg.get("layout", "vstack") or "vstack"),
        },
        "hls_url": f"http://localhost:{hls_port}/{stream_path}/index.m3u8",
        "left_preview_url": f"http://localhost:{CAMERA_SERVICE_PORT}/left.mjpg",
        "right_preview_url": f"http://localhost:{CAMERA_SERVICE_PORT}/right.mjpg",
        "camera_service_url": f"http://localhost:{CAMERA_SERVICE_PORT}",
    }


def normalize_hand_mode(mode):
    value = str(mode or "idle").strip().lower()
    aliases = {
        "vr_hand": "vr",
        "vrhand": "vr",
        "hand_tracking": "vr",
        "hand-tracking": "vr",
    }
    value = aliases.get(value, value)
    return value if value in {"idle", "vr", "glove", "preset"} else "idle"


DEFAULT_HAND_MODE = normalize_hand_mode(hand_control_cfg.get("default_mode", "idle"))
START_GLOVE_BY_DEFAULT = config_bool(hand_control_cfg.get("start_glove_by_default"), DEFAULT_HAND_MODE == "glove")

ADMITTANCE_MODE_NAMES = {
    0: "off",
    1: "handonly",
    2: "holistic",
    3: "handconfig",
}


def normalize_admittance_mode(value):
    """Normalize 0..3 or off/handonly/handconfig/holistic to 0..3; None if invalid."""
    if isinstance(value, bool):
        return None
    if isinstance(value, str):
        aliases = {
            "off": 0, "none": 0, "0": 0,
            "handonly": 1, "hand_only": 1, "legacy": 1, "1": 1,
            "holistic": 2, "arm_hand": 2, "2": 2,
            "handconfig": 3, "hand_config": 3, "config": 3, "3": 3,
        }
        value = aliases.get(value.strip().lower())
    if isinstance(value, float) and not float(value).is_integer():
        return None
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value in ADMITTANCE_MODE_NAMES else None

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if os.name == "nt":
    try:
        udp.ioctl(0x9800000C, False)
    except Exception:
        pass
udp.bind(("0.0.0.0", UDP_RECV_PORT))

recent_ws_crc = deque(maxlen=256)
UDP_CLIENTS = set()
STATUS_CLIENTS = set()
RAW_INPUT_CLIENTS = set()
CANONICAL_CLIENTS = set()
CLIENT_LOCK = threading.RLock()
STATE_LOCK = threading.RLock()
CAMERA_FRAME_LOCK = threading.RLock()
EXPERIMENT_LOCK = threading.RLock()
ARM_COMMAND_LOCK = threading.RLock()
CANONICAL_PUBLISH_LOCK = threading.RLock()
LOGS = deque(maxlen=300)
PENDING_ARM_COMMANDS = {}
PROCESSES = {}
LAST_UDP_RESET_LOG = 0.0
LAST_CAMERA_STATUS_PUSH = 0.0
MAX_CAMERA_FRAME_BYTES = 2 * 1024 * 1024
CAMERA_FRAME = {
    "data": None,
    "content_type": "image/jpeg",
    "time": 0.0,
    "seq": 0,
}


def experiment_date_key():
    return time.strftime("%Y%m%d", time.localtime())


def experiment_day_dir(day=None):
    return EXPERIMENT_LOG_DIR / (safe_slug(day or experiment_date_key(), experiment_date_key()))


def experiment_raw_dir(day=None):
    return experiment_day_dir(day) / "raw"


def experiment_paths(trial_id, day=None):
    safe_id = safe_slug(trial_id, "trial")
    day_dir = experiment_day_dir(day)
    return {
        "day_dir": day_dir,
        "raw_dir": day_dir / "raw",
        "raw_file": day_dir / "raw" / f"{safe_id}_samples.csv",
        "summary_file": day_dir / "summary.csv",
    }


def ensure_csv_header(path, columns):
    path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not path.exists() or path.stat().st_size == 0
    if needs_header:
        with path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=columns)
            writer.writeheader()


def append_csv_rows(path, columns, rows):
    if not rows:
        return 0
    ensure_csv_header(path, columns)
    with path.open("a", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
        for row in rows:
            writer.writerow(row)
    return len(rows)


def read_raw_samples(path):
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as fh:
        return [row for row in csv.DictReader(fh)]


def relative_to_base(path):
    try:
        return str(path.resolve().relative_to(BASE_DIR.resolve()))
    except Exception:
        return str(path)


def normalize_experiment_samples(samples):
    if samples is None:
        return []
    if not isinstance(samples, list):
        raise ValueError("samples must be a list")
    return [normalize_raw_sample(sample) for sample in samples]


def experiment_metadata_from_body(body):
    body = body or {}
    return {
        "date": safe_slug(body.get("date") or experiment_date_key(), experiment_date_key()),
        "task": safe_slug(body.get("task"), "task"),
        "method": safe_slug(body.get("method"), "method"),
        "trial": safe_slug(body.get("trial"), "1"),
        "success": normalize_success(body.get("success")),
    }


def start_experiment_trial(body):
    meta = experiment_metadata_from_body(body)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    msec = int((time.time() % 1.0) * 1000)
    trial_id = safe_slug(
        f"{stamp}_{meta['task']}_{meta['method']}_trial{meta['trial']}_{msec:03d}",
        "trial",
    )
    paths = experiment_paths(trial_id, meta["date"])
    with EXPERIMENT_LOCK:
        ensure_csv_header(paths["raw_file"], RAW_COLUMNS)
        ensure_csv_header(paths["summary_file"], SUMMARY_COLUMNS)
    return command_result(
        True,
        "experiment trial started",
        {
            "trial_id": trial_id,
            "raw_path": relative_to_base(paths["raw_file"]),
            "summary_path": relative_to_base(paths["summary_file"]),
            "date": meta["date"],
        },
    )


def append_experiment_samples(body):
    body = body or {}
    trial_id = safe_slug(body.get("trial_id"), "")
    if not trial_id:
        return command_result(False, "trial_id is required")
    date_key = safe_slug(body.get("date") or experiment_date_key(), experiment_date_key())
    try:
        samples = normalize_experiment_samples(body.get("samples"))
    except Exception as exc:
        return command_result(False, str(exc))
    paths = experiment_paths(trial_id, date_key)
    with EXPERIMENT_LOCK:
        count = append_csv_rows(paths["raw_file"], RAW_COLUMNS, samples)
    return command_result(True, f"experiment samples saved: {count}", {"saved": count})


def finish_experiment_trial(body):
    body = body or {}
    trial_id = safe_slug(body.get("trial_id"), "")
    if not trial_id:
        return command_result(False, "trial_id is required")
    meta = experiment_metadata_from_body(body)
    try:
        samples = normalize_experiment_samples(body.get("samples"))
    except Exception as exc:
        return command_result(False, str(exc))
    paths = experiment_paths(trial_id, meta["date"])
    with EXPERIMENT_LOCK:
        append_csv_rows(paths["raw_file"], RAW_COLUMNS, samples)
        raw_rows = read_raw_samples(paths["raw_file"])
        try:
            summary = compute_trial_summary(meta, raw_rows)
        except Exception as exc:
            return command_result(False, str(exc))
        append_csv_rows(paths["summary_file"], SUMMARY_COLUMNS, [summary])
    return command_result(
        True,
        "experiment trial finished",
        {
            "trial_id": trial_id,
            "summary": summary,
            "raw_path": relative_to_base(paths["raw_file"]),
            "summary_path": relative_to_base(paths["summary_file"]),
        },
    )

H5_FRAME_CRC_OFFSET = 289
H5_ORDER_COUNT = 16
H5_NOTE_SIZE = 64
H5_INVALID_FLOAT = -1.0
H5_SELECTOR_HOME = 10
ARM_HOME_REPEAT_COUNT = 8
ARM_HOME_REPEAT_INTERVAL_SEC = 0.035
ARM_SOURCE_REPEAT_COUNT = 3
ARM_SOURCE_REPEAT_INTERVAL_SEC = 0.02
ARM_PRESET_REPEAT_COUNT = 8
ARM_PRESET_REPEAT_INTERVAL_SEC = 0.035
ARM_TELEMETRY_TIMEOUT_SEC = 1.0
ARM_FRAME_COUNTER = 0
ARM_MOTION_ACTIVE_STATES = frozenset({"accepted", "running"})
ARM_MOTION_TERMINAL_STATES = frozenset({"complete", "failed", "preempted"})


def arm_motion_transition_is_regressive(previous_state, next_state):
    """Reject delayed UDP status packets that would re-lock a finished motion."""
    previous = str(previous_state or "idle").strip().lower()
    incoming = str(next_state or "unknown").strip().lower()
    if previous in ARM_MOTION_TERMINAL_STATES and incoming in ARM_MOTION_ACTIVE_STATES:
        return True
    return previous == "running" and incoming == "accepted"


def acknowledge_arm_command(ind):
    with ARM_COMMAND_LOCK:
        PENDING_ARM_COMMANDS.pop(int(ind) & 0xFFFFFFFF, None)


def send_h5_arm_frame_async(frame, attempts=3):
    ind = struct.unpack_from("<I", frame, 0)[0]
    with ARM_COMMAND_LOCK:
        PENDING_ARM_COMMANDS[ind] = True
    try:
        udp.sendto(frame, (UDP_HOST, UDP_SEND_PORT))
    except Exception as exc:
        acknowledge_arm_command(ind)
        return False, str(exc), ind

    def retry_worker():
        for delay in (0.05, 0.10)[:max(0, int(attempts) - 1)]:
            time.sleep(delay)
            with ARM_COMMAND_LOCK:
                if ind not in PENDING_ARM_COMMANDS:
                    return
            try:
                udp.sendto(frame, (UDP_HOST, UDP_SEND_PORT))
            except Exception:
                break
        with ARM_COMMAND_LOCK:
            unacknowledged = PENDING_ARM_COMMANDS.pop(ind, None) is not None
        if unacknowledged:
            with STATE_LOCK:
                motion = SYSTEM["arm_control"].get("motion", {})
                if motion.get("ind") == ind and motion.get("state") == "accepted":
                    motion["state"] = "failed"
                    SYSTEM["state"] = "ERROR"
            push_status()

    threading.Thread(target=retry_worker, daemon=True, name=f"arm-command-retry-{ind}").start()
    return True, f"H5 UDP {UDP_HOST}:{UDP_SEND_PORT} <- binary frame len={len(frame)} async", ind

SYSTEM = {
    "state": "OFFLINE",
    "hand_mode": DEFAULT_HAND_MODE,
    "arm_control": {
        "mode": "idle",
        "owner_id": None,
        "owner_since": 0.0,
        "last_telemetry_time": 0.0,
        "last_angles": None,
        "last_command_source": None,
        "motion": {
            "ind": None,
            "source": None,
            "state": "idle",
            "progress": 0.0,
            "duration_sec": RUNTIME_MOTION_DURATION_SEC,
        },
        "motion_companion_ok": True,
        "accepted_frames": 0,
        "rejected_frames": 0,
        "last_reject_reason": "",
    },
    "perception_assist": {
        "type": "perception_assist_state",
        "schema_version": 1,
        "state": "FREE",
        "valid": 0,
        "invalid_reason": "awaiting_wa100_telemetry_and_commissioned_baseline",
        "reason": "monitor_invalid_no_grasp_inference",
        "monitor_only": True,
        "grasp_success_confirmed": False,
        "loaded_fingers": [],
        "current_residual": [None] * 6,
    },
    # Independent arm/hand admittance policy state, reported by the WA100
    # schema 4 telemetry.  Never merged into perception_assist.
    "admittance_assist": {
        "type": "admittance_assist_state",
        "schema_version": 1,
        "mode": "unknown",
        "mode_raw": -1,
        "retreat_level": 0.0,
        "retreat_active": False,
        "closure": [None] * 6,
        "off_hold_active": [None] * 6,
        "gain_scale": [None] * 6,
        "trigger_on_ma": [None] * 6,
        "trigger_off_ma": [None] * 6,
        "load_evidence": [None] * 6,
        "fresh": False,
        "received_utc_ns": 0,
        "age_sec": None,
    },
    "canonical": CANONICAL.status(),
    "modules": {
        "backend": {"status": "ONLINE", "last_seen": time.time(), "message": "后端服务运行中"},
        "jetson": {"status": "OFFLINE", "last_seen": 0, "message": "未检查"},
        "arm": {"status": "OFFLINE", "last_seen": 0, "message": "未检查"},
        "udp_bridge": {"status": "OFFLINE", "last_seen": 0, "message": "未收到 UDP 桥接数据"},
        "hand": {"status": "OFFLINE", "last_seen": 0, "message": "未启动"},
        "glove": {"status": "OFFLINE", "last_seen": 0, "message": "未启动"},
        "hand_link": {"status": "OFFLINE", "last_seen": 0, "message": "未检测本地手套链路"},
        "matlab": {"status": "OFFLINE", "last_seen": 0, "message": "未连接"},
        "camera": {"status": "OFFLINE", "last_seen": 0, "message": "未配置检测"},
        "network": {"status": "OFFLINE", "last_seen": 0, "message": "未检查"},
    },
    "faults": [],
}

ERROR_HINTS = {
    "JETSON_UNREACHABLE": "Jetson 未连接，请检查电源、网线和 IP 地址，然后点击重新连接。",
    "CAN_STOPPED": "CAN 通讯节点未运行，请点击启动 Jetson 节点或检查 CAN 设备。",
    "ARM_STOPPED": "机械臂底层控制节点未运行，请点击启动 Jetson 节点或检查机械臂控制程序。",
    "HAND_EXE_MISSING": "灵巧手控制程序路径不存在，请检查 config.yaml 中的 hand_exe。",
    "GLOVE_OFFLINE": "数据手套程序离线，可切换到灵巧手预设手势模式。",
    "MATLAB_OFFLINE": "主手控制程序未响应，可切换到机械臂按钮控制或半自动演示模式。",
}


def now_ts():
    return time.strftime("%H:%M:%S")


def snapshot():
    with STATE_LOCK:
        now = time.time()
        arm_control = dict(SYSTEM.get("arm_control", {}))
        last_telemetry_time = float(arm_control.get("last_telemetry_time") or 0.0)
        telemetry_age = now - last_telemetry_time if last_telemetry_time else None
        arm_control["telemetry_age_sec"] = telemetry_age
        arm_control["telemetry_fresh"] = telemetry_age is not None and telemetry_age <= ARM_TELEMETRY_TIMEOUT_SEC
        return {
            "state": SYSTEM["state"],
            "hand_mode": SYSTEM.get("hand_mode", DEFAULT_HAND_MODE),
            "arm_control": arm_control,
            "perception_assist": json.loads(json.dumps(SYSTEM.get("perception_assist", {}), ensure_ascii=False)),
            "admittance_assist": json.loads(json.dumps(SYSTEM.get("admittance_assist", {}), ensure_ascii=False)),
            "semantic_admittance": {"mode": SEMANTIC.mode},
            "canonical": json.loads(json.dumps(SYSTEM.get("canonical", {}), ensure_ascii=False)),
            "modules": json.loads(json.dumps(SYSTEM["modules"], ensure_ascii=False)),
            "faults": list(SYSTEM["faults"])[-20:],
            "logs": list(LOGS)[:80],
            "server_time": time.time(),
        }


def push_status():
    payload = json.dumps(snapshot(), ensure_ascii=False).encode("utf-8")
    with CLIENT_LOCK:
        clients = list(STATUS_CLIENTS)
    for client in clients:
        try:
            client.send_text(payload)
        except Exception:
            client.close()


def log_event(level, message, detail=None):
    item = {"time": now_ts(), "level": level, "message": message}
    if detail:
        item["detail"] = str(detail)
    with STATE_LOCK:
        LOGS.appendleft(item)
    print(f"[{level}] {message}" + (f" | {detail}" if detail else ""))
    push_status()


def set_state(state, reason=None):
    with STATE_LOCK:
        old = SYSTEM["state"]
        SYSTEM["state"] = state
    if old != state:
        log_event("STATE", f"系统状态 {old} -> {state}", reason)
    else:
        push_status()


def set_module(name, status, message="", fault_code=None):
    with STATE_LOCK:
        mod = SYSTEM["modules"].setdefault(name, {"status": "OFFLINE", "last_seen": 0, "message": ""})
        old = mod.get("status")
        mod["status"] = status
        mod["message"] = message
        if status == "ONLINE":
            mod["last_seen"] = time.time()
        if fault_code:
            SYSTEM["faults"].append({"time": now_ts(), "code": fault_code, "message": ERROR_HINTS.get(fault_code, message)})
    if old != status:
        log_event("STATUS", f"{name} {old} -> {status}", message)
    else:
        push_status()


def set_hand_mode(mode):
    with STATE_LOCK:
        SYSTEM["hand_mode"] = normalize_hand_mode(mode)
    push_status()


def arm_telemetry_fresh(now=None):
    now = time.time() if now is None else float(now)
    with STATE_LOCK:
        last_seen = float(SYSTEM["arm_control"].get("last_telemetry_time") or 0.0)
    return bool(last_seen and now - last_seen <= ARM_TELEMETRY_TIMEOUT_SEC)


def current_arm_authority():
    with STATE_LOCK:
        control = SYSTEM["arm_control"]
        return control.get("mode", "idle"), control.get("owner_id")


def update_arm_authority_state(mode, owner_id=None):
    normalized = normalize_arm_mode(mode, allow_system=True)
    if normalized is None:
        raise ValueError(f"unsupported arm control mode: {mode}")
    with STATE_LOCK:
        control = SYSTEM["arm_control"]
        old_mode = control.get("mode", "idle")
        old_owner = control.get("owner_id")
        control["mode"] = normalized
        control["owner_id"] = owner_id
        control["owner_since"] = time.time()
        control["last_reject_reason"] = ""
    if old_mode != normalized or old_owner != owner_id:
        log_event("CONTROL", f"arm authority {old_mode} -> {normalized}", owner_id or "system")
    else:
        push_status()


def record_arm_frame_decision(decision):
    should_push = False
    with STATE_LOCK:
        control = SYSTEM["arm_control"]
        if decision.accepted:
            control["accepted_frames"] = int(control.get("accepted_frames") or 0) + 1
            if decision.frame is not None:
                control["last_command_source"] = decision.frame.source or decision.reason
        else:
            control["rejected_frames"] = int(control.get("rejected_frames") or 0) + 1
            previous = control.get("last_reject_reason") or ""
            control["last_reject_reason"] = decision.reason
            should_push = previous != decision.reason or control["rejected_frames"] % 50 == 0
    if should_push:
        push_status()


def update_arm_telemetry(data):
    if len(data) != 293:
        return False
    expected_crc = struct.unpack_from("<I", data, H5_FRAME_CRC_OFFSET)[0]
    actual_crc = binascii.crc32(data[:H5_FRAME_CRC_OFFSET]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        return False
    try:
        ind = struct.unpack_from("<I", data, 0)[0]
        angles = list(struct.unpack_from("<7f", data, 12))
    except struct.error:
        return False
    note = data[225:289].split(b"\x00", 1)[0].decode("utf-8", "ignore")
    if note.startswith("motion:"):
        parts = note.split(":", 2)
        state = parts[1] if len(parts) > 1 else "unknown"
        source = parts[2] if len(parts) > 2 else "preset"
        try:
            progress, duration = struct.unpack_from("<2f", data, 145)
        except struct.error:
            progress, duration = 0.0, RUNTIME_MOTION_DURATION_SEC
        motion = {
            "ind": ind,
            "source": source,
            "state": state,
            "progress": max(0.0, min(1.0, float(progress))),
            "duration_sec": max(0.0, float(duration)),
        }
        with STATE_LOCK:
            control = SYSTEM["arm_control"]
            control["last_telemetry_time"] = time.time()
            previous_motion = dict(control.get("motion") or {})
            same_motion = (
                previous_motion.get("ind") == ind
                and str(previous_motion.get("source") or "") == source
            )
            regressive = same_motion and arm_motion_transition_is_regressive(
                previous_motion.get("state"), state
            )
            if not regressive:
                control["motion"] = motion
                companion_ok = bool(control.get("motion_companion_ok", True))
                if state in ARM_MOTION_ACTIVE_STATES:
                    SYSTEM["state"] = ("HOMING" if source == "home" else "MOVING") if companion_ok else "ERROR"
                elif state == "complete":
                    SYSTEM["state"] = "READY" if companion_ok else "ERROR"
                elif state == "failed":
                    SYSTEM["state"] = "ERROR"
                elif state == "preempted":
                    SYSTEM["state"] = "READY" if companion_ok else "ERROR"

                # Home/preset owns the arm only for the duration of its motion.
                # Releasing the UI-side authority here lets a local/external
                # source be selected immediately after the terminal status.
                if state in ARM_MOTION_TERMINAL_STATES and control.get("mode") in {"home", "preset"}:
                    control["mode"] = "idle"
                    control["owner_id"] = None
                    control["owner_since"] = time.time()
                    control["last_reject_reason"] = ""
        acknowledge_arm_command(ind)
        push_status()
        return True
    if note == "ready" and all(abs(float(value) + 1.0) < 1e-6 for value in angles[:3]):
        return False
    if not all(math.isfinite(float(value)) for value in angles[:4]):
        return False
    with STATE_LOCK:
        control = SYSTEM["arm_control"]
        control["last_telemetry_time"] = time.time()
        control["last_angles"] = angles[:4]
    return True


def run_cmd(args, timeout=12):
    try:
        completed = subprocess.run(args, capture_output=True, text=True, timeout=timeout, shell=False)
        text = ((completed.stdout or "") + (completed.stderr or "")).strip()
        return completed.returncode == 0, text
    except subprocess.TimeoutExpired:
        return False, "命令执行超时"
    except Exception as exc:
        return False, str(exc)


def ping_host(host):
    if not host:
        return False, "未配置 Jetson host"
    args = ["ping", "-n", "1", "-w", "1000", host] if os.name == "nt" else ["ping", "-c", "1", "-W", "1", host]
    return run_cmd(args, timeout=4)


def ssh_run(script_key):
    host = jetson_cfg.get("host")
    user = jetson_cfg.get("user", "night")
    script = jetson_cfg.get(script_key)
    if not host or not script:
        return False, f"Jetson 配置不完整: host 或 {script_key}"
    return ssh_run_command(script, timeout=20)


def ssh_run_command(command, timeout=20):
    host = jetson_cfg.get("host")
    user = jetson_cfg.get("user", "night")
    if not host or not user:
        return False, "Jetson SSH config missing host or user"
    return run_cmd(["ssh", *SSH_COMMON_OPTIONS, f"{user}@{host}", command], timeout=timeout)


def parse_arm_status(text):
    can_running = "CAN_NODE:RUNNING" in text
    arm_running = "ARM_CONTROL:RUNNING" in text
    bridge_reported = "UDP_BRIDGE:" in text
    bridge_running = "UDP_BRIDGE:RUNNING" in text
    if bridge_running:
        set_module("udp_bridge", "ONLINE", "H5 UDP 桥接节点运行中")
    elif bridge_reported:
        set_module("udp_bridge", "ERROR", "H5 UDP 桥接节点未运行")
    if can_running and arm_running:
        set_module("arm", "ONLINE", "CAN 节点和机械臂控制节点运行中")
        return (not bridge_reported) or bridge_running
    if "CAN_NODE:STOPPED" in text or not can_running:
        set_module("arm", "ERROR", "CAN 通讯节点未运行", "CAN_STOPPED")
    if "ARM_CONTROL:STOPPED" in text or not arm_running:
        set_module("arm", "ERROR", "机械臂控制节点未运行", "ARM_STOPPED")
    return False


def program_args(name, path_key):
    arg_key = {"hand": "hand_args", "glove": "unity_glove_args"}.get(name, f"{path_key}_args")
    raw = str(program_cfg.get(arg_key, "") or "").strip()
    return shlex.split(raw) if raw else []


def udp_endpoints(port=None, pid=None):
    endpoints = []
    if os.name == "nt":
        filters = []
        if port is not None:
            filters.append(f"-LocalPort {int(port)}")
        if pid is not None:
            filters.append(f"-OwningProcess {int(pid)}")
        script = (
            "Get-NetUDPEndpoint "
            + " ".join(filters)
            + " -ErrorAction SilentlyContinue | "
            + "ForEach-Object { \"$($_.LocalAddress),$($_.LocalPort),$($_.OwningProcess)\" }"
        )
        ok, text = run_cmd(["powershell", "-NoProfile", "-Command", script], timeout=4)
        if ok:
            for line in text.splitlines():
                parts = [part.strip() for part in line.split(",")]
                if len(parts) != 3:
                    continue
                try:
                    endpoints.append({"address": parts[0], "port": int(parts[1]), "pid": int(parts[2])})
                except ValueError:
                    pass
            return endpoints

    ok, text = run_cmd(["netstat", "-ano", "-p", "UDP"], timeout=4)
    if not ok:
        return endpoints
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[0].upper() != "UDP":
            continue
        local = parts[1]
        owner = parts[-1]
        if ":" not in local:
            continue
        addr, local_port = local.rsplit(":", 1)
        try:
            local_port = int(local_port)
            owner_pid = int(owner)
        except ValueError:
            continue
        if port is not None and local_port != int(port):
            continue
        if pid is not None and owner_pid != int(pid):
            continue
        endpoints.append({"address": addr.strip("[]"), "port": local_port, "pid": owner_pid})
    return endpoints


def endpoint_matches_host(endpoint, host):
    address = endpoint.get("address", "")
    return address in {host, "0.0.0.0", "::", "::1"} or (host == "127.0.0.1" and address == "localhost")


def describe_endpoints(endpoints):
    return ", ".join(f"{item['address']}:{item['port']}/pid={item['pid']}" for item in endpoints)


def is_process_alive(name):
    proc = PROCESSES.get(name)
    return proc is not None and proc.poll() is None


def is_glove_required(mode=None):
    return normalize_hand_mode(mode or SYSTEM.get("hand_mode", DEFAULT_HAND_MODE)) == "glove"


def activate_hand_mode(mode, positions=None, name=None):
    normalized = normalize_hand_mode(mode)
    if normalized == "idle":
        return False, "请选择 VR 手部追踪、手套操控或预编程位置模式"
    if not is_process_alive("hand"):
        ok, msg = start_program("hand", "hand_exe")
        if not ok:
            return False, msg
    if normalized == "glove" and not is_process_alive("glove"):
        ok, msg = start_program("glove", "unity_glove")
        if not ok:
            return False, msg
    return send_hand_control(normalized, positions, name)


def refresh_hand_link_status():
    hand_eps = [item for item in udp_endpoints(port=HAND_UDP_LISTEN_PORT) if endpoint_matches_host(item, HAND_UDP_LISTEN_HOST)]
    glove_proc = PROCESSES.get("glove")
    glove_eps = udp_endpoints(pid=glove_proc.pid) if glove_proc and glove_proc.poll() is None else []
    if normalize_hand_mode(SYSTEM.get("hand_mode")) == "idle":
        if hand_eps and glove_eps:
            set_module("hand_link", "ONLINE", "灵巧手与 Unity 手套链路已就绪；当前为 idle 保持模式")
            return True
        if hand_eps or glove_eps:
            set_module("hand_link", "WARNING", "手部程序正在启动；当前为 idle 保持模式")
            return False
        set_module("hand_link", "WARNING", "手部控制模式为 idle，等待灵巧手与 Unity 手套链路就绪")
        return False
    if not is_glove_required():
        if hand_eps:
            set_module("hand_link", "ONLINE", f"VR hand tracking link ready: hand {describe_endpoints(hand_eps)}")
            return True
        if is_process_alive("hand"):
            set_module("hand_link", "WARNING", f"hand process is running, but UDP {HAND_UDP_LISTEN_HOST}:{HAND_UDP_LISTEN_PORT} is not listening")
            return False
        set_module("hand_link", "OFFLINE", "VR hand tracking link is offline")
        return False
    if hand_eps and glove_eps:
        set_module("hand_link", "ONLINE", f"本机 UDP 链路可检查: hand {describe_endpoints(hand_eps)}; glove {describe_endpoints(glove_eps)}")
        return True
    if hand_eps or glove_eps:
        set_module("hand_link", "WARNING", f"本机链路未完整: hand={describe_endpoints(hand_eps) or '无监听'}; glove={describe_endpoints(glove_eps) or '无UDP端点'}")
        return False
    if is_process_alive("hand") or is_process_alive("glove"):
        set_module("hand_link", "WARNING", "手/手套进程存在，但未检测到完整本机 UDP 链路")
        return False
    set_module("hand_link", "OFFLINE", "未检测本地手套链路")
    return False


def refresh_program_status(name):
    proc = PROCESSES.get(name)
    if proc is None:
        set_module(name, "OFFLINE", "未启动")
        refresh_hand_link_status()
        return False, "未启动"
    code = proc.poll()
    if code is not None:
        set_module(name, "ERROR", f"程序已退出，退出码 {code}")
        refresh_hand_link_status()
        return False, f"程序已退出，退出码 {code}"

    if name == "hand":
        endpoints = [
            item for item in udp_endpoints(port=HAND_UDP_LISTEN_PORT)
            if endpoint_matches_host(item, HAND_UDP_LISTEN_HOST) and item.get("pid") == proc.pid
        ]
        if endpoints:
            endpoint_text = describe_endpoints(endpoints)
            if RECORDING.hand_telemetry_fresh():
                msg = f"已监听 UDP {endpoint_text}，WA100 记录遥测正常"
                set_module(name, "ONLINE", msg)
                refresh_hand_link_status()
                return True, msg
            msg = f"控制端口已监听 {endpoint_text}，等待 WA100 UDP 25003 记录遥测"
            set_module(name, "WARNING", msg)
            refresh_hand_link_status()
            return False, msg
        msg = f"进程运行中，但未监听 UDP {HAND_UDP_LISTEN_HOST}:{HAND_UDP_LISTEN_PORT}"
        set_module(name, "WARNING", msg)
        refresh_hand_link_status()
        return False, msg

    if name == "glove":
        endpoints = udp_endpoints(pid=proc.pid)
        if endpoints:
            msg = f"进程运行中，本地 UDP 端点: {describe_endpoints(endpoints)}"
            set_module(name, "ONLINE", msg)
            refresh_hand_link_status()
            return True, msg
        msg = "进程运行中，但未发现该进程打开本地 UDP 端点"
        set_module(name, "WARNING", msg)
        refresh_hand_link_status()
        return False, msg

    set_module(name, "ONLINE", "程序运行中")
    return True, "程序运行中"


def wait_program_ready(name, timeout_sec):
    deadline = time.time() + float(timeout_sec)
    last_msg = ""
    while time.time() < deadline:
        ok, last_msg = refresh_program_status(name)
        if ok:
            return True, last_msg
        time.sleep(0.5)
    return refresh_program_status(name)


def start_program(name, path_key):
    path = program_cfg.get(path_key)
    if not path:
        set_module(name, "ERROR", f"未配置 {path_key}")
        return False, f"未配置 {path_key}"
    exe = Path(path)
    if not exe.is_absolute():
        exe = PROJECT_ROOT / exe
    if not exe.exists():
        set_module(name, "ERROR", f"程序不存在: {exe}", "HAND_EXE_MISSING" if name == "hand" else None)
        return False, f"程序不存在: {exe}"
    if name in PROCESSES and PROCESSES[name].poll() is None:
        set_module(name, "WARNING", "程序已在运行，正在检查本地监听")
        timeout_sec = 1 if name == "hand" else 1
        return wait_program_ready(name, timeout_sec)
    try:
        args = [str(exe)] + program_args(name, path_key)
        proc = subprocess.Popen(args, cwd=str(exe.parent), creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0))
        PROCESSES[name] = proc
        set_module(name, "WARNING", f"程序已启动 pid={proc.pid}，等待本地监听")
        timeout_sec = HAND_READY_TIMEOUT_SEC if name == "hand" else GLOVE_READY_TIMEOUT_SEC
        return wait_program_ready(name, timeout_sec)
    except Exception as exc:
        set_module(name, "ERROR", f"启动失败: {exc}")
        return False, str(exc)


def stop_program(name):
    proc = PROCESSES.get(name)
    if proc is not None and proc.poll() is None:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
    PROCESSES.pop(name, None)
    set_module(name, "OFFLINE", "程序已停止")
    refresh_hand_link_status()


def configured_exe(path_key):
    path = program_cfg.get(path_key)
    if not path:
        return None
    exe = Path(path)
    if not exe.is_absolute():
        exe = PROJECT_ROOT / exe
    return exe


def stop_configured_program(name, path_key):
    stop_program(name)
    exe = configured_exe(path_key)
    if not exe:
        return False, f"未配置 {path_key}"
    try:
        result = subprocess.run(
            ["taskkill", "/F", "/T", "/IM", exe.name],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode in (0, 128):
            set_module(name, "OFFLINE", "已关闭现有 exe")
            refresh_hand_link_status()
            return True, (result.stdout or result.stderr or "已关闭现有 exe").strip()
        return False, (result.stderr or result.stdout or "关闭 exe 失败").strip()
    except FileNotFoundError:
        return True, "taskkill 不可用，已停止当前托管进程"
    except Exception as exc:
        return False, str(exc)


def send_udp(host, port, text):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(text.encode("utf-8"), (host, int(port)))
        return True, f"UDP {host}:{port} <- {text}"
    except Exception as exc:
        return False, str(exc)


def stop_rtsp_camera_stream():
    script = BASE_DIR / "stop_rtsp_camera.ps1"
    if not script.exists():
        set_module("camera", "OFFLINE", "RTSP stop script was not found")
        return True, "RTSP stop script was not found"

    powershell = "powershell.exe" if os.name == "nt" else "pwsh"
    ok, msg = run_cmd(
        [powershell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        timeout=12,
    )
    if ok:
        set_module("camera", "OFFLINE", "RTSP camera stream stopped")
    else:
        set_module("camera", "WARNING", f"RTSP camera stop failed: {msg}")
    return ok, msg


def send_udp_repeat(host, port, text, count, interval_sec):
    try:
        sent = 0
        raw = text.encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            for idx in range(max(1, int(count))):
                s.sendto(raw, (host, int(port)))
                sent += 1
                if idx + 1 < count and interval_sec > 0:
                    time.sleep(interval_sec)
        repeat_suffix = f" x{sent}" if sent > 1 else ""
        return True, f"UDP {host}:{port} <- {text}{repeat_suffix}"
    except Exception as exc:
        return False, str(exc)


def fixed_frame_text(value, size):
    text = str(value or "")
    data = text.encode("utf-8")[:size]
    return data.ljust(size, b"\x00")


def next_arm_frame_counter():
    global ARM_FRAME_COUNTER
    value = ARM_FRAME_COUNTER
    ARM_FRAME_COUNTER = (ARM_FRAME_COUNTER + 1) & 0xFFFFFFFF
    return value


def pack_h5_arm_command_frame(mode, order=None, note="bridge"):
    values = list(order or [])
    if len(values) < H5_ORDER_COUNT:
        values.extend([0.0] * (H5_ORDER_COUNT - len(values)))
    values = [float(v) for v in values[:H5_ORDER_COUNT]]

    body = b"".join([
        struct.pack("<I", next_arm_frame_counter()),
        struct.pack("<Q", int(time.time() * 1000) & 0xFFFFFFFFFFFFFFFF),
        struct.pack("<7f", *([H5_INVALID_FLOAT] * 7)),
        struct.pack("<7f", *([H5_INVALID_FLOAT] * 7)),
        struct.pack("<7f", *([H5_INVALID_FLOAT] * 7)),
        struct.pack("<6f", *([H5_INVALID_FLOAT] * 6)),
        struct.pack("<6f", *([H5_INVALID_FLOAT] * 6)),
        struct.pack("<B", int(mode) & 0xFF),
        struct.pack("<16f", *values),
        b"\xFF" * 16,
        fixed_frame_text(note, H5_NOTE_SIZE),
    ])
    if len(body) != H5_FRAME_CRC_OFFSET:
        raise ValueError(f"bad H5 frame body length: {len(body)}")
    crc = binascii.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", crc)


def send_h5_arm_frame(frame, count=1, interval_sec=0.0):
    try:
        sent = 0
        for idx in range(max(1, int(count))):
            udp.sendto(frame, (UDP_HOST, UDP_SEND_PORT))
            if LOOP_FILTER:
                recent_ws_crc.append((binascii.crc32(frame) & 0xFFFFFFFF, time.time(), len(frame)))
            sent += 1
            if idx + 1 < count and interval_sec > 0:
                time.sleep(interval_sec)
        repeat_suffix = f" x{sent}" if sent > 1 else ""
        return True, f"H5 UDP {UDP_HOST}:{UDP_SEND_PORT} <- binary frame len={len(frame)}{repeat_suffix}"
    except Exception as exc:
        return False, str(exc)


def send_arm_home_frame():
    frame = pack_h5_arm_command_frame(
        H5_SELECTOR_HOME,
        [0.0] * H5_ORDER_COUNT,
        "bridge:home",
    )
    return send_h5_arm_frame_async(frame)


def send_arm_source_frame(mode):
    normalized = normalize_arm_mode(mode, allow_system=True)
    if normalized is None:
        return False, f"unsupported arm control mode: {mode}"
    selector = H5_SELECTOR_HOME if normalized == "home" else 0
    frame = pack_h5_arm_command_frame(
        selector,
        [0.0] * H5_ORDER_COUNT,
        f"bridge:source:{normalized}",
    )
    return send_h5_arm_frame(frame, ARM_SOURCE_REPEAT_COUNT, ARM_SOURCE_REPEAT_INTERVAL_SEC)


def acquire_arm_authority(mode, owner_id=None, require_telemetry=False):
    normalized = normalize_arm_mode(mode, allow_system=True)
    if normalized is None:
        return False, f"unsupported arm control mode: {mode}"
    if normalized in SELECTABLE_MODES:
        owner_id = normalize_client_id(owner_id)
        if owner_id is None:
            return False, "client_id must be a non-zero 16-byte hexadecimal identifier"
    else:
        owner_id = None
    if require_telemetry and not arm_telemetry_fresh():
        return False, "arm telemetry is stale; control remains idle"

    ok, msg = send_arm_source_frame(normalized)
    if not ok:
        return False, msg
    update_arm_authority_state(normalized, owner_id)
    return True, msg


def send_arm_preset_frame(degrees, name="preset"):
    if not isinstance(degrees, list) or len(degrees) != 4:
        return False, "degrees must contain exactly four joint angles", None
    try:
        radians = [math.radians(float(value)) for value in degrees]
    except (TypeError, ValueError):
        return False, "degrees must contain finite numbers", None
    if not all(math.isfinite(value) for value in radians):
        return False, "degrees must contain finite numbers", None
    order = radians + [0.0] * (H5_ORDER_COUNT - len(radians))
    frame = pack_h5_arm_command_frame(0x40, order, f"bridge:preset:{str(name or 'preset')[:40]}")
    return send_h5_arm_frame_async(frame)


HAND_CHANNEL_NAMES = ["thumb_pitch", "thumb_yaw", "index", "middle", "ring", "pinky"]

HAND_PRESETS = {
    "reset": [2000, 2000, 2000, 2000, 2000, 2000],
    "open": [2000, 2000, 2000, 2000, 2000, 2000],
    "half": [1000, 1000, 1000, 1000, 1000, 1000],
    "close": [200, 200, 200, 200, 200, 200],
    # Physical order: ID1 thumb pitch, ID2 thumb yaw, ID3-ID6 index-pinky.
    "pinch": [1200, 1600, 250, 2000, 2000, 2000],
    # Thumb opposition only: keep thumb pitch and all four fingers fully open.
    "hook": [2000, 300, 2000, 2000, 2000, 2000],
    # Touch anchors match the Hi5/PICO thumb contact fusion targets
    # (thumb_glove_calibration.json gesture_anchors).
    "thumb_index": [800, 300, 350, 2000, 2000, 2000],
    "thumb_middle": [800, 0, 2000, 350, 2000, 2000],
}


def clamp_hand_position(value):
    try:
        return max(0, min(2000, int(round(float(value)))))
    except Exception:
        return 2000


def sanitize_hand_positions(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("positions 必须是 6 个 0-2000 的数值")
    return [clamp_hand_position(value) for value in values]


def _preset_hand_value(name, positions, now_ns=None):
    now_ns = int(now_ns or time.time_ns())
    return {
        "stream": "hand_skeleton",
        "name": str(name),
        "positions": list(positions),
        "mapped_target_units": list(positions),
        "source_timestamp_ns": now_ns,
    }


def _observe_preset_hand(name, positions, now_ns=None):
    value = _preset_hand_value(name, positions, now_ns)
    accepted, reason = CANONICAL.observe_hand_input(
        "preset_hand", value, int(value["source_timestamp_ns"])
    )
    return accepted, reason, value


def send_hand_control(mode, positions=None, name=None):
    normalized_mode = normalize_hand_mode(mode)
    if normalized_mode == "idle":
        # Idle is a deterministic safe-open state, not a hold of whatever
        # encoder pose happened to be present when the executable started.
        positions = list(HAND_PRESETS["reset"])
        name = name or "idle_safe_open"
    elif normalized_mode == "preset" and positions is None:
        with STATE_LOCK:
            latched = dict(SYSTEM.get("hand_preset") or {})
        positions = list(latched.get("positions") or HAND_PRESETS["open"])
        name = name or str(latched.get("name") or "open")

    sanitized_positions = sanitize_hand_positions(positions) if positions is not None else None
    payload = {
        "type": "h5_hand_control",
        "schema_version": 2,
        "source": "control_ui",
        "mode": normalized_mode,
        "time": time.time(),
        "channel_names": HAND_CHANNEL_NAMES,
    }
    if name:
        payload["name"] = name
    if sanitized_positions is not None:
        payload["positions"] = sanitized_positions
    text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    # Prevent the 50 Hz publisher from sending an old target between the direct
    # mode/preset packet and the corresponding Canonical state transition.
    with CANONICAL_PUBLISH_LOCK:
        with CANONICAL.lock:
            previous_hand = (
                json.loads(json.dumps(CANONICAL.hand))
                if CANONICAL.hand is not None else None
            )
        if normalized_mode in {"preset", "idle"} and sanitized_positions is not None:
            accepted, reason, _ = _observe_preset_hand(
                name or normalized_mode, sanitized_positions
            )
            if not accepted:
                return False, f"Canonical preset rejected: {reason}"
        elif normalized_mode in {"vr", "glove"}:
            # Keep a stale placeholder so wrist/arm Canonical planning remains
            # available while the new hand source produces its first frame.
            # hand_mask stays zero, so the previous hand target is never sent.
            with CANONICAL.lock:
                if CANONICAL.hand is None:
                    placeholder = _preset_hand_value(
                        f"awaiting_{normalized_mode}", HAND_PRESETS["open"]
                    )
                    accepted, reason = CANONICAL.observe_hand_input(
                        "preset_hand", placeholder, int(placeholder["source_timestamp_ns"])
                    )
                    if not accepted:
                        return False, f"Canonical hand placeholder rejected: {reason}"
                CANONICAL.hand["receive_utc_ns"] = 0
                CANONICAL.hand.setdefault("invalid_reasons", []).append(
                    f"awaiting_{normalized_mode}_hand_adapter"
                )
                CANONICAL.last_error = f"awaiting {normalized_mode} hand adapter"

        ok, msg = send_udp_repeat(
            HAND_UDP_LISTEN_HOST,
            HAND_UDP_LISTEN_PORT,
            text,
            HAND_CONTROL_REPEAT_COUNT,
            HAND_CONTROL_REPEAT_INTERVAL_SEC,
        )
        if not ok:
            with CANONICAL.lock:
                CANONICAL.hand = previous_hand

    if ok:
        set_hand_mode(normalized_mode)
        refresh_hand_link_status()
        if normalized_mode == "preset" and name:
            log_event(
                "HAND",
                f"手型切换: {name}",
                f"mode={normalized_mode} positions={payload.get('positions')}",
            )
            sent_positions = payload.get("positions")
            with STATE_LOCK:
                SYSTEM["hand_preset"] = {
                    "name": str(name),
                    "positions": list(sent_positions) if sent_positions else [],
                    "changed_at": now_ts(),
                    "changed": True,
                }
            _record_preset_skeleton(name, sent_positions, update_canonical=False)
    return ok, msg


def send_admittance_mode(mode):
    """Send the admittance policy switch to the WA100 receiver (UDP 25001).

    mode: 0=off, 1=handonly, 2=holistic, 3=handconfig (int or name string).
    The authoritative state is whatever schema 4 telemetry reports back.
    """
    normalized = normalize_admittance_mode(mode)
    if normalized is None:
        return False, f"未知导纳模式: {mode!r}（支持 off/handonly/handconfig/holistic 或 0-3）"
    payload = {
        "type": "set_admittance_mode",
        "schema_version": 1,
        "mode": normalized,
        "source": "control_ui",
        "time": time.time(),
    }
    text = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    ok, msg = send_udp_repeat(
        HAND_UDP_LISTEN_HOST,
        HAND_UDP_LISTEN_PORT,
        text,
        HAND_CONTROL_REPEAT_COUNT,
        HAND_CONTROL_REPEAT_INTERVAL_SEC,
    )
    if ok:
        # Console print lives in the udp_receiver control loop; bridge only
        # keeps the UI-side log event.
        log_event("HAND", f"导纳模式切换: {ADMITTANCE_MODE_NAMES[normalized]}", f"mode={normalized}")
    return ok, msg


def _record_preset_skeleton(name, positions, update_canonical=True):
    """Publish one fixed posture through the same six-DOF Canonical adapter."""
    try:
        if not isinstance(positions, list) or len(positions) != 6:
            return
        now_ns = time.time_ns()
        if update_canonical:
            accepted, reason, value = _observe_preset_hand(name, positions, now_ns)
            if not accepted:
                raise ValueError(reason)
        else:
            value = _preset_hand_value(name, positions, now_ns)
        payload = {
            "type": "raw_master_input",
            "schema_version": 1,
            "source": "preset_hand",
            "signal_form": "discrete",
            "value": value,
        }
        RECORDING.observe_raw_input(payload, now_ns, time.monotonic_ns())
    except Exception as exc:
        log_event("WARNING", f"preset 骨架记录失败: {name}", exc)


def command_result(ok, message, extra=None):
    body = {"ok": ok, "message": message, "status": snapshot()}
    if extra:
        body.update(extra)
    return body


def store_camera_frame(raw, content_type):
    global LAST_CAMERA_STATUS_PUSH

    if not raw:
        return command_result(False, "empty camera frame")
    if len(raw) > MAX_CAMERA_FRAME_BYTES:
        return command_result(False, "camera frame is too large")

    now = time.time()
    safe_content_type = content_type or "image/jpeg"
    safe_content_type = safe_content_type.split(";", 1)[0].strip().lower() or "image/jpeg"

    with CAMERA_FRAME_LOCK:
        CAMERA_FRAME["data"] = raw
        CAMERA_FRAME["content_type"] = safe_content_type
        CAMERA_FRAME["time"] = now
        CAMERA_FRAME["seq"] = int(CAMERA_FRAME.get("seq") or 0) + 1
        seq = CAMERA_FRAME["seq"]

    with STATE_LOCK:
        mod = SYSTEM["modules"].setdefault("camera", {"status": "OFFLINE", "last_seen": 0, "message": ""})
        mod["status"] = "ONLINE"
        mod["last_seen"] = now
        mod["message"] = "PC browser USB camera frame bridge"

    if now - LAST_CAMERA_STATUS_PUSH > 1.0:
        LAST_CAMERA_STATUS_PUSH = now
        push_status()

    return command_result(True, "camera frame accepted", {"seq": seq})


def latest_camera_frame():
    with CAMERA_FRAME_LOCK:
        data = CAMERA_FRAME.get("data")
        content_type = CAMERA_FRAME.get("content_type") or "image/jpeg"
        timestamp = float(CAMERA_FRAME.get("time") or 0.0)
        seq = int(CAMERA_FRAME.get("seq") or 0)

    if not data:
        return None, content_type, timestamp, seq
    return data, content_type, timestamp, seq


def do_command(method, path, body=None):
    log_event("ACTION", f"{method} {path}")
    if path == "/api/recording/status" and method == "GET":
        with STATE_LOCK:
            recording_method = (
                f"{SYSTEM.get('arm_control', {}).get('mode', 'unknown')}+"
                f"{SYSTEM.get('hand_mode', 'unknown')}"
            )
        readiness = RECORDING.readiness(method=recording_method)
        return {
            "ok": True,
            **RECORDING.status(),
            "readiness": readiness,
            "canonical": CANONICAL.status(),
        }
    if path == "/api/recording/start" and method == "POST":
        with STATE_LOCK:
            arm_mode = SYSTEM.get("arm_control", {}).get("mode", "unknown")
            hand_mode = SYSTEM.get("hand_mode", "unknown")
            preset = dict(SYSTEM.get("hand_preset") or {})
        if hand_mode == "preset" and len(preset.get("positions") or []) == 6:
            # Refresh the discrete source before readiness is evaluated, then
            # publish it once more after start so the active trial owns a row.
            _record_preset_skeleton(preset.get("name") or "preset", list(preset["positions"]))
        result = RECORDING.start(f"{arm_mode}+{hand_mode}")
        if result.get("ok") and hand_mode == "preset" and len(preset.get("positions") or []) == 6:
            _record_preset_skeleton(preset.get("name") or "preset", list(preset["positions"]))
        return result
    if path == "/api/recording/stop" and method == "POST":
        return RECORDING.stop()
    if path == "/api/perception-assist/status" and method == "GET":
        with STATE_LOCK:
            status = json.loads(json.dumps(SYSTEM.get("perception_assist", {}), ensure_ascii=False))
        return {"ok": True, **status}
    if path == "/api/status":
        return command_result(True, "状态已返回")
    if path == "/api/canonical/planner" and method == "GET":
        return command_result(True, "canonical planner status", {
            "canonical": CANONICAL.status(),
        })
    if path == "/api/canonical/planner" and method == "POST":
        requested = (body or {}).get("profile") if isinstance(body, dict) else None
        try:
            selected = CANONICAL.set_planner_override(requested)
        except ValueError as exc:
            return command_result(False, str(exc), {
                "supported": ["default", "off", "jerk_limited", "one_euro"],
            })
        push_status()
        return command_result(True, f"canonical planner override set to {selected}", {
            "profile": selected,
            "canonical": CANONICAL.status(),
        })
    if path == "/api/camera/startup-mode" and method == "GET":
        return command_result(True, "摄像头启动模式已返回", camera_startup_settings())
    if path == "/api/camera/startup-mode" and method == "POST":
        requested_mode = (body or {}).get("mode") if isinstance(body, dict) else None
        try:
            saved_mode = save_camera_startup_override(requested_mode)
        except ValueError as exc:
            return command_result(False, str(exc), camera_startup_settings())
        except OSError as exc:
            return command_result(False, f"摄像头启动模式保存失败: {exc}", camera_startup_settings())
        return command_result(
            True,
            "摄像头模式已保存，将在下次一键启动时生效",
            {**camera_startup_settings(), "next_mode": saved_mode},
        )
    if path == "/api/experiment/trial/start" and method == "POST":
        return start_experiment_trial(body)
    if path == "/api/experiment/trial/samples" and method == "POST":
        return append_experiment_samples(body)
    if path == "/api/experiment/trial/finish" and method == "POST":
        return finish_experiment_trial(body)
    if path == "/api/arm/control" and method == "POST":
        requested_mode = str((body or {}).get("mode") or "").strip().lower()
        requested_owner = normalize_client_id((body or {}).get("client_id"))
        with STATE_LOCK:
            motion_state = str(SYSTEM["arm_control"].get("motion", {}).get("state") or "idle")
        if motion_state in {"accepted", "running"}:
            return command_result(False, "runtime arm motion is active; manual control remains locked")
        if requested_mode == "idle":
            _current_mode, current_owner = current_arm_authority()
            if current_owner and requested_owner != current_owner:
                return command_result(False, "only the active owner may release arm control")
            ok, msg = acquire_arm_authority("idle")
            return command_result(ok, "arm control released" if ok else msg, {"output": msg})
        mode = normalize_arm_mode(requested_mode)
        if mode is None:
            return command_result(False, f"unsupported arm control mode: {requested_mode}")
        if requested_owner is None:
            return command_result(False, "invalid client_id")
        ok, msg = acquire_arm_authority(mode, requested_owner, require_telemetry=True)
        return command_result(ok, f"arm control switched to {mode}" if ok else msg, {"output": msg})
    if path == "/api/arm/preset" and method == "POST":
        owner = normalize_client_id((body or {}).get("client_id"))
        if owner is None:
            return command_result(False, "invalid client_id")
        if not arm_telemetry_fresh():
            return command_result(False, "arm telemetry is stale; preset was not sent")
        degrees = (body or {}).get("degrees")
        name = str((body or {}).get("name") or "preset")
        ok, msg, command_ind = send_arm_preset_frame(degrees, name)
        if ok:
            update_arm_authority_state("preset", None)
            with STATE_LOCK:
                SYSTEM["arm_control"]["motion_companion_ok"] = True
                SYSTEM["arm_control"]["motion"] = {
                    "ind": command_ind, "source": "preset", "state": "accepted",
                    "progress": 0.0, "duration_sec": RUNTIME_MOTION_DURATION_SEC,
                }
            set_state("MOVING", f"arm preset {name}")
        return command_result(
            ok,
            f"arm preset accepted: {name}" if ok else msg,
            {
                "output": msg, "degrees": degrees, "command_ind": command_ind,
                "motion_state": "accepted" if ok else "failed",
                "duration_sec": RUNTIME_MOTION_DURATION_SEC,
            },
        )
    if path == "/api/system/reboot_board":
        ok, msg = ssh_run_command("sudo -n reboot", timeout=10)
        disconnected_for_reboot = "closed by remote host" in (msg or "").lower() or (
            "connection to" in (msg or "").lower() and "closed" in (msg or "").lower()
        )
        if ok or disconnected_for_reboot:
            set_module("jetson", "WARNING", "Jetson reboot command sent")
            set_module("arm", "OFFLINE", "Jetson is rebooting")
            set_module("udp_bridge", "OFFLINE", "Jetson is rebooting")
            set_state("OFFLINE", "Jetson reboot command sent")
            return command_result(True, "Jetson reboot command sent", {"output": msg})
        set_module("jetson", "ERROR", f"Jetson reboot failed: {msg}")
        return command_result(False, f"Jetson reboot failed: {msg}", {"output": msg})
    if path in ["/api/system/start", "/api/system/init"]:
        ok, msg = ping_host(jetson_cfg.get("host"))
        if not ok:
            set_module("jetson", "ERROR", "Jetson ping 不通", "JETSON_UNREACHABLE")
            set_module("network", "ERROR", "Jetson 网络不可达", "JETSON_UNREACHABLE")
            set_module("arm", "ERROR", "Jetson 离线，跳过远端机械臂节点启动", "JETSON_UNREACHABLE")
            set_module("udp_bridge", "ERROR", "Jetson 离线，跳过远端 UDP 桥接启动")
            jetson_start_msg = msg
            check_out = msg
            arm_ok = False
        else:
            set_module("jetson", "ONLINE", "Jetson ping 正常")
            set_module("network", "ONLINE", "网络链路可达")
            jetson_start_ok, jetson_start_msg = ssh_run("start_script")
            if not jetson_start_ok:
                set_module("arm", "ERROR", f"Jetson 节点启动失败: {jetson_start_msg}", "CAN_STOPPED")
                set_module("udp_bridge", "ERROR", "Jetson 节点启动失败")
                check_out = jetson_start_msg
                arm_ok = False
            else:
                ok_check, check_out = ssh_run("check_script")
                arm_ok = parse_arm_status(check_out) if ok_check else False
        if LOCAL_PROGRAM_DELAY_SEC > 0:
            log_event("INFO", f"等待 {LOCAL_PROGRAM_DELAY_SEC:g} 秒后启动本地手/手套程序")
            time.sleep(LOCAL_PROGRAM_DELAY_SEC)
        set_hand_mode("idle")
        hand_ok, _ = start_program("hand", "hand_exe")
        glove_ok, _ = start_program("glove", "unity_glove")
        if hand_ok:
            mode_ok, _ = send_hand_control("idle")
            hand_ok = hand_ok and mode_ok
        refresh_hand_link_status()
        set_module("matlab", "ONLINE", "USB 手柄由显控浏览器读取，主手/Simulink 已移入 Debug")
        acquire_arm_authority("idle")
        ready = arm_ok and hand_ok and glove_ok
        set_state("READY" if ready else "ERROR", "初始化/启动流程完成，部分模块可能失败")
        return command_result(
            ready,
            "初始化/启动流程完成",
            {"jetson_start": jetson_start_msg, "jetson_check": check_out},
        )
    if path == "/api/system/stop":
        acquire_arm_authority("idle")
        set_hand_mode("idle")
        ssh_run("stop_script")
        send_udp(MATLAB_HOST, MATLAB_PORT, "STOP")
        send_udp(HAND_HOST, HAND_PORT, "STOP")
        rtsp_ok, rtsp_msg = stop_rtsp_camera_stream()
        stop_program("glove")
        stop_program("hand")
        set_module("arm", "OFFLINE", "系统关闭，节点已停止")
        set_module("udp_bridge", "OFFLINE", "系统关闭，UDP 桥接已停止")
        set_module("matlab", "OFFLINE", "系统关闭，手柄输入停止")
        set_state("OFFLINE", "用户关闭系统")
        return command_result(True, "已发送系统停止命令", {"rtsp_stop_ok": rtsp_ok, "rtsp_stop_output": rtsp_msg})
    if path == "/api/system/estop":
        acquire_arm_authority("estop")
        arm_ok, arm_msg = send_udp(MATLAB_HOST, MATLAB_PORT, "STOP")
        hand_ok, hand_msg = send_udp(HAND_HOST, HAND_PORT, "STOP")
        set_module("arm", "WARNING" if arm_ok else "ERROR", "急停保持当前机械臂状态" if arm_ok else arm_msg)
        set_module("hand", "WARNING" if hand_ok else "ERROR", "急停保持当前灵巧手状态" if hand_ok else hand_msg)
        set_state("ESTOP", "急停保持当前状态，节点未关闭")
        return command_result(
            arm_ok and hand_ok,
            "急停已触发：机械臂和灵巧手保持当前状态，节点未关闭",
            {"arm_output": arm_msg, "hand_output": hand_msg},
        )
    if path == "/api/system/reset":
        with STATE_LOCK:
            SYSTEM["faults"].clear()
        arm_ok, arm_msg, command_ind = send_arm_home_frame()
        legacy_arm_ok, legacy_arm_msg = False, "disabled"
        if ARM_LEGACY_HOME_FALLBACK and not arm_ok:
            legacy_arm_ok, legacy_arm_msg = send_udp(MATLAB_HOST, MATLAB_PORT, "HOME")
        arm_ok = arm_ok or legacy_arm_ok
        if arm_ok:
            update_arm_authority_state("home", None)
        hand_ok, hand_msg = send_hand_control("preset", HAND_PRESETS["reset"], "reset")
        with STATE_LOCK:
            SYSTEM["arm_control"]["motion_companion_ok"] = bool(hand_ok)
            SYSTEM["arm_control"]["motion"] = {
                "ind": command_ind, "source": "home", "state": "accepted" if arm_ok else "failed",
                "progress": 0.0, "duration_sec": RUNTIME_MOTION_DURATION_SEC,
            }
        set_state("HOMING" if arm_ok and hand_ok else "ERROR", "arm homing accepted")
        return command_result(
            arm_ok and hand_ok,
            "已发送复位：机械臂回 0/0/0，灵巧手回 2000",
            {
                "arm_output": arm_msg, "legacy_arm_output": legacy_arm_msg,
                "hand_output": hand_msg, "hand_positions": HAND_PRESETS["reset"],
                "command_ind": command_ind, "motion_state": "accepted" if arm_ok else "failed",
                "duration_sec": RUNTIME_MOTION_DURATION_SEC,
            },
        )
    if path == "/api/arm/start_nodes":
        ok, msg = ssh_run("start_script")
        if ok:
            set_module("jetson", "ONLINE", "启动脚本执行成功")
        return command_result(ok, "Jetson 启动脚本已执行" if ok else f"启动失败: {msg}", {"output": msg})
    if path == "/api/arm/stop_nodes":
        ok, msg = ssh_run("stop_script")
        set_module("arm", "OFFLINE" if ok else "ERROR", "节点已停止" if ok else msg)
        if ok:
            set_module("udp_bridge", "OFFLINE", "UDP 桥接已停止")
        return command_result(ok, "Jetson 停止脚本已执行" if ok else f"停止失败: {msg}", {"output": msg})
    if path == "/api/arm/status":
        ok_ping, msg_ping = ping_host(jetson_cfg.get("host"))
        set_module("jetson", "ONLINE" if ok_ping else "ERROR", "Jetson ping 正常" if ok_ping else "Jetson ping 不通", None if ok_ping else "JETSON_UNREACHABLE")
        ok, msg = ssh_run("check_script") if ok_ping else (False, msg_ping)
        parsed_ok = parse_arm_status(msg) if ok else False
        return command_result(ok and parsed_ok, "机械臂状态已检查", {"ping": msg_ping, "output": msg})

    arm_cmds = {
        "/api/arm/preset_approach": "ARM_PRESET_APPROACH",
        "/api/arm/preset_grasp": "ARM_PRESET_GRASP",
        "/api/arm/preset_release": "ARM_PRESET_RELEASE",
        "/api/arm/stop": "ARM_STOP",
    }
    hand_cmds = {
        "/api/hand/open": "OPEN",
        "/api/hand/close": "CLOSE",
        "/api/hand/grasp_mode": "GRASP",
        "/api/hand/pinch": "PINCH",
        "/api/hand/hook": "HOOK",
        "/api/hand/stop": "STOP",
    }
    matlab_cmds = {"/api/matlab/start": "START", "/api/matlab/stop": "STOP", "/api/matlab/reset": "RESET"}
    demo_cmds = {
        "/api/demo/full": ("TASK_RUNNING", "FULL_DEMO"),
        "/api/demo/arm_only": ("TASK_RUNNING", "ARM_ONLY_DEMO"),
        "/api/demo/hand_only": ("TASK_RUNNING", "HAND_ONLY_DEMO"),
        "/api/demo/semi_auto": ("TASK_RUNNING", "SEMI_AUTO_DEMO"),
        "/api/demo/backup": ("READY", "BACKUP_DEMO"),
    }
    if path == "/api/hand/admittance-mode" and method == "GET":
        with STATE_LOCK:
            status = json.loads(json.dumps(SYSTEM.get("admittance_assist", {}), ensure_ascii=False))
        return {"ok": True, **status}
    if path == "/api/hand/admittance-mode" and method == "POST":
        requested = (body or {}).get("mode") if isinstance(body, dict) else None
        normalized = normalize_admittance_mode(requested)
        if normalized is None:
            return command_result(
                False,
                f"未知导纳模式: {requested!r}",
                {"supported": list(ADMITTANCE_MODE_NAMES.values())},
            )
        ok, msg = send_admittance_mode(normalized)
        return command_result(
            ok,
            f"导纳模式切换请求已发送: {ADMITTANCE_MODE_NAMES[normalized]}" if ok else f"导纳模式发送失败: {msg}",
            {"mode": normalized, "mode_name": ADMITTANCE_MODE_NAMES[normalized], "output": msg},
        )
    if path == "/api/semantic-admittance/mode" and method == "GET":
        return command_result(True, "semantic admittance mode", {
            "mode": SEMANTIC.mode, "semantic_admittance": {"mode": SEMANTIC.mode}
        })
    if path == "/api/semantic-admittance/mode" and method == "POST":
        requested = (body or {}).get("mode") if isinstance(body, dict) else None
        try:
            mode = SEMANTIC.set_mode(requested)
        except ValueError as exc:
            return command_result(False, str(exc), {"mode": SEMANTIC.mode})
        # Central correction and receiver-side admittance must never run twice.
        # OFF explicitly leaves the existing local policy available.
        if mode != "OFF":
            send_admittance_mode("off")
        push_status()
        return command_result(True, f"semantic admittance mode set to {mode}", {
            "mode": mode, "semantic_admittance": {"mode": mode}
        })
    if path == "/api/hand/mode/vr":
        ok, msg = activate_hand_mode("vr")
        return command_result(ok, "Switched to VR hand tracking mode" if ok else msg, {"output": msg})
    if path == "/api/hand/mode/glove":
        ok, msg = activate_hand_mode("glove")
        return command_result(ok, "已切换到手套操控模式" if ok else msg, {"output": msg})
    if path == "/api/hand/mode/preset":
        ok, msg = activate_hand_mode("preset")
        return command_result(ok, "已切换到预编程位置模式" if ok else msg, {"output": msg})
    if path.startswith("/api/hand/preset/"):
        preset_name = path.rsplit("/", 1)[-1]
        positions = HAND_PRESETS.get(preset_name)
        if positions is None:
            return command_result(False, f"未知预编程位置: {preset_name}")
        ok, msg = activate_hand_mode("preset", positions, preset_name)
        return command_result(ok, f"已发送预编程位置: {preset_name}" if ok else msg, {"positions": positions, "output": msg})
    if path == "/api/hand/position":
        try:
            positions = sanitize_hand_positions((body or {}).get("positions"))
        except Exception as exc:
            return command_result(False, str(exc))
        name = (body or {}).get("name") or "custom"
        ok, msg = activate_hand_mode("preset", positions, name)
        return command_result(ok, "已发送自定义预编程位置" if ok else msg, {"positions": positions, "output": msg})
    if path == "/api/hand/start_exe":
        ok, msg = start_program("hand", "hand_exe")
        idle_ok, idle_msg = send_hand_control("idle") if ok else (False, "hand start failed")
        return command_result(
            ok and idle_ok,
            "灵巧手程序已启动，idle 六路已强制回到 2000" if ok and idle_ok else (idle_msg if ok else msg),
            {"start_output": msg, "idle_output": idle_msg, "hand_positions": HAND_PRESETS["reset"]},
        )
    if path == "/api/hand/restart_exe":
        stop_ok, stop_msg = stop_configured_program("hand", "hand_exe")
        ok, msg = start_program("hand", "hand_exe")
        idle_ok, idle_msg = send_hand_control("idle") if ok else (False, "hand restart failed")
        return command_result(
            ok and idle_ok,
            "灵巧手 exe 已重启，idle 六路已强制回到 2000" if ok and idle_ok else (idle_msg if ok else f"灵巧手 exe 重启失败: {msg}"),
            {"stop_ok": stop_ok, "stop_output": stop_msg, "start_output": msg,
             "idle_output": idle_msg, "hand_positions": HAND_PRESETS["reset"]},
        )
    if path == "/api/glove/start":
        ok, msg = start_program("glove", "unity_glove")
        return command_result(ok, "Unity 数据手套程序已启动" if ok else msg)
    if path == "/api/hand/status":
        ok, msg = refresh_program_status("hand")
        return command_result(ok, msg)
    if path == "/api/glove/status":
        ok, msg = refresh_program_status("glove")
        return command_result(ok, msg)
    if path == "/api/hand/link_status":
        ok = refresh_hand_link_status()
        return command_result(ok, "本机手套链路已检查")
    if path in ["/api/matlab/status"]:
        return command_result(True, "状态已返回")
    if path in arm_cmds:
        ok, msg = send_udp(MATLAB_HOST, MATLAB_PORT, arm_cmds[path])
        return command_result(ok, f"已发送机械臂命令: {arm_cmds[path]}", {"output": msg})
    if path in hand_cmds:
        ok, msg = send_udp(HAND_HOST, HAND_PORT, hand_cmds[path])
        return command_result(ok, f"已发送灵巧手命令: {hand_cmds[path]}", {"output": msg})
    if path in matlab_cmds:
        ok, msg = send_udp(MATLAB_HOST, MATLAB_PORT, matlab_cmds[path])
        set_module("matlab", "ONLINE" if ok else "ERROR", "MATLAB UDP 命令已发送" if ok else msg, None if ok else "MATLAB_OFFLINE")
        return command_result(ok, f"已发送 MATLAB 命令: {matlab_cmds[path]}", {"output": msg})
    if path in demo_cmds:
        state, cmd = demo_cmds[path]
        set_state(state, cmd)
        send_udp(MATLAB_HOST, MATLAB_PORT, cmd)
        return command_result(True, f"演示模式已切换: {cmd}")
    return command_result(False, f"未知接口: {path}")


class ApiHandler(BaseHTTPRequestHandler):
    def _send_json(self, code, body):
        raw = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _send_bytes(self, code, raw, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store, no-cache, max-age=0")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_OPTIONS(self):
        self._send_json(200, {"ok": True})

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/camera/frame.jpg":
            frame, content_type, _timestamp, _seq = latest_camera_frame()
            if frame is None:
                self._send_json(503, command_result(False, "no camera frame has been received"))
                return
            self._send_bytes(200, frame, content_type)
            return
        if self.path == "/api/status" or self.path.startswith("/api/"):
            self._send_json(200, do_command("GET", self.path))
        else:
            self._send_json(404, command_result(False, f"未知接口: {self.path}"))

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/camera/frame":
            try:
                length = int(self.headers.get("Content-Length", "0") or 0)
                raw = self.rfile.read(length) if length > 0 else b""
            except Exception as exc:
                self._send_json(400, command_result(False, f"camera frame read failed: {exc}"))
                return
            self._send_json(200, store_camera_frame(raw, self.headers.get("Content-Type", "image/jpeg")))
            return
        body = None
        try:
            length = int(self.headers.get("Content-Length", "0") or 0)
            if length > 0:
                raw = self.rfile.read(length).decode("utf-8", errors="replace")
                body = json.loads(raw) if raw.strip() else None
        except Exception as exc:
            self._send_json(400, command_result(False, f"JSON 请求体解析失败: {exc}"))
            return
        self._send_json(200, do_command("POST", self.path, body))

    def log_message(self, fmt, *args):
        print(f"[HTTP] {self.address_string()} {fmt % args}")


class WsClient:
    def __init__(self, sock, path):
        self.sock = sock
        self.path = path
        self.lock = threading.Lock()
        self.alive = True

    def send_frame(self, opcode, payload):
        if not self.alive:
            return
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        header = bytearray([0x80 | opcode])
        length = len(payload)
        if length < 126:
            header.append(length)
        elif length <= 0xFFFF:
            header.extend([126])
            header.extend(struct.pack("!H", length))
        else:
            header.extend([127])
            header.extend(struct.pack("!Q", length))
        with self.lock:
            self.sock.sendall(bytes(header) + payload)

    def send_text(self, payload):
        self.send_frame(0x1, payload)

    def send_binary(self, payload):
        self.send_frame(0x2, payload)

    def close(self):
        self.alive = False
        with CLIENT_LOCK:
            UDP_CLIENTS.discard(self)
            STATUS_CLIENTS.discard(self)
            RAW_INPUT_CLIENTS.discard(self)
            CANONICAL_CLIENTS.discard(self)
            raw_connected = bool(RAW_INPUT_CLIENTS)
        RECORDING.set_browser_raw_connected(raw_connected)
        try:
            self.sock.close()
        except Exception:
            pass


def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data += chunk
    return data


def recv_ws_frame(sock):
    head = recv_exact(sock, 2)
    opcode = head[0] & 0x0F
    masked = bool(head[1] & 0x80)
    length = head[1] & 0x7F
    if length == 126:
        length = struct.unpack("!H", recv_exact(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", recv_exact(sock, 8))[0]
    mask = recv_exact(sock, 4) if masked else b"\x00\x00\x00\x00"
    payload = bytearray(recv_exact(sock, length))
    if masked:
        for i in range(length):
            payload[i] ^= mask[i % 4]
    return opcode, bytes(payload)


def _numeric_array(value, size):
    return isinstance(value, list) and len(value) == size and all(
        isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value
    )


def validate_raw_master_input(payload, browser_only=False):
    if not isinstance(payload, dict) or payload.get("type") != "raw_master_input":
        return False, "invalid_type"
    if int(payload.get("schema_version", 0)) != 1:
        return False, "invalid_schema"
    source = str(payload.get("source") or "")
    signal_form = str(payload.get("signal_form") or "")
    value = payload.get("value")
    if signal_form not in {"discrete", "continuous"} or not isinstance(value, dict):
        return False, "invalid_envelope"
    if browser_only and source not in {"keyboard", "gamepad"}:
        return False, "browser_source_not_allowed"

    stream = str(value.get("stream") or "")
    with STATE_LOCK:
        arm_mode = str(SYSTEM.get("arm_control", {}).get("mode") or "idle")
        hand_mode = str(SYSTEM.get("hand_mode") or "preset")
    if source == "keyboard":
        if arm_mode != "keyboard": return False, "inactive_source"
        if value.get("event") not in {"down", "up"} or not isinstance(value.get("pressed_keys"), list):
            return False, "invalid_keyboard_event"
    elif source == "gamepad":
        if arm_mode != "gamepad": return False, "inactive_source"
        if not isinstance(value.get("axes"), list) or not isinstance(value.get("buttons"), list):
            return False, "invalid_gamepad_sample"
    elif source == "pico_hand" and stream == "wrist_pose":
        if arm_mode != "hand_vision": return False, "inactive_source"
        if not _numeric_array(value.get("position"), 3) or not _numeric_array(value.get("rotation"), 4):
            return False, "invalid_pico_wrist_pose"
    elif source == "pico_hand" and stream == "hand_skeleton":
        if hand_mode != "vr": return False, "inactive_source"
        if not _numeric_array(value.get("rightPositions"), 63) or not _numeric_array(value.get("rightRotations"), 63):
            return False, "invalid_pico_hand_skeleton"
    elif source == "data_glove" and stream == "hand_skeleton":
        if hand_mode != "glove": return False, "inactive_source"
        if not _numeric_array(value.get("rightRotations"), 63):
            return False, "invalid_data_glove"
        if value.get("rightPositions") is not None and not _numeric_array(value.get("rightPositions"), 63):
            return False, "invalid_data_glove_positions"
    elif source == "vr_controller" and stream == "controller_pose":
        if arm_mode != "controller_delta": return False, "inactive_source"
        if not _numeric_array(value.get("position_m"), 3) or not _numeric_array(value.get("rotation_xyzw"), 4):
            return False, "invalid_vr_controller_pose"
    else:
        return False, "unsupported_source"
    return True, ""


def _canonical_semantic_bundle(goal, now_ns):
    """Join nominal intent, actual feedback and bounded correction by decision_seq."""
    bundle_started_ns = time.monotonic_ns()
    decision_seq = int(goal.get("seq") or 0)
    nominal = canonical_snapshot(
        "nominal", timestamp_ns=now_ns, decision_seq=decision_seq,
        wrist_pose=goal.get("wrist_pose_C"),
        hand_position_units=goal.get("hand_position_units"),
        hand_skeleton=goal.get("hand_skeleton_C"),
        wrist_mask=int((goal.get("validity") or {}).get("wrist_dof_mask", 0)),
        hand_mask=int((goal.get("validity") or {}).get("hand_dof_mask", 0)),
        source=f"{goal.get('wrist_source', '')}+{goal.get('hand_source', '')}",
        calibrations=goal.get("calibrations"), invalid_reasons=goal.get("invalid_reasons") or [],
    )
    with CANONICAL_MEASURED_LOCK:
        arm = dict(CANONICAL_MEASURED.get("arm") or {})
        hand = dict(CANONICAL_MEASURED.get("hand") or {})
        current = dict(CANONICAL_MEASURED.get("current") or {})
    measured_reasons = []
    wrist_pose, wrist_mask = None, 0
    try:
        if now_ns - int(arm.get("receive_utc_ns") or 0) <= SEMANTIC.limits.measured_timeout_sec * 1e9:
            wrist_pose = arm_forward_kinematics(arm.get("actual_q_rad"))
            wrist_mask = 0x3F
        else:
            measured_reasons.append("arm_encoder_stale")
    except (TypeError, ValueError):
        measured_reasons.append("arm_encoder_invalid")
    hand_units, hand_skeleton, hand_mask = None, None, 0
    try:
        if (
            now_ns - int(hand.get("receive_utc_ns") or 0) <= SEMANTIC.limits.measured_timeout_sec * 1e9
            and int(hand.get("position_valid_mask") or 0) == 0x3F
        ):
            # Actual WA100 feedback only; never substitute commanded positions.
            hand_units = [float(value) for value in hand.get("actual_position_units")]
            hand_skeleton = CANONICAL.hand_kinematics.canonical_skeleton_21(hand_units).tolist()
            hand_mask = 0x3F
        else:
            measured_reasons.append("wa100_actual_position_stale_or_invalid")
    except (TypeError, ValueError):
        measured_reasons.append("wa100_actual_position_invalid")
    measured = canonical_snapshot(
        "measured", timestamp_ns=now_ns, decision_seq=decision_seq,
        wrist_pose=wrist_pose, hand_position_units=hand_units, hand_skeleton=hand_skeleton,
        wrist_mask=wrist_mask, hand_mask=hand_mask, source="arm_encoder+wa100_feedback",
        calibrations={
            "arm_fk": "ui-arm-fk-l3-l5-0.424m-v1",
            "wa100_kinematics": CANONICAL.hand_kinematics.calibration_id,
            "wa100_kinematics_sha256": CANONICAL.hand_kinematics.content_sha256,
        }, invalid_reasons=measured_reasons,
    )
    measured["measurement_state"] = {
        "q_a_measured": arm.get("actual_q_rad"),
        "chi_h_measured": hand.get("actual_position_units"),
        "arm_source_time_ns": int(arm.get("source_time_ns") or 0),
        "hand_source_time_ns": int(hand.get("source_time_ns") or 0),
    }
    measured_done_ns = time.monotonic_ns()
    result = SEMANTIC.update(
        nominal, measured, current.get("residual"),
        current_timestamp_ns=int(current.get("timestamp_ns") or 0),
        now_ns=now_ns,
    )
    admittance_done_ns = time.monotonic_ns()
    corrected, audit = result["corrected"], result["audit"]
    nominal_targets = dict(goal.get("final_targets") or {})
    arm_targets = list(nominal_targets.get("arm_target_q_rad") or [])
    hand_targets = list(nominal_targets.get("hand_target_units") or [])
    planning = dict(goal.get("planning") or {})
    wrist_valid = int((goal.get("validity") or {}).get("wrist_dof_mask", 0)) != 0
    planned_pose = goal.get("wrist_pose_C") or {}
    try:
        quaternion = [float(value) for value in planned_pose.get("orientation_xyzw", [])]
        if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
            raise ValueError("planned wrist quaternion is invalid")
        x, y, z, w = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        arm_targets = arm_inverse_kinematics(planned_pose["position_m"], roll)
    except (KeyError, TypeError, ValueError):
        wrist_valid = False
        planning["valid_for_dispatch"] = False
        planning["ik_error"] = "planned_arm_ik_failed"
        goal.setdefault("invalid_reasons", []).append("planned_arm_ik_failed")
    if audit["dispatch_corrected"] and audit["mode"] == "ARM_HAND" and corrected.get("wrist_pose_C"):
        try:
            roll = arm_targets[3] if len(arm_targets) >= 4 else 0.0
            arm_targets = arm_inverse_kinematics(corrected["wrist_pose_C"]["position_m"], roll)
        except (TypeError, ValueError):
            audit["degradation_reasons"].append("corrected_arm_ik_failed")
    if audit["dispatch_corrected"]:
        corrected_targets = corrected.get("hand_position_units") or []
        if len(corrected_targets) == 6:
            hand_targets = list(corrected_targets)
    slave_command = {
        "type": "slave_kinematic_reference_cmd", "schema_version": 1,
        "timestamp_ns": now_ns, "decision_seq": decision_seq,
        "source": "canonical_corrected" if audit["dispatch_corrected"] else "canonical_nominal",
        "arm_target_q_rad": arm_targets, "hand_target_units": hand_targets,
        "valid_for_dispatch": bool(wrist_valid and planning.get("valid_for_dispatch", False)),
        "wrist_dof_mask": int((goal.get("validity") or {}).get("wrist_dof_mask", 0)),
        "wrist_source": str(goal.get("wrist_source") or ""),
    }
    audit["performance"] = {
        "measured_fk_ms": (measured_done_ns - bundle_started_ns) / 1e6,
        "admittance_ms": (admittance_done_ns - measured_done_ns) / 1e6,
        "ik_bundle_ms": (time.monotonic_ns() - admittance_done_ns) / 1e6,
    }
    envelope = dict(goal)
    published_hand = corrected if audit["dispatch_corrected"] else nominal
    envelope.update({
        "timestamp_ns": now_ns, "decision_seq": decision_seq,
        "canonical_nominal": nominal, "canonical_measured": measured,
        "canonical_corrected": corrected, "semantic_admittance": audit,
        "slave_kinematic_reference_cmd": slave_command,
        "wrist_pose_C": corrected["wrist_pose_C"] if audit["dispatch_corrected"] else nominal["wrist_pose_C"],
        "hand_position_units": published_hand["hand_position_units"],
        "hand_skeleton_C": published_hand["hand_skeleton_C"],
        "final_targets": slave_command, "planning": planning,
    })
    return envelope


def _publish_canonical_goal_unlocked(receive_utc_ns=None):
    global LATEST_SLAVE_COMMAND
    started_ns = time.monotonic_ns()
    with STATE_LOCK:
        arm_mode = str(SYSTEM.get("arm_control", {}).get("mode") or "idle")
        hand_mode = str(SYSTEM.get("hand_mode") or "idle")
    goal = CANONICAL.build(arm_mode, hand_mode, receive_utc_ns)
    built_ns = time.monotonic_ns()
    if goal is None:
        with STATE_LOCK:
            SYSTEM["canonical"] = CANONICAL.status()
        return None
    goal = _canonical_semantic_bundle(goal, int(receive_utc_ns or time.time_ns()))
    semantic_done_ns = time.monotonic_ns()
    with SLAVE_COMMAND_LOCK:
        LATEST_SLAVE_COMMAND = {
            **dict(goal.get("slave_kinematic_reference_cmd") or {}),
            "dispatch_corrected": bool((goal.get("semantic_admittance") or {}).get("dispatch_corrected")),
            "mode": str((goal.get("semantic_admittance") or {}).get("mode") or "OFF"),
            "no_send": bool(goal.get("no_send")),
        }
    goal["publish_rate_hz"] = CANONICAL_PUBLISH_HZ
    goal["publisher_performance"] = {
        "build_ms": (built_ns - started_ns) / 1e6,
        "semantic_and_dispatch_ms": (semantic_done_ns - built_ns) / 1e6,
        "critical_ms": (time.monotonic_ns() - started_ns) / 1e6,
    }
    record_started_ns = time.monotonic_ns()
    RECORDING.observe_canonical_goal(goal, receive_utc_ns or time.time_ns(), record_started_ns)
    record_done_ns = time.monotonic_ns()
    _publish_canonical_async(goal)
    with CANONICAL_PERF_LOCK:
        CANONICAL_PERF["publish_starts_ns"].append(started_ns)
        CANONICAL_PERF["critical_ms"].append((semantic_done_ns - started_ns) / 1e6)
        CANONICAL_PERF["build_ms"].append((built_ns - started_ns) / 1e6)
        CANONICAL_PERF["semantic_ms"].append((semantic_done_ns - built_ns) / 1e6)
        semantic_performance = (goal.get("semantic_admittance") or {}).get("performance") or {}
        for source_key, destination_key in (
            ("measured_fk_ms", "measured_fk_ms"),
            ("admittance_ms", "admittance_ms"),
            ("ik_bundle_ms", "ik_bundle_ms"),
        ):
            value = semantic_performance.get(source_key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                CANONICAL_PERF[destination_key].append(float(value))
        CANONICAL_PERF["record_enqueue_ms"].append((record_done_ns - record_started_ns) / 1e6)
    with STATE_LOCK:
        SYSTEM["canonical"] = {**CANONICAL.status(), "performance": canonical_performance_snapshot()}
    return goal


def _publish_canonical_async(goal):
    global CANONICAL_LATEST_ENVELOPE, CANONICAL_LATEST_VERSION
    with CANONICAL_ASYNC_CONDITION:
        CANONICAL_LATEST_ENVELOPE = goal
        CANONICAL_LATEST_VERSION += 1
        CANONICAL_ASYNC_CONDITION.notify_all()


def _wait_for_canonical(last_version):
    with CANONICAL_ASYNC_CONDITION:
        while CANONICAL_LATEST_VERSION == last_version:
            CANONICAL_ASYNC_CONDITION.wait()
        return CANONICAL_LATEST_VERSION, CANONICAL_LATEST_ENVELOPE


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    return float(ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))])


def canonical_performance_snapshot():
    with CANONICAL_PERF_LOCK:
        starts = list(CANONICAL_PERF["publish_starts_ns"])
        intervals = [(right - left) / 1e6 for left, right in zip(starts, starts[1:]) if right > left]
        rate_hz = ((len(starts) - 1) * 1e9 / (starts[-1] - starts[0])) if len(starts) > 1 else 0.0
        return {
            "target_hz": CANONICAL_PUBLISH_HZ,
            "measured_hz": rate_hz,
            "interval_p95_ms": _percentile(intervals, 0.95),
            "interval_p99_ms": _percentile(intervals, 0.99),
            "critical_p95_ms": _percentile(CANONICAL_PERF["critical_ms"], 0.95),
            "build_p95_ms": _percentile(CANONICAL_PERF["build_ms"], 0.95),
            "semantic_p95_ms": _percentile(CANONICAL_PERF["semantic_ms"], 0.95),
            "measured_fk_p95_ms": _percentile(CANONICAL_PERF["measured_fk_ms"], 0.95),
            "admittance_p95_ms": _percentile(CANONICAL_PERF["admittance_ms"], 0.95),
            "ik_bundle_p95_ms": _percentile(CANONICAL_PERF["ik_bundle_ms"], 0.95),
            "record_enqueue_p95_ms": _percentile(CANONICAL_PERF["record_enqueue_ms"], 0.95),
            **{key: int(CANONICAL_PERF[key]) for key in (
                "deadline_misses", "ws_sent", "ws_errors", "ws_overwritten",
                "hand_udp_sent", "hand_udp_errors", "hand_udp_overwritten",
            )},
        }


def canonical_websocket_broadcast_loop():
    last_version = 0
    while True:
        version, goal = _wait_for_canonical(last_version)
        skipped = max(0, version - last_version - 1)
        last_version = version
        encoded = json.dumps(goal, ensure_ascii=False, separators=(",", ":"))
        with CLIENT_LOCK:
            clients = list(CANONICAL_CLIENTS)
        sent = errors = 0
        for client in clients:
            try:
                client.send_text(encoded)
                sent += 1
            except Exception:
                errors += 1
                client.close()
        with CANONICAL_PERF_LOCK:
            CANONICAL_PERF["ws_sent"] += sent
            CANONICAL_PERF["ws_errors"] += errors
            CANONICAL_PERF["ws_overwritten"] += skipped


def canonical_hand_dispatch_loop():
    last_version = 0
    while True:
        version, goal = _wait_for_canonical(last_version)
        skipped = max(0, version - last_version - 1)
        last_version = version
        targets = (goal.get("slave_kinematic_reference_cmd") or {}).get("hand_target_units")
        hand_valid = int((goal.get("validity") or {}).get("hand_dof_mask", 0)) != 0
        sent = errors = 0
        if (
            CANONICAL_CONTROL_ENABLED and not bool(goal.get("no_send")) and hand_valid
            and isinstance(targets, list) and len(targets) == 6
        ):
            try:
                encoded = json.dumps(goal, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                CANONICAL_HAND_SOCKET.sendto(encoded, (CANONICAL_HAND_HOST, CANONICAL_HAND_PORT))
                sent = 1
            except OSError as exc:
                errors = 1
                with STATE_LOCK:
                    SYSTEM["canonical"]["last_error"] = f"canonical hand dispatch failed: {exc}"
        with CANONICAL_PERF_LOCK:
            CANONICAL_PERF["hand_udp_sent"] += sent
            CANONICAL_PERF["hand_udp_errors"] += errors
            CANONICAL_PERF["hand_udp_overwritten"] += skipped


def publish_canonical_goal(receive_utc_ns=None):
    # Wrist and hand adapters arrive on different threads. Keep seq allocation,
    # recording and publication in one order even when both update together.
    with CANONICAL_PUBLISH_LOCK:
        return _publish_canonical_goal_unlocked(receive_utc_ns)


def canonical_publish_loop():
    """Resample the latest asynchronous adapter states onto a fixed clock."""
    next_tick_ns = time.monotonic_ns()
    while True:
        publish_canonical_goal(time.time_ns())
        next_tick_ns += CANONICAL_PUBLISH_PERIOD_NS
        now_ns = time.monotonic_ns()
        if next_tick_ns <= now_ns:
            # Stay on the absolute clock. Skip whole missed slots without a
            # catch-up burst and expose the loss in performance telemetry.
            next_tick_ns, missed = advance_canonical_deadline(
                next_tick_ns, now_ns, CANONICAL_PUBLISH_PERIOD_NS
            )
            with CANONICAL_PERF_LOCK:
                CANONICAL_PERF["deadline_misses"] += missed
            now_ns = time.monotonic_ns()
        if next_tick_ns > now_ns:
            time.sleep((next_tick_ns - now_ns) / 1e9)


def advance_canonical_deadline(next_tick_ns, now_ns, period_ns):
    """Advance an overdue absolute deadline without emitting catch-up bursts."""
    period_ns = max(1, int(period_ns))
    next_tick_ns, now_ns = int(next_tick_ns), int(now_ns)
    if next_tick_ns > now_ns:
        return next_tick_ns, 0
    missed = int((now_ns - next_tick_ns) // period_ns) + 1
    return next_tick_ns + missed * period_ns, missed


def handle_canonical_wrist_adapter(payload, receive_utc_ns=None):
    receive_utc_ns = int(receive_utc_ns or time.time_ns())
    valid, reason = CANONICAL.observe_wrist_adapter(payload, receive_utc_ns)
    if not valid:
        with STATE_LOCK:
            SYSTEM["canonical"] = {**CANONICAL.status(), "last_error": reason}
        return False
    return True


def handle_raw_master_input(payload, browser_only=False, receive_utc_ns=None, receive_monotonic_ns=None):
    source = str(payload.get("source") or "unknown") if isinstance(payload, dict) else "unknown"
    valid, reason = validate_raw_master_input(payload, browser_only=browser_only)
    if not valid:
        RECORDING.reject_raw_input(source, reason)
        return False
    recorded = RECORDING.observe_raw_input(payload, receive_utc_ns, receive_monotonic_ns)
    value = payload.get("value") or {}
    if source in {"pico_hand", "data_glove"} and value.get("stream") == "hand_skeleton":
        ok, canonical_reason = CANONICAL.observe_hand_input(source, value, receive_utc_ns)
        if not ok:
            with STATE_LOCK:
                SYSTEM["canonical"] = {**CANONICAL.status(), "last_error": canonical_reason}
    return recorded


def apply_canonical_arm_reference(frame, decoded_frame=None):
    """Replace J1-J4 with the fresh planned Canonical command.

    Emergency stop is never delayed. Normal frames are suppressed when the
    Canonical command is missing or stale so raw adapter targets cannot bypass
    the planner.
    """
    if decoded_frame is not None and bool(getattr(decoded_frame, "emergency_stop", False)):
        return frame
    if not CANONICAL_CONTROL_ENABLED:
        return frame
    with SLAVE_COMMAND_LOCK:
        command = dict(LATEST_SLAVE_COMMAND or {})
    if (
        not command.get("valid_for_dispatch")
        or command.get("no_send")
        or time.time_ns() - int(command.get("timestamp_ns") or 0) > 100_000_000
    ):
        return None
    if decoded_frame is not None:
        expected_source = {
            "keyboard": "keyboard",
            "gamepad": "gamepad",
            "hand_vision": "pico_wrist",
            "controller_delta": "vr_controller",
        }.get(str(getattr(decoded_frame, "source", "") or ""))
        if expected_source is not None and command.get("wrist_source") != expected_source:
            return None
    targets = command.get("arm_target_q_rad")
    if not isinstance(targets, list) or len(targets) < 4 or len(frame) <= H5_FRAME_CRC_OFFSET + 3:
        return None
    try:
        values = [float(value) for value in targets[:4]]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    result = bytearray(frame)
    # frame_config.json: mode byte ends at 145; order[0] starts there.
    struct.pack_into("<4f", result, 145, *values)
    struct.pack_into("<I", result, H5_FRAME_CRC_OFFSET,
                     binascii.crc32(result[:H5_FRAME_CRC_OFFSET]) & 0xFFFFFFFF)
    return bytes(result)


def websocket_client_loop(client):
    try:
        if client.path == STATUS_PATH:
            client.send_text(json.dumps(snapshot(), ensure_ascii=False))
        while client.alive:
            opcode, payload = recv_ws_frame(client.sock)
            if opcode == 0x8:
                break
            if client.path == RAW_INPUT_PATH and opcode == 0x1:
                try:
                    raw_payload = json.loads(payload.decode("utf-8"))
                    if raw_payload.get("type") == "canonical_wrist_adapter":
                        handle_canonical_wrist_adapter(raw_payload)
                    else:
                        handle_raw_master_input(raw_payload, browser_only=True)
                except (UnicodeDecodeError, ValueError, TypeError):
                    RECORDING.reject_raw_input("browser", "invalid_json")
                continue
            if client.path == UDP_PATH and opcode == 0x2:
                active_mode, active_owner = current_arm_authority()
                decision = arbitrate_ws_frame(
                    payload,
                    active_mode,
                    active_owner,
                    arm_telemetry_fresh(),
                )
                record_arm_frame_decision(decision)
                if not decision.accepted:
                    continue
                if decision.frame is not None and decision.frame.emergency_stop:
                    acquire_arm_authority("estop")
                if LOOP_FILTER:
                    recent_ws_crc.append((binascii.crc32(payload) & 0xFFFFFFFF, time.time(), len(payload)))
                planned_payload = apply_canonical_arm_reference(payload, decision.frame)
                if planned_payload is not None:
                    udp.sendto(planned_payload, (UDP_HOST, UDP_SEND_PORT))
                # Per-frame output is intentionally disabled to avoid control-path jitter.
    except Exception as exc:
        print(f"[WS] client closed: {exc}")
    finally:
        client.close()


def websocket_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((WS_HOST, WS_PORT))
    srv.listen(20)
    print(
        f"[WS] bridge on ws://{WS_HOST}:{WS_PORT}{UDP_PATH}; "
        f"status ws://{WS_HOST}:{WS_PORT}{STATUS_PATH}; raw input ws://{WS_HOST}:{WS_PORT}{RAW_INPUT_PATH}; "
        f"canonical ws://{WS_HOST}:{WS_PORT}{CANONICAL_PATH}"
    )
    while True:
        sock, addr = srv.accept()
        try:
            request = sock.recv(4096).decode("utf-8", errors="ignore")
            lines = request.split("\r\n")
            path = lines[0].split(" ")[1] if lines and len(lines[0].split(" ")) >= 2 else "/"
            headers = {}
            for line in lines[1:]:
                if ":" in line:
                    k, v = line.split(":", 1)
                    headers[k.strip().lower()] = v.strip()
            if path not in [UDP_PATH, STATUS_PATH, RAW_INPUT_PATH, CANONICAL_PATH] or "sec-websocket-key" not in headers:
                body = f"Demo Console backend. Use {UDP_PATH}, {STATUS_PATH}, {RAW_INPUT_PATH}, or {CANONICAL_PATH}\n".encode("utf-8")
                sock.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nContent-Length: " + str(len(body)).encode() + b"\r\n\r\n" + body)
                sock.close()
                continue
            accept = base64.b64encode(hashlib.sha1((headers["sec-websocket-key"] + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
            )
            sock.sendall(response.encode("utf-8"))
            client = WsClient(sock, path)
            with CLIENT_LOCK:
                if path == STATUS_PATH:
                    STATUS_CLIENTS.add(client)
                elif path == RAW_INPUT_PATH:
                    RAW_INPUT_CLIENTS.add(client)
                    RECORDING.set_browser_raw_connected(True)
                elif path == CANONICAL_PATH:
                    CANONICAL_CLIENTS.add(client)
                else:
                    UDP_CLIENTS.add(client)
            print(f"[WS] client {addr} connected path={path}")
            threading.Thread(target=websocket_client_loop, args=(client,), daemon=True).start()
        except Exception as exc:
            print(f"[WS] handshake failed: {exc}")
            try:
                sock.close()
            except Exception:
                pass


def udp_broadcast_loop():
    global LAST_UDP_RESET_LOG
    print(f"[UDP] recv on 0.0.0.0:{UDP_RECV_PORT} -> WS; send to {UDP_HOST}:{UDP_SEND_PORT}")
    while True:
        try:
            data, _ = udp.recvfrom(65535)
        except OSError as exc:
            if isinstance(exc, ConnectionResetError) or getattr(exc, "winerror", None) == 10054:
                now = time.time()
                if now - LAST_UDP_RESET_LOG > 5:
                    LAST_UDP_RESET_LOG = now
                    log_event("WARNING", "UDP 远端端口暂不可达，遥测监听继续保持", exc)
                continue
            raise
        if data[:1] == b"{":
            try:
                payload = json.loads(data.decode("utf-8"))
                if payload.get("type") == "arm_recording_state" and int(payload.get("schema_version", 0)) == 1:
                    receive_utc_ns = time.time_ns()
                    with CANONICAL_MEASURED_LOCK:
                        CANONICAL_MEASURED["arm"] = {
                            "receive_utc_ns": receive_utc_ns,
                            "source_time_ns": int(payload.get("source_time_ns") or 0),
                            "actual_q_rad": payload.get("actual_q_rad"),
                        }
                    RECORDING.observe_arm(payload, receive_utc_ns, time.monotonic_ns())
                    continue
            except (UnicodeDecodeError, ValueError, TypeError):
                pass
        if LOOP_FILTER:
            crc = binascii.crc32(data) & 0xFFFFFFFF
            now = time.time()
            while recent_ws_crc and now - recent_ws_crc[0][1] > LOOP_FILTER_WINDOW:
                recent_ws_crc.popleft()
            if any((crc == c and len(data) == ln) for c, _t, ln in recent_ws_crc):
                continue
        update_arm_telemetry(data)
        set_module("network", "ONLINE", "收到 Jetson UDP 数据")
        set_module("udp_bridge", "ONLINE", "收到 H5 UDP 桥接心跳/遥测")
        with CLIENT_LOCK:
            clients = list(UDP_CLIENTS)
        for client in clients:
            try:
                client.send_binary(data)
            except Exception:
                client.close()


def _finite_six(value, default=None):
    """Return a 6-list of finite floats or the given default (schema lenient)."""
    if not isinstance(value, list) or len(value) != 6:
        return default
    try:
        values = [float(item) for item in value]
    except (TypeError, ValueError):
        return default
    return values if all(math.isfinite(item) for item in values) else default


def _bool_six(value, default=None):
    if not isinstance(value, list) or len(value) != 6:
        return default
    return [bool(item) for item in value]


def _observe_admittance_assist(payload, schema_version, receive_utc_ns, receive_monotonic_ns):
    """Maintain SYSTEM["admittance_assist"] from WA100 telemetry.

    Schema < 4 (or any invalid/missing field) fails open: the arm-side assist
    is treated as inactive so the raw operator target always passes through.
    """
    with STATE_LOCK:
        previous = SYSTEM.get("admittance_assist", {})
    mode_name = None
    retreat_active = False
    retreat_level = 0.0
    if schema_version >= 4:
        mode_name = str(payload.get("admittance_mode") or "")
        if mode_name not in set(ADMITTANCE_MODE_NAMES.values()):
            mode_name = None
        try:
            retreat_level = float(payload.get("retreat_level", 0.0))
        except (TypeError, ValueError):
            retreat_level = 0.0
        if not math.isfinite(retreat_level):
            retreat_level = 0.0
        retreat_active = bool(payload.get("retreat_active", False)) and mode_name is not None
        if mode_name in {"off", "handonly", "handconfig"}:
            # Receiver guarantees zeroed retreat outside Holistic; enforce here too.
            retreat_level = 0.0
            retreat_active = False
    age_sec = (receive_utc_ns - int(previous.get("received_utc_ns") or receive_utc_ns)) / 1e9
    # "fresh" means telemetry freshness only; retreat_active is reported
    # independently so a handonly/off session is not mislabelled as stale.
    fresh = (
        schema_version >= 4
        and mode_name is not None
        and age_sec <= ADMITTANCE_ASSIST_TIMEOUT_SEC
    )
    with STATE_LOCK:
        SYSTEM["admittance_assist"] = {
            "type": "admittance_assist_state",
            "schema_version": 1,
            "mode": mode_name or "unknown",
            "mode_raw": next((code for code, name in ADMITTANCE_MODE_NAMES.items() if name == mode_name), -1),
            "retreat_level": retreat_level,
            "retreat_active": bool(retreat_active),
            "closure": _finite_six(payload.get("admittance_closure"), [None] * 6),
            "off_hold_active": _bool_six(payload.get("off_hold_active"), [None] * 6),
            "gain_scale": _finite_six(payload.get("admittance_gain_scale"), [None] * 6),
            "trigger_on_ma": _finite_six(payload.get("admittance_trigger_on_ma"), [None] * 6),
            "trigger_off_ma": _finite_six(payload.get("admittance_trigger_off_ma"), [None] * 6),
            "load_evidence": _finite_six(payload.get("channel_load_evidence"), [None] * 6),
            "fresh": bool(fresh),
            "received_utc_ns": receive_utc_ns,
            "age_sec": age_sec,
        }


def hand_telemetry_loop():
    global PERCEPTION_LAST_PAYLOAD, PERCEPTION_LAST_RECEIVE_MONOTONIC_NS, PERCEPTION_LAST_STATUS_PUSH
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HAND_TELEMETRY_HOST, HAND_TELEMETRY_PORT))
    print(f"[UDP] hand recording telemetry on {HAND_TELEMETRY_HOST}:{HAND_TELEMETRY_PORT}")
    while True:
        data, _source = sock.recvfrom(65535)
        try:
            payload = json.loads(data.decode("utf-8"))
            schema_version = int(payload.get("schema_version", 0))
            if payload.get("type") != "wa100_recording_state" or schema_version not in {1, 2, 3, 4}:
                continue
            receive_utc_ns, receive_monotonic_ns = time.time_ns(), time.monotonic_ns()
            _observe_admittance_assist(payload, schema_version, receive_utc_ns, receive_monotonic_ns)
            decision = PERCEPTION_MONITOR.update(payload, receive_utc_ns, receive_monotonic_ns)
            position_valid = _bool_six(payload.get("position_valid"), [False] * 6)
            with CANONICAL_MEASURED_LOCK:
                CANONICAL_MEASURED["hand"] = {
                    "receive_utc_ns": receive_utc_ns,
                    "source_time_ns": int(payload.get("source_time_ns") or receive_utc_ns),
                    "actual_position_units": payload.get("actual_position_units"),
                    "position_valid_mask": sum(1 << index for index, valid in enumerate(position_valid) if valid),
                }
                CANONICAL_MEASURED["current"] = {
                    "timestamp_ns": receive_utc_ns,
                    "residual": decision.get("current_residual") if decision.get("valid") else None,
                }
            with STATE_LOCK:
                preset_state = dict(SYSTEM.get("hand_preset") or {})
            if preset_state:
                decision["preset"] = {
                    "name": str(preset_state.get("name") or ""),
                    "positions": list(preset_state.get("positions") or []),
                    "changed_at": preset_state.get("changed_at"),
                }
                if preset_state.get("changed"):
                    with STATE_LOCK:
                        SYSTEM["hand_preset"]["changed"] = False
                    events = list(decision.get("events") or [])
                    events.append(f"PRESET_CHANGED:{preset_state.get('name') or ''}")
                    decision["events"] = events
            PERCEPTION_LAST_PAYLOAD = payload
            PERCEPTION_LAST_RECEIVE_MONOTONIC_NS = receive_monotonic_ns
            RECORDING.observe_hand(payload, receive_utc_ns, receive_monotonic_ns)
            RECORDING.observe_episode(decision, receive_utc_ns, receive_monotonic_ns)
            with STATE_LOCK:
                old = SYSTEM.get("perception_assist", {})
                changed = old.get("state") != decision["state"] or old.get("valid") != decision["valid"]
                SYSTEM["perception_assist"] = decision
            encoded = json.dumps(decision, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            try:
                PERCEPTION_UNITY_SOCKET.sendto(encoded, PERCEPTION_UNITY_TARGET)
            except OSError:
                pass
            if decision.get("events"):
                level = "WARNING" if decision["state"] in {"EARLY_CONTACT", "OVERLOAD"} else "ASSIST"
                log_event(level, decision.get("prompt_message") or decision["state"], decision.get("reason"))
            elif changed or time.monotonic() - PERCEPTION_LAST_STATUS_PUSH >= 0.20:
                PERCEPTION_LAST_STATUS_PUSH = time.monotonic()
                push_status()
        except (UnicodeDecodeError, ValueError, TypeError):
            continue


def master_fusion_telemetry_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MASTER_FUSION_HOST, MASTER_FUSION_PORT))
    print(f"[UDP] master fusion telemetry on {MASTER_FUSION_HOST}:{MASTER_FUSION_PORT}")
    while True:
        data, _source = sock.recvfrom(65535)
        try:
            payload = json.loads(data.decode("utf-8"))
            if payload.get("type") != "master_fusion_state" or int(payload.get("schema_version", 0)) != 1:
                continue
            RECORDING.observe_master_fusion(payload, time.time_ns(), time.monotonic_ns())
        except (UnicodeDecodeError, ValueError, TypeError):
            continue


def raw_master_input_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((RAW_MASTER_HOST, RAW_MASTER_PORT))
    print(f"[UDP] raw master input on {RAW_MASTER_HOST}:{RAW_MASTER_PORT}")
    while True:
        data, _source = sock.recvfrom(65535)
        receive_utc_ns, receive_monotonic_ns = time.time_ns(), time.monotonic_ns()
        try:
            payload = json.loads(data.decode("utf-8"))
            handle_raw_master_input(payload, receive_utc_ns=receive_utc_ns, receive_monotonic_ns=receive_monotonic_ns)
        except (UnicodeDecodeError, ValueError, TypeError):
            RECORDING.reject_raw_input("udp", "invalid_json")


def heartbeat_loop():
    global PERCEPTION_LAST_STATUS_PUSH
    heartbeat_modules = {"network", "udp_bridge", "camera"}
    while True:
        now = time.time()
        changed = False
        expired_authority = None
        with STATE_LOCK:
            for name, mod in SYSTEM["modules"].items():
                if name not in heartbeat_modules or mod.get("status") in ["ERROR", "OFFLINE"]:
                    continue
                age = now - float(mod.get("last_seen") or 0)
                if age > 3 and mod["status"] != "OFFLINE":
                    mod["status"] = "OFFLINE"
                    mod["message"] = "超过 3 秒未收到心跳或状态刷新"
                    changed = True
                elif age > 1 and mod["status"] == "ONLINE":
                    mod["status"] = "WARNING"
                    mod["message"] = "超过 1 秒未收到心跳或状态刷新"
                    changed = True
        with STATE_LOCK:
            control = SYSTEM["arm_control"]
            control_mode = control.get("mode", "idle")
            last_telemetry = float(control.get("last_telemetry_time") or 0.0)
            if control_mode in SELECTABLE_MODES and (
                not last_telemetry or now - last_telemetry > ARM_TELEMETRY_TIMEOUT_SEC
            ):
                expired_authority = (control_mode, control.get("owner_id"))
                control["mode"] = "idle"
                control["owner_id"] = None
                control["owner_since"] = now
                control["last_reject_reason"] = "stale_telemetry"
                changed = True
        if expired_authority:
            send_arm_source_frame("idle")
            log_event("CONTROL", f"arm authority {expired_authority[0]} -> idle", "stale telemetry")
        if changed:
            push_status()
        if PERCEPTION_LAST_PAYLOAD and PERCEPTION_LAST_RECEIVE_MONOTONIC_NS:
            stale_after_ns = int(float(PERCEPTION_CONFIG.get("stale_timeout_sec", 0.25)) * 1e9)
            if time.monotonic_ns() - PERCEPTION_LAST_RECEIVE_MONOTONIC_NS > stale_after_ns:
                with STATE_LOCK:
                    was_valid = bool(SYSTEM.get("perception_assist", {}).get("valid"))
                if was_valid:
                    receive_utc_ns, receive_monotonic_ns = time.time_ns(), time.monotonic_ns()
                    decision = PERCEPTION_MONITOR.update(PERCEPTION_LAST_PAYLOAD, receive_utc_ns, receive_monotonic_ns)
                    RECORDING.observe_episode(decision, receive_utc_ns, receive_monotonic_ns)
                    with STATE_LOCK:
                        SYSTEM["perception_assist"] = decision
                    try:
                        PERCEPTION_UNITY_SOCKET.sendto(
                            json.dumps(decision, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                            PERCEPTION_UNITY_TARGET,
                        )
                    except OSError:
                        pass
                    log_event("WARNING", decision.get("prompt_message") or "抓持辅助判别数据过期", decision.get("invalid_reason"))
        try:
            watchdog_result = RECORDING.watchdog()
            if watchdog_result:
                log_event("ERROR", "recording stopped after modality loss", watchdog_result.get("error"))
        except Exception as exc:
            log_event("ERROR", "recording watchdog failed", exc)
        time.sleep(1)


def local_program_monitor_loop():
    while True:
        if "hand" in PROCESSES:
            refresh_program_status("hand")
        if "glove" in PROCESSES:
            refresh_program_status("glove")
        if "hand" in PROCESSES or "glove" in PROCESSES:
            refresh_hand_link_status()
        time.sleep(2)


def start_http_api():
    server = ThreadingHTTPServer((API_HOST, API_PORT), ApiHandler)
    print(f"[HTTP] REST API on http://{API_HOST}:{API_PORT}")
    server.serve_forever()


def main():
    threading.Thread(target=start_http_api, daemon=True).start()
    threading.Thread(target=websocket_server, daemon=True).start()
    threading.Thread(target=udp_broadcast_loop, daemon=True).start()
    threading.Thread(target=hand_telemetry_loop, daemon=True).start()
    threading.Thread(target=master_fusion_telemetry_loop, daemon=True).start()
    threading.Thread(target=raw_master_input_loop, daemon=True).start()
    threading.Thread(target=canonical_websocket_broadcast_loop, daemon=True,
                     name="canonical-ws-broadcast").start()
    threading.Thread(target=canonical_hand_dispatch_loop, daemon=True,
                     name="canonical-hand-dispatch").start()
    threading.Thread(target=canonical_publish_loop, daemon=True, name="canonical-50hz").start()
    threading.Thread(target=heartbeat_loop, daemon=True).start()
    threading.Thread(target=local_program_monitor_loop, daemon=True).start()
    acquire_arm_authority("idle")
    log_event("INFO", "Demo Console backend started")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
