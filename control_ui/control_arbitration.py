"""Pure helpers for validating and arbitrating H5 arm command frames."""

from __future__ import annotations

import binascii
import re
import struct
from dataclasses import dataclass
from typing import Optional


H5_FRAME_SIZE = 293
H5_MODE_OFFSET = 144
H5_ORDER_OFFSET = 145
H5_RESERVED_OFFSET = 209
H5_RESERVED_SIZE = 16
H5_NOTE_OFFSET = 225
H5_NOTE_SIZE = 64
H5_CRC_OFFSET = 289

IDLE_MODE = "idle"
SYSTEM_MODES = frozenset({"idle", "home", "preset", "estop"})
LOCAL_UI_MODES = frozenset({"keyboard", "gamepad", "controller_delta", "hand_vision"})
EXTERNAL_MODES = frozenset({"teleop", "imitation"})
SELECTABLE_MODES = LOCAL_UI_MODES | EXTERNAL_MODES
ALL_MODES = SELECTABLE_MODES | SYSTEM_MODES

_CLIENT_ID_RE = re.compile(r"^[0-9a-f]{32}$")


class ArmFrameError(ValueError):
    """Raised when a command frame is malformed or fails its CRC."""


@dataclass(frozen=True)
class ArmFrameInfo:
    mode: int
    selector: int
    emergency_stop: bool
    note: str
    source: Optional[str]
    owner_id: str
    order: tuple


@dataclass(frozen=True)
class ArbitrationDecision:
    accepted: bool
    reason: str
    frame: Optional[ArmFrameInfo] = None


def normalize_arm_mode(value: object, allow_system: bool = False) -> Optional[str]:
    mode = str(value or "").strip().lower()
    allowed = ALL_MODES if allow_system else SELECTABLE_MODES
    return mode if mode in allowed else None


def normalize_client_id(value: object) -> Optional[str]:
    text = str(value or "").strip().lower().replace("-", "")
    if not _CLIENT_ID_RE.fullmatch(text):
        return None
    if text in {"0" * 32, "f" * 32}:
        return None
    return text


def client_id_bytes(value: object) -> bytes:
    client_id = normalize_client_id(value)
    if client_id is None:
        raise ValueError("client_id must be a non-zero 16-byte hexadecimal identifier")
    return bytes.fromhex(client_id)


def frame_source_from_note(note: str) -> Optional[str]:
    if not note.startswith("ui:"):
        return None
    source = note[3:].strip().lower()
    if source == "arm_preset":
        return "preset"
    return source if source in SELECTABLE_MODES else None


def parse_arm_frame(payload: bytes) -> ArmFrameInfo:
    if len(payload) != H5_FRAME_SIZE:
        raise ArmFrameError("bad_length")

    expected_crc = struct.unpack_from("<I", payload, H5_CRC_OFFSET)[0]
    actual_crc = binascii.crc32(payload[:H5_CRC_OFFSET]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise ArmFrameError("bad_crc")

    mode = struct.unpack_from("<B", payload, H5_MODE_OFFSET)[0]
    note_raw = payload[H5_NOTE_OFFSET:H5_NOTE_OFFSET + H5_NOTE_SIZE].split(b"\x00", 1)[0]
    note = note_raw.decode("utf-8", "ignore")
    owner = payload[H5_RESERVED_OFFSET:H5_RESERVED_OFFSET + H5_RESERVED_SIZE].hex()
    order = struct.unpack_from("<16f", payload, H5_ORDER_OFFSET)
    return ArmFrameInfo(
        mode=mode,
        selector=mode & 0x0F,
        emergency_stop=bool(mode & 0x80),
        note=note,
        source=frame_source_from_note(note),
        owner_id=owner,
        order=order,
    )


def arbitrate_ws_frame(
    payload: bytes,
    active_mode: str,
    active_owner_id: Optional[str],
    telemetry_fresh: bool,
) -> ArbitrationDecision:
    try:
        frame = parse_arm_frame(payload)
    except ArmFrameError as exc:
        return ArbitrationDecision(False, str(exc))

    if frame.emergency_stop:
        return ArbitrationDecision(True, "emergency_stop", frame)
    if not telemetry_fresh:
        return ArbitrationDecision(False, "stale_telemetry", frame)
    if frame.source is None:
        return ArbitrationDecision(False, "unknown_source", frame)
    if frame.source != active_mode:
        return ArbitrationDecision(False, "inactive_source", frame)
    if normalize_client_id(active_owner_id) != frame.owner_id:
        return ArbitrationDecision(False, "wrong_owner", frame)
    if frame.source not in LOCAL_UI_MODES:
        return ArbitrationDecision(False, "external_source_has_no_ui_motion_frame", frame)
    return ArbitrationDecision(True, "accepted", frame)
