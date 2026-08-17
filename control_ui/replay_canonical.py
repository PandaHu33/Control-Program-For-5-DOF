#!/usr/bin/env python3
"""Offline, no-send validation of an E1 canonical recording.

The tool never opens a control socket.  It reconstructs hand skeletons from
raw_input.jsonl and verifies wrist-delta/absolute-pose consistency in the
recorded canonical stream.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

try:
    from canonical_goal import (
        FrozenHi5HandModel,
        integrate_wrist_pose,
        pico_skeleton_in_wrist_frame,
        validate_canonical_goal,
    )
except ImportError:
    from control_ui.canonical_goal import (
        FrozenHi5HandModel,
        integrate_wrist_pose,
        pico_skeleton_in_wrist_frame,
        validate_canonical_goal,
    )


def _jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(path)
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path.name}:{number}: {exc}") from exc
    return rows


def _resolve_hand_model(canonical_rows: list[dict[str, Any]], requested: Path | None) -> Path:
    if requested is not None:
        return Path(requested)
    recorded_id = next((
        str((row.get("calibrations") or {}).get("hand_model") or "")
        for row in canonical_rows
        if (row.get("calibrations") or {}).get("hand_model")
    ), "")
    for filename in ("hi5_hand_model_v8.json", "hi5_hand_model_v7.json", "hi5_hand_model_v6.json", "hi5_hand_model_v5.json", "hi5_hand_model_v4.json", "hi5_hand_model_v3.json", "hi5_hand_model_v2.json", "hi5_hand_model_v1.json"):
        candidate = Path(__file__).with_name(filename)
        if candidate.exists() and (not recorded_id or FrozenHi5HandModel.load(candidate).calibration_id == recorded_id):
            return candidate
    raise ValueError(f"no local Hi5 model matches recorded calibration_id={recorded_id!r}")


def verify_session(session_dir: Path, hand_model_path: Path | None, tolerance: float = 1e-6) -> dict[str, Any]:
    session_dir = Path(session_dir).resolve()
    raw_rows = _jsonl(session_dir / "raw_input.jsonl")
    canonical_rows = _jsonl(session_dir / "canonical_goal.jsonl")
    hand_model_path = _resolve_hand_model(canonical_rows, hand_model_path)
    model = FrozenHi5HandModel.load(hand_model_path)
    errors = []
    recorded_seqs = [int(row.get("seq", 0)) for row in canonical_rows]
    unique_seqs = sorted(set(seq for seq in recorded_seqs if seq > 0))
    sequence_out_of_order = sum(
        current <= previous
        for previous, current in zip(recorded_seqs, recorded_seqs[1:])
    )
    sequence_duplicates = len(recorded_seqs) - len(set(recorded_seqs))
    sequence_drops = sum(
        max(0, current - previous - 1)
        for previous, current in zip(unique_seqs, unique_seqs[1:])
    )
    if sequence_out_of_order:
        errors.append(f"canonical stream out of order: {sequence_out_of_order}")
    if sequence_duplicates:
        errors.append(f"canonical duplicate seq: {sequence_duplicates}")
    if sequence_drops:
        errors.append(f"canonical missing seq: {sequence_drops}")
    raw_hands = {}
    for row in raw_rows:
        source, value = str(row.get("source") or ""), row.get("value") or {}
        if value.get("stream") != "hand_skeleton":
            continue
        seq = int(value.get("frameId", value.get("frame_id", -1)))
        raw_hands[(source, seq)] = value

    previous = None
    compared_hands = 0
    # Semantic verification follows canonical seq order. This prevents a file
    # write-order race from being misreported as a pose-integration error while
    # still failing the session explicitly through sequence_out_of_order.
    ordered_canonical_rows = sorted(canonical_rows, key=lambda row: int(row.get("seq", 0)))
    for index, row in enumerate(ordered_canonical_rows):
        valid, reason = validate_canonical_goal(row)
        if not valid:
            errors.append(f"canonical[{index}] {reason}")
            continue
        if previous is not None:
            current_wrist_seq = int((row.get("source_state") or {}).get("wrist_seq", -1))
            previous_wrist_seq = int((previous.get("source_state") or {}).get("wrist_seq", -1))
            same_wrist_update = (
                row.get("wrist_source") == previous.get("wrist_source")
                and current_wrist_seq == previous_wrist_seq
            )
            if same_wrist_update:
                expected_pose = previous["wrist_pose_C"]
            else:
                expected_pose = integrate_wrist_pose(previous["wrist_pose_C"], row["adapter_wrist_delta"])
            position_error = float(np.max(np.abs(
                np.asarray(expected_pose["position_m"]) - np.asarray(row["wrist_pose_C"]["position_m"])
            )))
            expected_q = np.asarray(expected_pose["orientation_xyzw"], dtype=float)
            actual_q = np.asarray(row["wrist_pose_C"]["orientation_xyzw"], dtype=float)
            quaternion_error = min(float(np.linalg.norm(expected_q - actual_q)), float(np.linalg.norm(expected_q + actual_q)))
            if position_error > tolerance or quaternion_error > tolerance:
                errors.append(
                    f"canonical[{index}] wrist integration mismatch "
                    f"position={position_error:.3g} quaternion={quaternion_error:.3g}"
                )
        previous = row

        source = str(row.get("hand_source") or "")
        seq = int((row.get("source_state") or {}).get("hand_seq", -1))
        raw = raw_hands.get((source, seq))
        if raw is None:
            continue
        try:
            reconstructed = (
                pico_skeleton_in_wrist_frame(raw.get("rightPositions"))
                if source == "pico_hand" else model.skeleton(
                    raw.get("rightRotations"), raw.get("mapped_target_units")
                )
            )
            skeleton_error = float(np.max(np.abs(reconstructed - np.asarray(row["hand_skeleton_w"], dtype=float))))
            compared_hands += 1
            if skeleton_error > tolerance:
                errors.append(f"canonical[{index}] hand skeleton mismatch {skeleton_error:.3g}")
        except (TypeError, ValueError) as exc:
            errors.append(f"canonical[{index}] hand replay failed: {exc}")

    return {
        "ok": not errors and bool(canonical_rows),
        "session_dir": str(session_dir),
        "raw_samples": len(raw_rows),
        "canonical_samples": len(canonical_rows),
        "sequence_drops": sequence_drops,
        "sequence_out_of_order": sequence_out_of_order,
        "sequence_duplicates": sequence_duplicates,
        "hand_samples_compared": compared_hands,
        "tolerance": tolerance,
        "hand_model": str(hand_model_path),
        "hand_model_calibration_id": model.calibration_id,
        "errors": errors,
        "no_send": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument(
        "--hand-model",
        type=Path,
        default=None,
        help="override model; by default select v1/v2/v3/v4/v5/v6/v7/v8 from the recording calibration_id",
    )
    parser.add_argument("--tolerance", type=float, default=1e-6)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = verify_session(args.session_dir, args.hand_model, max(0.0, args.tolerance))
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
