#!/usr/bin/env python3
"""Offline validation for WA100 Canonical v4 and read-only legacy playback."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

try:
    from .canonical_goal import validate_canonical_goal
    from .wa100_kinematics import WA100Kinematics
except ImportError:
    from canonical_goal import validate_canonical_goal
    from wa100_kinematics import WA100Kinematics


def _jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path.name}:{number} is not an object")
        rows.append(value)
    return rows


def _legacy_skeleton(row: dict[str, Any]) -> np.ndarray:
    value = row.get("hand_skeleton_C", row.get("hand_skeleton_w"))
    result = np.asarray(value, dtype=float)
    if result.shape != (21, 3) or not np.all(np.isfinite(result)):
        raise ValueError("legacy record has no readable 21-point skeleton")
    return result


def verify_session(session_dir: Path, kinematics_model_path: Path | None = None,
                   tolerance: float = 1e-6, hand_rig_path: Path | None = None) -> dict[str, Any]:
    """Verify v4 FK exactly; schema 1-3 rows remain display-only readable.

    ``hand_rig_path`` remains in the Python signature solely so old scripts do
    not crash.  Human-rig reconstruction is intentionally no longer executed.
    """
    session_dir = Path(session_dir)
    rows = _jsonl(session_dir / "canonical_goal.jsonl")
    model_path = Path(kinematics_model_path) if kinematics_model_path else Path(__file__).with_name("wa100_kinematics_model.json")
    model = WA100Kinematics.load(model_path)
    errors: list[str] = []
    v4_count = legacy_count = fk_checked = 0
    seqs: list[int] = []
    max_fk_error = 0.0
    for index, row in enumerate(rows):
        sequence = int(row.get("seq", 0))
        if sequence > 0:
            seqs.append(sequence)
        schema = int(row.get("schema_version", 0))
        if schema < 4:
            legacy_count += 1
            try:
                _legacy_skeleton(row)
            except ValueError as exc:
                errors.append(f"canonical[{index}] {exc}")
            continue
        v4_count += 1
        valid, reason = validate_canonical_goal(row)
        if not valid:
            errors.append(f"canonical[{index}] {reason}")
            continue
        recorded_hash = str((row.get("calibrations") or {}).get("wa100_kinematics_sha256") or "")
        if recorded_hash != model.content_sha256:
            errors.append(f"canonical[{index}] WA100 model sha256 mismatch")
            continue
        for layer_name in ("canonical_nominal", "canonical_measured", "canonical_corrected"):
            layer = row.get(layer_name)
            if not isinstance(layer, dict):
                errors.append(f"canonical[{index}] missing {layer_name}")
                continue
            units, skeleton = layer.get("hand_position_units"), layer.get("hand_skeleton_C")
            if units is None and skeleton is None:
                continue
            try:
                reconstructed = model.canonical_skeleton_21(units)
                recorded = np.asarray(skeleton, dtype=float).reshape(21, 3)
                error = float(np.max(np.abs(reconstructed - recorded)))
                max_fk_error = max(max_fk_error, error)
                fk_checked += 1
                if error > tolerance:
                    errors.append(f"canonical[{index}] {layer_name} FK mismatch {error:.3g}")
            except (TypeError, ValueError) as exc:
                errors.append(f"canonical[{index}] {layer_name} invalid: {exc}")
    unique = sorted(set(seqs))
    drops = sum(max(0, current - previous - 1) for previous, current in zip(unique, unique[1:]))
    duplicates = len(seqs) - len(unique)
    out_of_order = sum(current <= previous for previous, current in zip(seqs, seqs[1:]))
    if drops:
        errors.append(f"canonical missing seq: {drops}")
    if duplicates:
        errors.append(f"canonical duplicate seq: {duplicates}")
    if out_of_order:
        errors.append(f"canonical stream out of order: {out_of_order}")
    return {
        "ok": bool(rows) and not errors,
        "errors": errors,
        "canonical_samples": len(rows),
        "schema_v4_samples": v4_count,
        "legacy_read_only_samples": legacy_count,
        "fk_checked_layers": fk_checked,
        "max_fk_error": max_fk_error,
        "sequence_drops": drops,
        "sequence_duplicates": duplicates,
        "sequence_out_of_order": out_of_order,
        "wa100_kinematics": str(model_path.resolve()),
        "wa100_kinematics_id": model.calibration_id,
        "wa100_kinematics_sha256": model.content_sha256,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--kinematics-model", type=Path)
    parser.add_argument("--hand-model", dest="kinematics_model", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--hand-rig", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = verify_session(args.session_dir, args.kinematics_model, max(0.0, args.tolerance), args.hand_rig)
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
