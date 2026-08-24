import tempfile
import time
from pathlib import Path

import numpy as np

from control_ui.recording_service import RecordingManager
from control_ui.semantic_admittance import (
    SemanticAdmittance, SemanticLimits, arm_forward_kinematics,
    arm_inverse_kinematics, canonical_snapshot, normalize_semantic_mode,
)
from control_ui.wa100_kinematics import WA100Kinematics


MODEL = Path(__file__).resolve().parents[1] / "control_ui" / "wa100_kinematics_model.json"


def _states(now):
    fk = WA100Kinematics.load(MODEL)
    units = [500.0] * 6
    hand = fk.canonical_skeleton_21(units).tolist()
    nominal = canonical_snapshot(
        "nominal", timestamp_ns=now, decision_seq=7,
        wrist_pose={"position_m": [0.6, 0.0, 0.1], "orientation_xyzw": [0, 0, 0, 1]},
        hand_position_units=units, hand_skeleton=hand, wrist_mask=0x3F, hand_mask=0x3F,
        source="test", calibrations={"wa100_kinematics": fk.calibration_id},
    )
    measured = canonical_snapshot(
        "measured", timestamp_ns=now, decision_seq=7,
        wrist_pose={"position_m": [0.59, 0.0, 0.1], "orientation_xyzw": [0, 0, 0, 1]},
        hand_position_units=units, hand_skeleton=hand, wrist_mask=0x3F, hand_mask=0x3F,
        source="feedback",
    )
    return fk, nominal, measured


def test_mode_contract_and_default_off_pass_through():
    assert [normalize_semantic_mode(v) for v in ("off", "shadow", "hand-only", "armhand")] == [
        "OFF", "SHADOW", "HAND_ONLY", "ARM_HAND"
    ]
    now = time.time_ns(); fk, nominal, measured = _states(now)
    result = SemanticAdmittance(hand_kinematics=fk).update(
        nominal, measured, [200.0] * 6, current_timestamp_ns=now, now_ns=now
    )
    assert result["audit"]["mode"] == "OFF"
    assert not result["audit"]["dispatch_corrected"]
    np.testing.assert_allclose(result["corrected"]["hand_position_units"], nominal["hand_position_units"])
    np.testing.assert_allclose(result["corrected"]["hand_skeleton_C"], nominal["hand_skeleton_C"])


def test_hand_only_corrects_units_toward_open_then_recomputes_fk():
    now = time.time_ns(); fk, nominal, measured = _states(now)
    layer = SemanticAdmittance(SemanticLimits(trigger_ma=20.0), "HAND_ONLY", fk)
    result = layer.update(nominal, measured, [100.0] * 6, current_timestamp_ns=now, now_ns=now)
    corrected_units = np.asarray(result["corrected"]["hand_position_units"])
    assert result["audit"]["dispatch_corrected"]
    assert np.all(corrected_units > np.asarray(nominal["hand_position_units"]))
    np.testing.assert_allclose(
        result["corrected"]["hand_skeleton_C"], fk.canonical_skeleton_21(corrected_units), atol=1e-12
    )


def test_missing_or_stale_feedback_clears_dispatch():
    now = time.time_ns(); fk, nominal, measured = _states(now)
    for broken, residual, residual_time in (
        (None, [100.0] * 6, now),
        ({**measured, "validity": {"wrist_dof_mask": 0, "hand_dof_mask": 0}}, [100.0] * 6, now),
        (measured, None, now),
        (measured, [100.0] * 6, now - 1_000_000_000),
    ):
        result = SemanticAdmittance(SemanticLimits(trigger_ma=20.0), "ARM_HAND", fk).update(
            nominal, broken, residual, current_timestamp_ns=residual_time, now_ns=now
        )
        assert not result["audit"]["dispatch_corrected"]
        assert result["audit"]["delta_wrist_C"] == [0.0] * 6


def test_arm_fk_ik_contract_round_trip():
    q = [0.2, 0.8, 0.5, -0.3, 0.0]
    pose = arm_forward_kinematics(q)
    recovered = arm_inverse_kinematics(pose["position_m"], q[3])
    pose2 = arm_forward_kinematics([*recovered, 0.0])
    np.testing.assert_allclose(pose2["position_m"], pose["position_m"], atol=1e-9)


def test_manifest_summary_counts_v4_semantic_streams():
    with tempfile.TemporaryDirectory() as directory:
        manager = RecordingManager(Path(directory), min_free_bytes=0)
        manager._canonical_rows = [{
            "schema_version": 4, "seq": 1,
            "validity": {"wrist_dof_mask": 0x3F, "hand_dof_mask": 0x3F},
            "canonical_nominal": {}, "canonical_measured": {}, "canonical_corrected": {},
            "semantic_admittance": {"mode": "SHADOW"}, "slave_kinematic_reference_cmd": {},
        }]
        summary = manager._canonical_summary()
        assert summary["semantic_missing"] == []
        assert summary["software_version"] == "wa100-canonical-v4"
