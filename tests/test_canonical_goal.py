import math
import time
from pathlib import Path

import numpy as np

from control_ui.canonical_goal import (
    HAND_ALL_VALID_MASK,
    WRIST_SUPPORTED_MASK,
    CanonicalGoalBuilder,
    integrate_wrist_pose,
    validate_canonical_goal,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "control_ui" / "wa100_kinematics_model.json"


def wrist_payload(now, source="vr_controller"):
    return {
        "type": "canonical_wrist_adapter", "schema_version": 1,
        "wrist_source": source, "source_seq": 3, "source_time_ns": now,
        "adapter_wrist_delta": [0.01, 0, 0, 0, 0, 0],
        "wrist_pose_C": {"position_m": [0.5, 0, 0.1], "orientation_xyzw": [0, 0, 0, -1]},
        "arm_target_q_rad": [0.1, 0.2, 0.3, 0.4],
        "wrist_dof_mask": WRIST_SUPPORTED_MASK, "mapping_id": "controller-test",
    }


def test_world_translation_and_body_rotation_integrate_consistently():
    pose = {"position_m": [0.1, 0.2, 0.3], "orientation_xyzw": [0, 0, 0, 1]}
    updated = integrate_wrist_pose(pose, [0.01, -0.02, 0.03, math.pi / 2, 0, 0])
    np.testing.assert_allclose(updated["position_m"], [0.11, 0.18, 0.33], atol=1e-12)
    np.testing.assert_allclose(updated["orientation_xyzw"], [math.sqrt(0.5), 0, 0, math.sqrt(0.5)], atol=1e-12)


def test_three_master_sources_with_same_units_have_identical_fk():
    units = [300, 700, 250, 800, 1200, 1600]
    skeletons = []
    for source in ("pico_hand", "data_glove", "preset_hand"):
        builder = CanonicalGoalBuilder(MODEL)
        now = time.time_ns()
        assert builder.observe_wrist_adapter(wrist_payload(now), now)[0]
        value = {"mapped_target_units": units, "frameId": 9}
        assert builder.observe_hand_input(source, value, now)[0]
        goal = builder.build("controller_delta", "preset" if source == "preset_hand" else "glove", now)
        assert goal["schema_version"] == 4
        assert goal["validity"]["hand_dof_mask"] == HAND_ALL_VALID_MASK
        assert goal["hand_position_units"] == units
        assert goal["final_targets"]["hand_target_units"] == units
        assert "hand_pose_C" not in goal and "hand_skeleton_w" not in goal
        assert "hand" not in goal["raw_intent"]
        assert validate_canonical_goal(goal) == (True, "")
        skeletons.append(goal["hand_skeleton_C"])
    np.testing.assert_allclose(skeletons[0], skeletons[1], atol=1e-12)
    np.testing.assert_allclose(skeletons[0], skeletons[2], atol=1e-12)


def test_invalid_six_dof_input_does_not_replace_last_valid_state():
    builder = CanonicalGoalBuilder(MODEL)
    now = time.time_ns()
    assert builder.observe_hand_input("data_glove", {"mapped_target_units": [1000] * 6}, now)[0]
    before = dict(builder.hand)
    ok, reason = builder.observe_hand_input("pico_hand", {"mapped_target_units": [1000] * 5}, now + 1)
    assert not ok and "six" in reason
    assert builder.hand["target_units"] == before["target_units"]


def test_preset_state_does_not_expire_but_continuous_input_does():
    now = time.time_ns()
    preset = CanonicalGoalBuilder(MODEL, stale_timeout_sec=0.01)
    preset.observe_wrist_adapter(wrist_payload(now, "keyboard"), now)
    preset.observe_hand_input("preset_hand", {"positions": [1000] * 6}, now)
    preset_goal = preset.build("keyboard", "preset", now + 20_000_000)
    assert preset_goal["validity"]["hand_dof_mask"] == 0x3F

    glove = CanonicalGoalBuilder(MODEL, stale_timeout_sec=0.01)
    glove.observe_wrist_adapter(wrist_payload(now, "keyboard"), now)
    glove.observe_hand_input("data_glove", {"mapped_target_units": [1000] * 6}, now)
    glove_goal = glove.build("keyboard", "glove", now + 20_000_000)
    assert glove_goal["validity"]["hand_dof_mask"] == 0
    assert "hand_stale" in glove_goal["invalid_reasons"]
