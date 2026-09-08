import importlib.util
import statistics
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_PATH = ROOT / "Hand_Tracker/unity_hand_udp_bridge.py"
CPP_PATH = ROOT / "wa100-sdk-publish/examples/udp_receiver_unity.cpp"


def load_bridge_module():
    spec = importlib.util.spec_from_file_location("unity_hand_udp_bridge_filter_test", BRIDGE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


BRIDGE = load_bridge_module()
BRIDGE_SOURCE = BRIDGE_PATH.read_text(encoding="utf-8")
CPP_SOURCE = CPP_PATH.read_text(encoding="utf-8")


class VrHandFilterContractTests(unittest.TestCase):
    def test_filter_runs_after_xr_validation_and_before_forwarding(self):
        handler_begin = BRIDGE_SOURCE.index("    def _handle_line")
        handler_end = BRIDGE_SOURCE.index("    def _filter_xr_skeletons", handler_begin)
        handler = BRIDGE_SOURCE[handler_begin:handler_end]

        self.assertLess(handler.index("validate_packet(payload)"), handler.index("self._filter_xr_skeletons"))
        self.assertLess(handler.index("self._filter_xr_skeletons"), handler.index("json.dumps(payload"))
        self.assertLess(handler.index("self._filter_xr_skeletons"), handler.index("classify_left_fist"))

    def test_post_retarget_vr_filter_is_removed(self):
        self.assertNotIn("VrHandTargetOneEuroFilter", CPP_SOURCE)
        self.assertNotIn("g_vr_hand_target_filter", CPP_SOURCE)
        self.assertIn("if ((ControlMode)g_control_mode.load() == ControlMode::Vr)", CPP_SOURCE)
        self.assertIn("shaped_desired[i] = desired_pos[i];", CPP_SOURCE)

    def test_stationary_noise_is_reduced_at_skeleton_stage(self):
        skeleton_filter = BRIDGE.XrSkeletonOneEuroFilter()
        raw = []
        filtered = []
        for index in range(200):
            value = 0.01 if index % 2 else -0.01
            sample = [0.0] * BRIDGE.EXPECTED_FLOAT_COUNT
            sample[0] = value
            raw.append(value)
            filtered.append(skeleton_filter.apply(sample, now=index * 0.02)[0])

        self.assertLess(statistics.pstdev(filtered[100:]), statistics.pstdev(raw[100:]) * 0.5)

    def test_tracking_gap_reanchors_without_stale_drag(self):
        skeleton_filter = BRIDGE.XrSkeletonOneEuroFilter()
        first = [0.0] * BRIDGE.EXPECTED_FLOAT_COUNT
        moved = [0.0] * BRIDGE.EXPECTED_FLOAT_COUNT
        moved[0] = 0.2
        skeleton_filter.apply(first, now=0.0)
        smoothed = skeleton_filter.apply(moved, now=0.02)
        reanchored = skeleton_filter.apply(moved, now=0.50)

        self.assertLess(smoothed[0], moved[0])
        self.assertEqual(reanchored, moved)


if __name__ == "__main__":
    unittest.main()
