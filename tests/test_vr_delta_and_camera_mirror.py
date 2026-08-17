import copy
import importlib.util
import json
import math
import pathlib
import sys
import threading
import time
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


sys.path.insert(0, str(ROOT / "Hand_Tracker"))
HAND = load_module(ROOT / "Hand_Tracker" / "hand_arm_control.py", "hand_arm_control_delta_test")
UNITY_HAND = load_module(ROOT / "Hand_Tracker" / "unity_hand_udp_bridge.py", "unity_hand_bridge_gesture_test")

try:
    CAMERA = load_module(
        ROOT / "control_ui" / "latest_frame_dual_stream.py",
        "latest_frame_dual_stream_mirror_test",
    )
except ModuleNotFoundError:
    CAMERA = None


@unittest.skipIf(HAND.np is None, "numpy is required for VR delta tests")
class VrWristDeltaTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(HAND.DEFAULT_CONFIG)
        self.config["controller_delta"]["gain_xyz"] = [0.5, 0.5, 0.5]
        self.settings = HAND.build_settings(self.config)
        self.wrist = HAND.VRWristAxisController(self.config, self.settings)
        self.controller = HAND.ControllerDeltaAxisController(self.config)

    @staticmethod
    def pose(position, deadman=True, tracked=True, source="xr_hand_wrist", age=0.0):
        return {
            "seq": 1,
            "device_time": 1.0,
            "source": source,
            "tracked": tracked,
            "deadman_active": deadman,
            "position": HAND.np.array(position, dtype=float),
            "rotation": HAND.np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
            "received_at": time.monotonic() - age,
            "wall_time": time.time(),
        }

    def test_deadman_anchor_move_release_and_reanchor(self):
        waiting = self.wrist.update(self.pose([1.0, 2.0, 3.0], deadman=False), {})
        self.assertFalse(waiting["control_active"])
        self.assertEqual(waiting["session_id"], 0)

        anchor = self.wrist.update(self.pose([1.0, 2.0, 3.0]), {})
        self.assertTrue(anchor["control_active"])
        self.assertEqual(anchor["session_id"], 1)
        self.assertEqual(anchor["position_delta"], {"x": 0.0, "y": 0.0, "z": 0.0})

        moved = self.wrist.update(self.pose([1.1, 2.2, 3.3]), {})
        self.assertAlmostEqual(moved["position_delta"]["x"], 0.15)
        self.assertAlmostEqual(moved["position_delta"]["y"], 0.05)
        self.assertAlmostEqual(moved["position_delta"]["z"], 0.10)

        stationary = self.wrist.update(self.pose([1.1, 2.2, 3.3]), {})
        self.assertEqual(stationary["position_delta"], moved["position_delta"])

        released = self.wrist.update(self.pose([1.1, 2.2, 3.3], deadman=False), {})
        self.assertFalse(released["control_active"])
        self.assertEqual(released["position_delta"], {"x": 0.0, "y": 0.0, "z": 0.0})

        reanchored = self.wrist.update(self.pose([4.0, 5.0, 6.0]), {})
        self.assertTrue(reanchored["control_active"])
        self.assertEqual(reanchored["session_id"], 2)
        self.assertEqual(reanchored["position_delta"], {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_hand_and_controller_use_identical_axis_mapping_and_gain(self):
        wrist_anchor = self.pose([0.0, 0.0, 0.0])
        controller_anchor = self.pose([0.0, 0.0, 0.0], source="right_controller")
        self.wrist.update(wrist_anchor, {})
        self.controller.update(controller_anchor, {})

        wrist_result = self.wrist.update(self.pose([0.2, -0.4, 0.6]), {})
        controller_result = self.controller.update(
            self.pose([0.2, -0.4, 0.6], source="right_controller"),
            {},
        )
        self.assertEqual(wrist_result["mapping"]["gain_xyz"], [0.5, 0.5, 0.5])
        self.assertEqual(wrist_result["position_delta"], controller_result["position_delta"])
        self.assertEqual(wrist_result["raw_delta"], controller_result["raw_delta"])

    def test_tracking_loss_and_stale_pose_clear_control(self):
        self.wrist.update(self.pose([0.0, 0.0, 0.0]), {})
        lost = self.wrist.update(self.pose([0.1, 0.1, 0.1], tracked=False), {})
        self.assertFalse(lost["control_active"])
        self.assertFalse(lost["vr"]["anchor_set"])

        self.wrist.update(self.pose([0.0, 0.0, 0.0]), {})
        stale = self.wrist.update(self.pose([0.1, 0.1, 0.1], age=1.0), {})
        self.assertFalse(stale["control_active"])
        self.assertFalse(stale["vr"]["anchor_set"])

    def test_hand_and_controller_wrist_roll_map_to_joint4_at_half_angle(self):
        anchor = self.pose([0.0, 0.0, 0.0])
        controller_anchor = self.pose([0.0, 0.0, 0.0], source="right_controller")
        self.wrist.update(anchor, {})
        self.controller.update(controller_anchor, {})

        half = math.radians(90.0) / 2.0
        rotation = HAND.np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=float)
        wrist_pose = self.pose([0.0, 0.0, 0.0])
        wrist_pose["rotation"] = rotation
        controller_pose = self.pose([0.0, 0.0, 0.0], source="right_controller")
        controller_pose["rotation"] = rotation

        wrist_result = self.wrist.update(wrist_pose, {})
        controller_result = self.controller.update(controller_pose, {})
        expected = math.radians(-45.0)
        self.assertAlmostEqual(wrist_result["joint4_delta_rad"], expected)
        self.assertAlmostEqual(controller_result["joint4_delta_rad"], expected)

        self.wrist.update(self.pose([0.0, 0.0, 0.0], deadman=False), {})
        reanchor = self.pose([0.0, 0.0, 0.0])
        reanchor["rotation"] = rotation
        result = self.wrist.update(reanchor, {})
        self.assertAlmostEqual(result["joint4_delta_rad"], 0.0)

    def test_joint4_rotates_about_the_same_local_z_axis_used_for_forward_back(self):
        self.assertEqual(self.config["vr"]["joint4_axis"], "z")

        anchor_angle = math.radians(60.0) / 2.0
        anchor_rotation = HAND.np.array(
            [0.0, math.sin(anchor_angle), 0.0, math.cos(anchor_angle)],
            dtype=float,
        )
        anchor = self.pose([0.0, 0.0, 0.0])
        anchor["rotation"] = anchor_rotation
        self.wrist.update(anchor, {})

        twist_angle = math.radians(90.0) / 2.0
        local_forward_twist = HAND.np.array(
            [0.0, 0.0, math.sin(twist_angle), math.cos(twist_angle)],
            dtype=float,
        )
        moved = self.pose([0.0, 0.0, 0.2])
        moved["rotation"] = HAND.quaternion_multiply(anchor_rotation, local_forward_twist)
        result = self.wrist.update(moved, {})

        self.assertAlmostEqual(result["position_delta"]["x"], 0.1)
        self.assertAlmostEqual(result["joint4_delta_rad"], math.radians(-45.0))

        cross_axis_rotation = HAND.np.array(
            [math.sin(twist_angle), 0.0, 0.0, math.cos(twist_angle)],
            dtype=float,
        )
        moved["rotation"] = HAND.quaternion_multiply(anchor_rotation, cross_axis_rotation)
        result = self.wrist.update(moved, {})
        self.assertAlmostEqual(result["joint4_delta_rad"], 0.0)

    def test_missing_tcp_deadman_defaults_to_false(self):
        receiver = HAND.VRWristPoseReceiver("127.0.0.1", 0, threading.Event())
        receiver._parse_line("1,1.0,right,1,0.1,0.2,0.3,0,0,0,1")
        pose = receiver.latest("right", "xr_hand_wrist")
        self.assertIsNotNone(pose)
        self.assertFalse(pose["deadman_active"])

        receiver._parse_line("2,1.1,right,1,0.1,0.2,0.3,0,0,0,1,xr_hand_wrist,1")
        pose = receiver.latest("right", "xr_hand_wrist")
        self.assertTrue(pose["deadman_active"])


@unittest.skipIf(CAMERA is None, "opencv-python is required for camera mirror tests")
class CameraMirrorTests(unittest.TestCase):
    def test_rgb_camera_vertical_flip_reverses_rows(self):
        np = CAMERA.np
        frame = np.zeros((3, 1, 3), dtype=np.uint8)
        frame[:, 0, 0] = [1, 2, 3]
        flipped = CAMERA.apply_vertical_flip(frame, True)
        unchanged = CAMERA.apply_vertical_flip(frame, False)
        self.assertEqual(flipped[:, 0, 0].tolist(), [3, 2, 1])
        self.assertEqual(unchanged[:, 0, 0].tolist(), [1, 2, 3])

    def test_each_camera_is_mirrored_before_vertical_stack(self):
        np = CAMERA.np
        left = np.zeros((1, 3, 3), dtype=np.uint8)
        right = np.zeros((1, 3, 3), dtype=np.uint8)
        left[0, :, 0] = [1, 2, 3]
        right[0, :, 0] = [10, 20, 30]

        left_out = CAMERA.prepare_frame(left, 3, 1, True)
        right_out = CAMERA.prepare_frame(right, 3, 1, True)
        combined = np.vstack((left_out, right_out))

        self.assertEqual(combined[0, :, 0].tolist(), [3, 2, 1])
        self.assertEqual(combined[1, :, 0].tolist(), [30, 20, 10])

    def test_mirror_can_be_disabled_per_camera(self):
        np = CAMERA.np
        frame = np.zeros((1, 3, 3), dtype=np.uint8)
        frame[0, :, 0] = [1, 2, 3]
        out = CAMERA.prepare_frame(frame, 3, 1, False)
        self.assertEqual(out[0, :, 0].tolist(), [1, 2, 3])


class UiDeltaContractTests(unittest.TestCase):
    def test_h5_consumes_hand_delta_by_session_without_velocity_scaling(self):
        ui = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
        start = ui.index("function applyHandVisionStep()")
        end = ui.index("function applyControllerDeltaStep()", start)
        body = ui[start:end]
        self.assertIn("handVisionPositionDelta(handVisionState)", body)
        self.assertIn("handVisionLastSessionId !== sessionId", body)
        self.assertIn("delta.x - handVisionLastDelta.x", body)
        self.assertNotIn("teleopPhaseSpeedScale", body)
        self.assertNotIn("position_step_m", body)

    def test_h5_maps_session_relative_wrist_roll_to_joint4(self):
        ui = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
        self.assertIn("targetAngles[3] = handVisionJoint4Base + joint4DeltaFromState(state);", ui)
        self.assertIn("targetAngles[3] = controllerJoint4Base + joint4DeltaFromState(state);", ui)
        self.assertIn("applyHandVisionJoint4(handVisionState);", ui)
        self.assertIn("applyControllerJoint4(controllerDeltaState);", ui)

    def test_h5_wrist_panel_replaces_legacy_gripper_with_joint4_rotation(self):
        ui = (ROOT / "control_ui" / "index.html").read_text(encoding="utf-8")
        panel_start = ui.index('<div class="hand-axis-grid" id="handVisionReadout">')
        panel_end = ui.index("</div>\n          <p", panel_start)
        panel = ui[panel_start:panel_end]
        self.assertIn("腕部旋转 / J4", panel)
        self.assertIn('id="handWristRotation"', panel)
        self.assertNotIn("夹爪", panel)
        self.assertNotIn("handGripperCommand", ui)
        self.assertNotIn('mode === "hand_vision") return handVisionGripperCommand()', ui)

    def test_ffmpeg_fallback_flips_each_input_before_stack(self):
        script = (ROOT / "control_ui" / "start_dual_rtsp_camera.ps1").read_text(encoding="utf-8")
        config = (ROOT / "control_ui" / "config.yaml").read_text(encoding="utf-8")
        self.assertIn('$previousErrorActionPreference = $ErrorActionPreference', script)
        self.assertIn('$ErrorActionPreference = "Continue"', script)
        self.assertIn('$ErrorActionPreference = $previousErrorActionPreference', script)
        self.assertIn('$leftFlipFilter = if ($leftMirrorEnabled) { ",hflip" }', script)
        self.assertIn('$rightFlipFilter = if ($rightMirrorEnabled) { ",hflip" }', script)
        self.assertIn('$leftVerticalFlipFilter = if ($leftVerticalFlipEnabled) { ",vflip" }', script)
        self.assertIn('$rightVerticalFlipFilter = if ($rightVerticalFlipEnabled) { ",vflip" }', script)
        self.assertIn("[left][right]vstack=inputs=2", script)
        self.assertIn("left_mirror: false", config)
        self.assertIn("right_mirror: true", config)
        self.assertIn("left_vertical_flip: false", config)
        self.assertIn("right_vertical_flip: false", config)


class XrHandGestureTests(unittest.TestCase):
    @staticmethod
    def hand_positions(curl_ratio):
        values = [0.0] * UNITY_HAND.EXPECTED_FLOAT_COUNT
        for finger_index, (mcp, pip, tip, open_deg, close_deg) in enumerate(UNITY_HAND.FINGER_CURL_CALIBRATION):
            angle = open_deg - float(curl_ratio) * (open_deg - close_deg)
            theta = math.pi - math.radians(angle)
            y = float(finger_index) * 2.0
            points = {
                mcp: (-1.0, y, 0.0),
                pip: (0.0, y, 0.0),
                tip: (math.cos(theta), y + math.sin(theta), 0.0),
            }
            for joint, point in points.items():
                values[joint * 3:joint * 3 + 3] = point
        return values

    def test_left_fist_binary_state_and_hysteresis(self):
        closed = {"leftTracked": True, "leftPositions": self.hand_positions(0.9)}
        opened = {"leftTracked": True, "leftPositions": self.hand_positions(0.1)}
        middle = {"leftTracked": True, "leftPositions": self.hand_positions(0.65)}

        self.assertTrue(UNITY_HAND.classify_left_fist(closed)[0])
        self.assertFalse(UNITY_HAND.classify_left_fist(opened, previous=True)[0])
        self.assertFalse(UNITY_HAND.classify_left_fist(middle, previous=False)[0])
        self.assertTrue(UNITY_HAND.classify_left_fist(middle, previous=True)[0])

    def test_bridge_publishes_h5_left_fist_deadman_state(self):
        class FakeSocket:
            def __init__(self):
                self.sent = []

            def sendto(self, data, target):
                self.sent.append((json.loads(data.decode("utf-8")), target))

        bridge = UNITY_HAND.UnityHandBridge(
            "127.0.0.1", 5006, "127.0.0.1", 25001, False, True, 0.4, ""
        )
        payload = {
            "source": "xr-hands",
            "frameId": 1,
            "unityTime": 1.0,
            "rightTracked": True,
            "leftTracked": True,
            "rightPositions": [0.0] * UNITY_HAND.EXPECTED_FLOAT_COUNT,
            "rightRotations": [0.0] * UNITY_HAND.EXPECTED_FLOAT_COUNT,
            "leftPositions": self.hand_positions(0.9),
            "leftRotations": [0.0] * UNITY_HAND.EXPECTED_FLOAT_COUNT,
        }
        sock = FakeSocket()
        bridge._handle_line(json.dumps(payload).encode("utf-8"), sock)
        forwarded = [item for item, target in sock.sent if target[1] == 25001][0]
        self.assertEqual(forwarded["source"], "xr-hands")
        self.assertIn("unity-xr-hands", forwarded["jointFormat"])
        self.assertEqual(len(forwarded["rightPositions"]), 63)
        self.assertEqual(len(forwarded["rightRotations"]), 63)
        gesture = [item for item, target in sock.sent if target[1] == 25002][0]
        self.assertTrue(gesture["left_fist"])
        self.assertEqual(gesture["deadman"], 1)
        self.assertIn("right_operator_intent_closure_prior", gesture)
        self.assertFalse(gesture["right_prior_is_robot_actual_closure"])

    def test_stale_gesture_state_fails_closed(self):
        stopped = threading.Event()
        receiver = HAND.XrHandGestureStateReceiver("127.0.0.1", 0, 0.4, stopped)
        receiver.latest = {"left_tracked": True, "left_fist": True, "left_fist_score": 0.9, "frame_id": 1}
        receiver.received_at = time.monotonic()
        self.assertTrue(receiver.snapshot()["deadman_active"])
        receiver.received_at = time.monotonic() - 1.0
        self.assertFalse(receiver.snapshot()["deadman_active"])


if __name__ == "__main__":
    unittest.main()
