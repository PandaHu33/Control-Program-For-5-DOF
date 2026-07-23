import ast
import importlib.util
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class _Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


def load_h5_bridge(path, module_name):
    rospy = types.ModuleType("rospy")
    rospy.Time = types.SimpleNamespace(now=lambda: 0)
    sys.modules.setdefault("rospy", rospy)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _Message
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Imu = _Message
    sensor_msgs_msg.JointState = _Message
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    mainpulator = types.ModuleType("mainpulator")
    mainpulator_msg = types.ModuleType("mainpulator.msg")
    mainpulator_msg.ArmRecordingState = _Message
    sys.modules.setdefault("mainpulator", mainpulator)
    sys.modules.setdefault("mainpulator.msg", mainpulator_msg)

    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class H5SourceHelperTests(unittest.TestCase):
    def test_active_source_is_initialized_once_in_constructor(self):
        paths = [
            ROOT / "DataSet_ws/src/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
            ROOT / "deployment/jetson_ros1_20.04/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
        ]
        for path in paths:
            tree = ast.parse(path.read_text(encoding="utf-8"))
            bridge_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "H5UdpBridge")
            methods = {node.name: node for node in bridge_class.body if isinstance(node, ast.FunctionDef)}

            def active_source_assignments(method):
                return [
                    node for node in ast.walk(method)
                    if isinstance(node, (ast.Assign, ast.AnnAssign))
                    and any(
                        isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"
                        and target.attr == "active_source"
                        for target in (node.targets if isinstance(node, ast.Assign) else [node.target])
                    )
                ]

            self.assertEqual(len(active_source_assignments(methods["__init__"])), 1, path)

    def test_workspace_and_deployment_helpers_match(self):
        paths = [
            ROOT / "DataSet_ws/src/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
            ROOT / "deployment/jetson_ros1_20.04/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
        ]
        for index, path in enumerate(paths):
            module = load_h5_bridge(path, "h5_bridge_test_%d" % index)
            self.assertEqual(module.source_switch_from_note("bridge:source:keyboard"), "keyboard")
            self.assertEqual(module.source_from_note("ui:controller_delta"), "controller_delta")
            self.assertEqual(module.source_from_note("bridge:home"), "home")
            self.assertEqual(module.source_from_note("bridge:preset:test"), "preset")
            self.assertIsNone(module.source_from_note("ui:vision"))
            self.assertIsNone(module.source_switch_from_note("bridge:source:unknown"))

    def test_versioned_recording_message_serializes_validity_and_nan(self):
        module = load_h5_bridge(
            ROOT / "DataSet_ws/src/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
            "h5_bridge_recording_test",
        )
        bridge = object.__new__(module.H5UdpBridge)
        bridge.tx_queue = module.queue.Queue()

        class Stamp:
            def to_sec(self):
                return 1.25

            def to_nsec(self):
                return 1_250_000_000

        message = _Message(
            schema_version=1,
            header=_Message(seq=9, stamp=Stamp()),
            control_mode="torque",
            current_valid_mask=0b00111,
            velocity_valid_mask=0b01111,
            actual_q_rad=[1.0] * 5,
            target_q_rad=[2.0] * 5,
            actual_dq_rad_s=[3.0, 3.0, 3.0, 3.0, float("nan")],
            target_dq_rad_s=[4.0] * 5,
            actual_current_ma=[100.0, 200.0, 300.0, float("nan"), float("nan")],
            commanded_torque_nm=[5.0, 5.0, 5.0, float("nan"), float("nan")],
        )
        bridge.on_recording_state(message)
        payload = bridge.tx_queue.get_nowait()["_recording"]
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["source_time_ns"], 1_250_000_000)
        self.assertEqual(payload["current_valid_mask"], 0b00111)
        self.assertIsNone(payload["actual_current_ma"][4])


if __name__ == "__main__":
    unittest.main()
