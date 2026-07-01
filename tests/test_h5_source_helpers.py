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

    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class H5SourceHelperTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
