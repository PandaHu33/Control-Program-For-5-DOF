import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def coefficients(q0, dq0, ddq0, qf, duration):
    delta = qf - q0
    return (
        q0,
        dq0,
        0.5 * ddq0,
        (20.0 * delta - 12.0 * dq0 * duration - 3.0 * ddq0 * duration**2) / (2.0 * duration**3),
        (-30.0 * delta + 16.0 * dq0 * duration + 3.0 * ddq0 * duration**2) / (2.0 * duration**4),
        (12.0 * delta - 6.0 * dq0 * duration - ddq0 * duration**2) / (2.0 * duration**5),
    )


def evaluate(c, t):
    q = sum(value * t**index for index, value in enumerate(c))
    dq = sum(index * c[index] * t ** (index - 1) for index in range(1, 6))
    ddq = sum(index * (index - 1) * c[index] * t ** (index - 2) for index in range(2, 6))
    return q, dq, ddq


class RuntimeMotionContractTests(unittest.TestCase):
    def test_general_quintic_boundaries_and_replan_continuity(self):
        cases = [
            (0.4, 0.0, 0.0, 0.0, 5.0),
            (-1.2, 0.3, -0.2, 0.7, 5.0),
            (0.9, -0.4, 0.15, -0.5, 2.5),
        ]
        for q0, dq0, ddq0, qf, duration in cases:
            c = coefficients(q0, dq0, ddq0, qf, duration)
            self.assertEqual(evaluate(c, 0.0), (q0, dq0, ddq0))
            q, dq, ddq = evaluate(c, duration)
            self.assertAlmostEqual(q, qf, places=10)
            self.assertAlmostEqual(dq, 0.0, places=10)
            self.assertAlmostEqual(ddq, 0.0, places=10)

    def test_launch_defaults_and_interfaces(self):
        launch = (ROOT / "DataSet_ws/src/mainpulator/launch/mainpulatorlaunch.launch").read_text(encoding="utf-8")
        source = (ROOT / "DataSet_ws/src/mainpulator/src/test_node.cpp").read_text(encoding="utf-8")
        self.assertIn('<arg name="runtime_motion_duration" default="5.0" />', launch)
        self.assertIn('<arg name="joint5_enabled" default="false" />', launch)
        self.assertIn('nh.param("joint5_enabled", joint5_enabled, false);', source)
        self.assertEqual(
            len(re.findall(
                r"if \(joint5_enabled\)\s*\{\s*param::MomentInit\(joint5, socket_can\);",
                source,
            )),
            2,
        )
        self.assertIn("feedback_joint_count = joint5_enabled ? 5 : 4;", source)
        self.assertIn("rollback < feedback_joint_count", source)
        self.assertEqual(
            len(re.findall(
                r"if \(joint5_enabled\)\s*\{\s*param::Enable\(socket_can, 6, joint5\);",
                source,
            )),
            2,
        )
        self.assertRegex(
            source,
            r"if \(joint5_enabled\)\s*\{\s*// 关节 5力矩控制[\s\S]*?joint5\.MomentOutput\(tol5\);",
        )
        self.assertIn("if (index == 4 && !joint5_enabled) break;", source)
        self.assertIn('"/arm/motion_status"', source)
        self.assertIn("StartRuntimeMotion(msg, command_source)", source)
        self.assertIn("last_runtime_motion_stamp == msg.header.stamp", source)

    def test_ros_info_calls_remain_commented(self):
        paths = [
            ROOT / "DataSet_ws/src/mainpulator/src/test_node.cpp",
            ROOT / "DataSet_ws/src/h5_udp_bridge/scripts/h5_udp_bridge_node.py",
        ]
        for path in paths:
            for line in path.read_text(encoding="utf-8").splitlines():
                stripped = line.lstrip()
                self.assertFalse("ROS_INFO" in stripped and not stripped.startswith("//"), line)
                self.assertFalse("rospy.loginfo" in stripped and not stripped.startswith("#"), line)

    def test_legacy_torque_startup_order_precedes_feedback_and_planning(self):
        source = (ROOT / "DataSet_ws/src/mainpulator/src/test_node.cpp").read_text(encoding="utf-8")
        moment5 = source.index("param::MomentInit(joint5, socket_can)")
        moment1 = source.index("param::MomentInit(joint1, socket_can)")
        moment2 = source.index("param::MomentInit(joint2, socket_can)")
        moment3 = source.index("param::MomentInit(joint3, socket_can)")
        position4 = source.index("param::PositionInit(joint4, socket_can)")
        enable1 = source.index("param::Enable(socket_can, 6, joint1)")
        enable2 = source.index("param::Enable(socket_can, 6, joint2)")
        enable3 = source.index("param::Enable(socket_can, 6, joint3)")
        enable5 = source.index("param::Enable(socket_can, 6, joint5)")
        enable4 = source.index("param::Enable(socket_can, 6, joint4)")
        feedback = source.index("if (!TorqueFeedbackReady())")
        capture = source.index("torque_home_start_q = q")
        ordered = [moment5, moment1, moment2, moment3, position4,
                   enable1, enable2, enable3, enable5, enable4, feedback, capture]
        self.assertEqual(ordered, sorted(ordered))
        self.assertNotIn("StartCanopenCommunication", source)

    def test_ui_keeps_zero_velocity_and_acceleration_slots(self):
        ui = (ROOT / "control_ui/index.html").read_text(encoding="utf-8")
        self.assertIn("const orderVec = Array(orderCount).fill(0)", ui)
        self.assertNotIn("orderVec[4] =", ui)
        self.assertNotIn("orderVec[7] =", ui)
        self.assertIn("ARM_COMMAND_PERIOD_MS", ui)


if __name__ == "__main__":
    unittest.main()
