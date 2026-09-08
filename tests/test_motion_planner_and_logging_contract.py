import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "DataSet_ws/src/mainpulator"


class MotionPlannerAndLoggingContractTests(unittest.TestCase):
    def test_vendored_ruckig_is_pinned_and_built_without_network_fetch(self):
        version = (PACKAGE / "third_party/ruckig/VERSION").read_text(encoding="utf-8")
        cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("v0.15.3", version)
        self.assertIn("37b6e7a4c60f5befd2506741d5f7d0ae7eefb3db", version)
        self.assertIn("set(CMAKE_CXX_STANDARD 17)", cmake)
        self.assertIn("add_library(ruckig_local", cmake)
        self.assertIn("third_party/ruckig/src/ruckig/brake.cpp", cmake)
        self.assertNotIn("FetchContent", cmake)

    def test_planner_is_four_dof_100_hz_time_synchronized_and_discrete(self):
        header = (PACKAGE / "include/mainpulator/online_joint_planner.h").read_text(encoding="utf-8")
        planner = (PACKAGE / "src/online_joint_planner.cpp").read_text(encoding="utf-8")
        node = (PACKAGE / "src/test_node.cpp").read_text(encoding="utf-8")
        self.assertIn("kArmPlannerDofs = 4", header)
        self.assertIn("control_period_sec = 0.01", header)
        self.assertIn("Synchronization::Time", planner)
        self.assertIn("DurationDiscretization::Discrete", planner)
        self.assertIn("OnlineJointPlanner(0.01)", node)
        self.assertIn("output_.pass_to_input(input_)", planner)
        self.assertIn("trajectory_planner_enabled && control_mode != ControlMode::Torque", node)

    def test_planner_switch_and_limits_are_exposed_with_safe_defaults(self):
        launch = (PACKAGE / "launch/mainpulatorlaunch.launch").read_text(encoding="utf-8")
        for expected in (
            '<arg name="trajectory_planner_enabled" default="true" />',
            '<arg name="trajectory_max_velocity" default="[1.0, 1.0, 1.0, 1.0]" />',
            '<arg name="trajectory_max_acceleration" default="[2.0, 2.0, 2.0, 2.0]" />',
            '<arg name="trajectory_max_jerk" default="[10.0, 10.0, 10.0, 10.0]" />',
        ):
            self.assertIn(expected, launch)

    def test_logger_is_bounded_nonblocking_and_drained_on_shutdown(self):
        logger = (PACKAGE / "src/motion_csv_logger.cpp").read_text(encoding="utf-8")
        node = (PACKAGE / "src/test_node.cpp").read_text(encoding="utf-8")
        self.assertIn("std::try_to_lock", logger)
        self.assertIn("queue_.size() >= config_.queue_capacity", logger)
        self.assertIn("writer_thread_ = std::thread", logger)
        self.assertIn("writer_thread_.join()", logger)
        self.assertIn("rows_since_flush >= config_.flush_rows", logger)
        self.assertIn("motion_csv_logger->stop()", node)
        self.assertIn("RecordMotionLogSample(loop_index++, loop_dt_sec)", node)

    def test_csv_schema_and_log_launch_defaults(self):
        logger = (PACKAGE / "src/motion_csv_logger.cpp").read_text(encoding="utf-8")
        launch = (PACKAGE / "launch/mainpulatorlaunch.launch").read_text(encoding="utf-8")
        for field in (
            "session_id,wall_time_ns,ros_time_ns,loop_index,dt_sec",
            '"raw_q", "raw_dq", "raw_ddq", "expected_q", "expected_dq", "expected_ddq", "actual_q", "actual_dq"',
            "dropped_log_samples",
        ):
            self.assertIn(field, logger)
        self.assertIn('<arg name="motion_data_log_enabled" default="false" />', launch)
        self.assertIn('<arg name="motion_data_log_queue_capacity" default="8192" />', launch)
        self.assertIn('<arg name="motion_data_log_flush_rows" default="100" />', launch)

    def test_deployment_environment_switches_are_forwarded(self):
        for relative_path in ("robot/robot_env.sh", "deployment/jetson_ros1_20.04/robot_env.sh"):
            environment = (ROOT / relative_path).read_text(encoding="utf-8")
            self.assertIn("ARM_TRAJECTORY_PLANNER_ENABLE", environment)
            self.assertIn("ARM_MOTION_DATA_LOG_ENABLE", environment)
            self.assertIn("ARM_MOTION_DATA_LOG_PATH", environment)
            self.assertIn('ARM_JOINT5_ENABLE="${ARM_JOINT5_ENABLE:-0}"', environment)
            self.assertIn("trajectory_planner_enabled:=${ARM_TRAJECTORY_PLANNER_ENABLE}", environment)
            self.assertIn("motion_data_log_enabled:=${ARM_MOTION_DATA_LOG_ENABLE}", environment)
            self.assertIn("joint5_enabled:=${ARM_JOINT5_ENABLE}", environment)

    def test_source_wrist_planning_is_enabled_but_slave_ruckig_owns_joint_trajectory(self):
        config = (ROOT / "control_ui/config.yaml").read_text(encoding="utf-8")
        node = (PACKAGE / "src/test_node.cpp").read_text(encoding="utf-8")
        for environment_path in (
            ROOT / "robot/robot_env.sh",
            ROOT / "deployment/jetson_ros1_20.04/robot_env.sh",
        ):
            environment = environment_path.read_text(encoding="utf-8")
            self.assertIn('ARM_TRAJECTORY_PLANNER_ENABLE="${ARM_TRAJECTORY_PLANNER_ENABLE:-1}"', environment)
        self.assertIn("planner_enabled: true", config)
        self.assertIn('planner_keyboard_profile: "jerk_limited"', config)
        self.assertIn('planner_gamepad_profile: "jerk_limited"', config)
        self.assertIn('planner_pico_wrist_profile: "one_euro"', config)
        self.assertIn('planner_vr_controller_profile: "one_euro"', config)
        self.assertIn('target_dq.fill(0.0);', node)
        self.assertIn('target_ddq.fill(0.0);', node)
        self.assertNotIn('expect_dq(0,0) = -msg.angular_velocity.x;', node)
        self.assertNotIn('expect_ddq(0,0) = -msg.linear_acceleration.x;', node)

    def test_controller_fixed_step_and_startup_homing_contract_remain(self):
        node = (PACKAGE / "src/test_node.cpp").read_text(encoding="utf-8")
        self.assertEqual(node.count("AdaptiveBackstepping(expect_q, expect_dq, expect_ddq, q, dq, zerovector, 0.002)"), 1)
        self.assertIn("10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5", node)
        self.assertLess(node.index("param::MomentInit(joint5, socket_can)"),
                        node.index("torque_home_start_q = q"))


if __name__ == "__main__":
    unittest.main()
