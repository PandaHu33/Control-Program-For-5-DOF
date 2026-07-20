#!/usr/bin/env bash
set -euo pipefail

# ROS1 on Ubuntu 18.04 uses Melodic.
export ROS_DISTRO="${ROS_DISTRO:-melodic}"
export ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.1.35:11311}"
export ROS_IP="${ROS_IP:-192.168.1.35}"

# Change this to your real catkin workspace on your system.
export ROBOT_WS="${ROBOT_WS:-/home/night/DataSet_ws}"
export ROBOT_WS_SETUP="${ROBOT_WS}/devel/setup.bash"

# Runtime files.
export ROBOT_RUNTIME_DIR="${ROBOT_RUNTIME_DIR:-/home/night/robot/runtime}"
export ROBOT_LOG_DIR="${ROBOT_LOG_DIR:-/home/night/robot/logs}"
# Field default: do not create persistent stdout/stderr log files on Jetson.
# Set ROBOT_LOG_ENABLE=1 only when you intentionally need bringup logs.
export ROBOT_LOG_ENABLE="${ROBOT_LOG_ENABLE:-0}"

# Replace these commands with your actual ROS package/node or roslaunch commands.
export CAN_CMD="${CAN_CMD:-roslaunch mainpulator socketcan.launch}"
export ARM_CONTROL_TYPE="${ARM_CONTROL_TYPE:-torque}"
export ARM_HOME_TIMEOUT="${ARM_HOME_TIMEOUT:-12}"
export ARM_RUNTIME_MOTION_DURATION="${ARM_RUNTIME_MOTION_DURATION:-5.0}"
export ARM_TRAJECTORY_PLANNER_ENABLE="${ARM_TRAJECTORY_PLANNER_ENABLE:-1}"
export ARM_MOTION_DATA_LOG_ENABLE="${ARM_MOTION_DATA_LOG_ENABLE:-1}"
export ARM_MOTION_DATA_LOG_PATH="${ARM_MOTION_DATA_LOG_PATH:-${ROBOT_LOG_DIR}/arm_motion_planned_2x.csv}"
export ARM_CMD="${ARM_CMD:-roslaunch mainpulator mainpulatorlaunch.launch control_type:=${ARM_CONTROL_TYPE} runtime_motion_duration:=${ARM_RUNTIME_MOTION_DURATION} trajectory_planner_enabled:=${ARM_TRAJECTORY_PLANNER_ENABLE} motion_data_log_enabled:=${ARM_MOTION_DATA_LOG_ENABLE} motion_data_log_path:=${ARM_MOTION_DATA_LOG_PATH}}"

# Keep the H5 UDP bridge enabled so the UI receives arm telemetry even when
# the low-level arm node also consumes the command stream directly.
export UDP_BRIDGE_ENABLE="${UDP_BRIDGE_ENABLE:-1}"
export UDP_BRIDGE_CMD="${UDP_BRIDGE_CMD:-roslaunch h5_udp_bridge h5_udp_bridge.launch}"

# Match these with control_ui/config.yaml.
export H5_UDP_LISTEN_PORT="${H5_UDP_LISTEN_PORT:-14551}"
export H5_TELEMETRY_TARGET_IP="${H5_TELEMETRY_TARGET_IP:-192.168.1.11}"
export H5_TELEMETRY_TARGET_PORT="${H5_TELEMETRY_TARGET_PORT:-14550}"
