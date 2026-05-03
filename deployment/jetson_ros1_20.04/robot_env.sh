#!/usr/bin/env bash
set -euo pipefail

# ROS1 on Ubuntu 20.04 uses Noetic.
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.1.35:11311}"
export ROS_IP="${ROS_IP:-192.168.1.35}"

# Change this to your real catkin workspace on your system.
export ROBOT_WS="${ROBOT_WS:-/home/mumu/DataSet_ws}"
export ROBOT_WS_SETUP="${ROBOT_WS}/devel/setup.bash"

# Runtime files.
export ROBOT_RUNTIME_DIR="${ROBOT_RUNTIME_DIR:-/home/mumu/robot/runtime}"
export ROBOT_LOG_DIR="${ROBOT_LOG_DIR:-/home/mumu/robot/logs}"

# Replace these commands with your actual ROS package/node or roslaunch commands.
export CAN_CMD="${CAN_CMD:-roslaunch mainpulator socketcan.launch}"
export ARM_CMD="${ARM_CMD:-roslaunch mainpulator mainpulatorlaunch.launch}"

# Keep the H5 UDP bridge enabled so the UI receives arm telemetry even when
# the low-level arm node also consumes the command stream directly.
export UDP_BRIDGE_ENABLE="${UDP_BRIDGE_ENABLE:-1}"
export UDP_BRIDGE_CMD="${UDP_BRIDGE_CMD:-roslaunch h5_udp_bridge h5_udp_bridge.launch}"

# Match these with control_ui/config.yaml.
export H5_UDP_LISTEN_PORT="${H5_UDP_LISTEN_PORT:-14551}"
export H5_TELEMETRY_TARGET_IP="${H5_TELEMETRY_TARGET_IP:-192.168.1.11}"
export H5_TELEMETRY_TARGET_PORT="${H5_TELEMETRY_TARGET_PORT:-14550}"
