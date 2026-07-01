#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${BASE_DIR}/robot_env.sh"

mkdir -p "${ROBOT_RUNTIME_DIR}"
if [[ "${ROBOT_LOG_ENABLE:-0}" == "1" ]]; then
  mkdir -p "${ROBOT_LOG_DIR}"
else
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/robot_ros_logs}"
  mkdir -p "${ROS_LOG_DIR}"
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}"
  exit 10
fi

source "${ROS_SETUP}"
if [[ -f "${ROBOT_WS_SETUP}" ]]; then
  source "${ROBOT_WS_SETUP}"
else
  echo "WARNING: workspace setup not found: ${ROBOT_WS_SETUP}"
fi

start_named_process() {
  local name="$1"
  local cmd="$2"
  local pid_file="${ROBOT_RUNTIME_DIR}/${name}.pid"
  local log_file="/dev/null"
  if [[ "${ROBOT_LOG_ENABLE:-0}" == "1" ]]; then
    log_file="${ROBOT_LOG_DIR}/${name}.log"
  fi

  if [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" >/dev/null 2>&1; then
    echo "${name}:ALREADY_RUNNING pid=$(cat "${pid_file}")"
    return 0
  fi

  bash -lc "source '${ROS_SETUP}'; [[ -f '${ROBOT_WS_SETUP}' ]] && source '${ROBOT_WS_SETUP}'; exec ${cmd}" \
    >"${log_file}" 2>&1 &
  echo $! > "${pid_file}"
  sleep 0.5

  if kill -0 "$(cat "${pid_file}")" >/dev/null 2>&1; then
    echo "${name}:STARTED pid=$(cat "${pid_file}")"
  else
    if [[ "${ROBOT_LOG_ENABLE:-0}" == "1" ]]; then
      echo "${name}:FAILED see ${log_file}"
    else
      echo "${name}:FAILED"
    fi
    exit 20
  fi
}

if ! pgrep -f "roscore|rosmaster" >/dev/null 2>&1; then
  roscore_log_file="/dev/null"
  if [[ "${ROBOT_LOG_ENABLE:-0}" == "1" ]]; then
    roscore_log_file="${ROBOT_LOG_DIR}/roscore.log"
  fi
  roscore >"${roscore_log_file}" 2>&1 &
  echo $! > "${ROBOT_RUNTIME_DIR}/roscore.pid"
  sleep 2
fi

start_named_process "can_node" "${CAN_CMD}"
sleep 2
start_named_process "arm_control" "${ARM_CMD}"

if [[ "${UDP_BRIDGE_ENABLE}" == "1" ]]; then
  start_named_process "h5_udp_bridge" "${UDP_BRIDGE_CMD}"
fi

echo "ARM_NODES_STARTED"
