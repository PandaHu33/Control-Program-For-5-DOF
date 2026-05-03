#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${BASE_DIR}/robot_env.sh"

is_running() {
  local name="$1"
  local pid_file="${ROBOT_RUNTIME_DIR}/${name}.pid"
  [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" >/dev/null 2>&1
}

if pgrep -f "roscore|rosmaster" >/dev/null 2>&1; then
  echo "ROSCORE:RUNNING"
else
  echo "ROSCORE:STOPPED"
fi

if is_running "can_node"; then
  echo "CAN_NODE:RUNNING"
else
  echo "CAN_NODE:STOPPED"
fi

if is_running "arm_control"; then
  echo "ARM_CONTROL:RUNNING"
else
  echo "ARM_CONTROL:STOPPED"
fi

if [[ "${UDP_BRIDGE_ENABLE}" == "1" ]]; then
  if is_running "h5_udp_bridge"; then
    echo "UDP_BRIDGE:RUNNING"
  else
    echo "UDP_BRIDGE:STOPPED"
  fi
fi
