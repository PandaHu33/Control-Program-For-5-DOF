#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${BASE_DIR}/robot_env.sh"

mkdir -p "${ROBOT_RUNTIME_DIR}" "${ROBOT_LOG_DIR}"

stop_named_process() {
  local name="$1"
  local pid_file="${ROBOT_RUNTIME_DIR}/${name}.pid"

  if [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" >/dev/null 2>&1; then
    kill "$(cat "${pid_file}")" >/dev/null 2>&1 || true
    sleep 1
    if kill -0 "$(cat "${pid_file}")" >/dev/null 2>&1; then
      kill -9 "$(cat "${pid_file}")" >/dev/null 2>&1 || true
    fi
    rm -f "${pid_file}"
    echo "${name}:STOPPED"
  else
    rm -f "${pid_file}"
    echo "${name}:NOT_RUNNING"
  fi
}

stop_named_process "h5_udp_bridge"
stop_named_process "arm_control"
stop_named_process "can_node"

# Stop roscore by default when the H5 supervisor is closed.
if [[ "${KEEP_ROSCORE:-0}" != "1" ]]; then
  stop_named_process "roscore"
fi

echo "ARM_NODES_STOPPED"
