#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${BASE_DIR}/robot_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${BASE_DIR}/robot_env.sh"
else
  export ROBOT_LOG_DIR="${ROBOT_LOG_DIR:-${BASE_DIR}/logs}"
fi

usage() {
  cat <<'EOF'
Usage:
  can_loss_test.sh [-i can0] [-t 30] [--expected-rate FPS] [--expected-count N]

Options:
  -i, --interface IFACE      CAN interface name. Default: can0
  -t, --duration SECONDS     Capture duration. Default: 30
  --expected-rate FPS        Optional expected bus receive rate in frames/sec.
  --expected-count N         Optional expected frame count for the test window.
  -h, --help                 Show this help.

Examples:
  /home/night/robot/can_loss_test.sh
  /home/night/robot/can_loss_test.sh -i can0 -t 60
  /home/night/robot/can_loss_test.sh -i can0 -t 60 --expected-rate 1000
EOF
}

IFACE="can0"
DURATION="30"
EXPECTED_RATE=""
EXPECTED_COUNT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--interface)
      IFACE="${2:-}"
      shift 2
      ;;
    -t|--duration)
      DURATION="${2:-}"
      shift 2
      ;;
    --expected-rate)
      EXPECTED_RATE="${2:-}"
      shift 2
      ;;
    --expected-count)
      EXPECTED_COUNT="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${IFACE}" ]]; then
  echo "ERROR: CAN interface cannot be empty" >&2
  exit 2
fi

if ! [[ "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk "BEGIN { exit !(${DURATION} > 0) }"; then
  echo "ERROR: duration must be a positive number" >&2
  exit 2
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "ERROR: missing command: ip" >&2
  exit 10
fi

if ! command -v candump >/dev/null 2>&1; then
  echo "ERROR: missing command: candump. Install can-utils first: sudo apt install -y can-utils" >&2
  exit 11
fi

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "ERROR: CAN interface not found: ${IFACE}" >&2
  echo "Available CAN-like interfaces:" >&2
  ip -o link show | awk -F': ' '/can|vcan/{print "  " $2}' >&2 || true
  exit 12
fi

mkdir -p "${ROBOT_LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RAW_LOG="${ROBOT_LOG_DIR}/can_loss_${IFACE}_${STAMP}.candump"
SUMMARY_LOG="${ROBOT_LOG_DIR}/can_loss_${IFACE}_${STAMP}.summary"

read_rx_stats() {
  local iface="$1"
  python3 - "$iface" <<'PY'
import pathlib
import sys

iface = sys.argv[1]
base = pathlib.Path("/sys/class/net") / iface / "statistics"
names = ("rx_packets", "rx_dropped", "rx_errors", "rx_over_errors", "rx_fifo_errors", "tx_packets", "tx_dropped", "tx_errors")
values = []
for name in names:
    try:
        values.append(str(int((base / name).read_text().strip())))
    except Exception:
        values.append("0")
print(" ".join(values))
PY
}

format_rate() {
  local count="$1"
  local seconds="$2"
  awk -v c="${count}" -v s="${seconds}" 'BEGIN { if (s > 0) printf "%.2f", c / s; else printf "0.00" }'
}

loss_percent() {
  local lost="$1"
  local expected="$2"
  awk -v l="${lost}" -v e="${expected}" 'BEGIN { if (e > 0) printf "%.4f", (l * 100.0) / e; else printf "0.0000" }'
}

echo "CAN loss test starting"
echo "  interface: ${IFACE}"
echo "  duration:  ${DURATION}s"
echo "  raw log:   ${RAW_LOG}"
echo "  summary:   ${SUMMARY_LOG}"
echo

read -r RX0 DROP0 ERR0 OVER0 FIFO0 TX0 TXDROP0 TXERR0 < <(read_rx_stats "${IFACE}")

set +e
timeout --preserve-status "${DURATION}" candump -td -L "${IFACE}" >"${RAW_LOG}" 2>"${RAW_LOG}.err"
CANDUMP_STATUS=$?
set -e

read -r RX1 DROP1 ERR1 OVER1 FIFO1 TX1 TXDROP1 TXERR1 < <(read_rx_stats "${IFACE}")

RX_DELTA=$((RX1 - RX0))
DROP_DELTA=$((DROP1 - DROP0))
ERR_DELTA=$((ERR1 - ERR0))
OVER_DELTA=$((OVER1 - OVER0))
FIFO_DELTA=$((FIFO1 - FIFO0))
TX_DELTA=$((TX1 - TX0))
TXDROP_DELTA=$((TXDROP1 - TXDROP0))
TXERR_DELTA=$((TXERR1 - TXERR0))
CAPTURED_COUNT="$(grep -cve '^[[:space:]]*$' "${RAW_LOG}" || true)"
USERSPACE_LOST=$((RX_DELTA - CAPTURED_COUNT))
if [[ "${USERSPACE_LOST}" -lt 0 ]]; then
  USERSPACE_LOST=0
fi

KERNEL_EXPECTED=$((RX_DELTA + DROP_DELTA + ERR_DELTA + OVER_DELTA + FIFO_DELTA))
if [[ "${KERNEL_EXPECTED}" -lt "${CAPTURED_COUNT}" ]]; then
  KERNEL_EXPECTED="${CAPTURED_COUNT}"
fi
KERNEL_LOST=$((DROP_DELTA + ERR_DELTA + OVER_DELTA + FIFO_DELTA + USERSPACE_LOST))

if [[ -n "${EXPECTED_COUNT}" ]]; then
  TEST_EXPECTED="${EXPECTED_COUNT}"
elif [[ -n "${EXPECTED_RATE}" ]]; then
  TEST_EXPECTED="$(awk -v r="${EXPECTED_RATE}" -v s="${DURATION}" 'BEGIN { printf "%d", r * s + 0.5 }')"
else
  TEST_EXPECTED="${KERNEL_EXPECTED}"
fi

TEST_LOST=$((TEST_EXPECTED - CAPTURED_COUNT))
if [[ "${TEST_LOST}" -lt 0 ]]; then
  TEST_LOST=0
fi

{
  echo "CAN_LOSS_TEST"
  echo "timestamp=${STAMP}"
  echo "interface=${IFACE}"
  echo "duration_sec=${DURATION}"
  echo "candump_status=${CANDUMP_STATUS}"
  echo "raw_log=${RAW_LOG}"
  echo
  echo "COUNTS"
  echo "captured_frames=${CAPTURED_COUNT}"
  echo "kernel_rx_packets_delta=${RX_DELTA}"
  echo "kernel_rx_dropped_delta=${DROP_DELTA}"
  echo "kernel_rx_errors_delta=${ERR_DELTA}"
  echo "kernel_rx_over_errors_delta=${OVER_DELTA}"
  echo "kernel_rx_fifo_errors_delta=${FIFO_DELTA}"
  echo "userspace_capture_gap=${USERSPACE_LOST}"
  echo "kernel_tx_packets_delta=${TX_DELTA}"
  echo "kernel_tx_dropped_delta=${TXDROP_DELTA}"
  echo "kernel_tx_errors_delta=${TXERR_DELTA}"
  echo
  echo "RATES"
  echo "captured_fps=$(format_rate "${CAPTURED_COUNT}" "${DURATION}")"
  echo "kernel_rx_fps=$(format_rate "${RX_DELTA}" "${DURATION}")"
  echo
  echo "LOSS"
  echo "kernel_observed_expected=${KERNEL_EXPECTED}"
  echo "kernel_observed_lost=${KERNEL_LOST}"
  echo "kernel_observed_loss_percent=$(loss_percent "${KERNEL_LOST}" "${KERNEL_EXPECTED}")"
  echo "test_expected=${TEST_EXPECTED}"
  echo "test_lost=${TEST_LOST}"
  echo "test_loss_percent=$(loss_percent "${TEST_LOST}" "${TEST_EXPECTED}")"
  echo
  echo "TOP_CAN_IDS"
  awk '
    NF >= 3 {
      id=$3
      sub(/#.*$/, "", id)
      count[id]++
    }
    END {
      for (id in count) print count[id], id
    }
  ' "${RAW_LOG}" | sort -nr | head -20
  echo
  echo "NOTES"
  echo "- kernel_observed_loss_percent uses Linux interface dropped/error/overrun counters plus a candump capture gap."
  echo "- test_loss_percent is only meaningful when --expected-rate or --expected-count matches the real transmitter rate/count."
  echo "- If candump_status is 124 or 143, timeout stopped candump normally after the requested duration."
} | tee "${SUMMARY_LOG}"

if [[ -s "${RAW_LOG}.err" ]]; then
  echo
  echo "candump stderr:"
  cat "${RAW_LOG}.err"
fi
