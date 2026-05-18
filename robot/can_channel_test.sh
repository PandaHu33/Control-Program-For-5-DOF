#!/bin/bash

IFACE=${1:-can0}
COUNT=${2:-100}
GAP=${3:-0.02}

IDS=("001" "002" "003" "004" "005")

echo "CAN interface: $IFACE"
echo "Test count per ID: $COUNT"
echo "Gap: ${GAP}s"
echo

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "Missing command: $1"
        exit 1
    }
}

need_cmd ip
need_cmd cansend
need_cmd candump

echo "[1] Initial CAN status:"
ip -details -statistics link show "$IFACE"
echo

LOG="/tmp/can_test_${IFACE}_$(date +%s).log"

echo "[2] Start candump log: $LOG"
candump -tz -e "$IFACE" > "$LOG" 2>&1 &
DUMP_PID=$!

sleep 0.5

echo "[3] Testing CAN IDs: ${IDS[*]}"
echo

START_BUS_OFF=$(ip -details -statistics link show "$IFACE" | grep -o "bus-off [0-9]*" | awk '{print $2}')
START_RESTARTED=$(ip -details -statistics link show "$IFACE" | grep -o "re-started [0-9]*" | awk '{print $2}')
START_DROPPED=$(ip -statistics link show "$IFACE" | awk '/TX:/{getline; print $4}')

FAIL=0
TOTAL=0

for ID in "${IDS[@]}"; do
    echo "Testing ID 0x$ID ..."
    ID_FAIL=0

    for ((i=1; i<=COUNT; i++)); do
        TOTAL=$((TOTAL + 1))

        # RTR frame, avoids sending control payload
        cansend "$IFACE" "${ID}#R" >/dev/null 2>&1
        RET=$?

        if [ "$RET" -ne 0 ]; then
            FAIL=$((FAIL + 1))
            ID_FAIL=$((ID_FAIL + 1))
        fi

        sleep "$GAP"
    done

    echo "  ID 0x$ID send_fail: $ID_FAIL / $COUNT"
done

sleep 0.5

kill "$DUMP_PID" >/dev/null 2>&1

END_STATUS=$(ip -details -statistics link show "$IFACE")

END_BUS_OFF=$(echo "$END_STATUS" | grep -o "bus-off [0-9]*" | awk '{print $2}')
END_RESTARTED=$(echo "$END_STATUS" | grep -o "re-started [0-9]*" | awk '{print $2}')
END_DROPPED=$(ip -statistics link show "$IFACE" | awk '/TX:/{getline; print $4}')

BUS_OFF_DELTA=$((END_BUS_OFF - START_BUS_OFF))
RESTARTED_DELTA=$((END_RESTARTED - START_RESTARTED))
DROPPED_DELTA=$((END_DROPPED - START_DROPPED))

echo
echo "[4] Final CAN status:"
echo "$END_STATUS"
echo

echo "[5] Result summary:"
echo "  Total sends       : $TOTAL"
echo "  Send failures     : $FAIL"
echo "  TX dropped delta  : $DROPPED_DELTA"
echo "  Bus-off delta     : $BUS_OFF_DELTA"
echo "  Restarted delta   : $RESTARTED_DELTA"
echo "  Candump log       : $LOG"
echo

if [ "$BUS_OFF_DELTA" -gt 0 ]; then
    echo "Result: BAD - bus-off occurred."
elif [ "$RESTARTED_DELTA" -gt 0 ]; then
    echo "Result: BAD - CAN controller restarted."
elif [ "$FAIL" -gt 0 ]; then
    echo "Result: UNSTABLE - cansend failures occurred."
elif [ "$DROPPED_DELTA" -gt 0 ]; then
    echo "Result: UNSTABLE - TX dropped increased."
else
    echo "Result: OK - no obvious CAN link error."
fi
