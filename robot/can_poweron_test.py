#!/usr/bin/env python3
"""Safe CANopen power-on acceptance test for the five-joint arm.

The default mode is read-only. ``--mode-test`` first requires a clean SDO
preflight, applies only the same 0x6040=0x0080 Fault Reset used by the deployed
driver when needed, then writes and polls 0x6060. It never enables a drive or
writes target position, target torque, or an NMT reset command.
"""

import argparse
import datetime
import os
import re
import socket
import struct
import subprocess
import sys
import time


CAN_FRAME = struct.Struct("=IB3x8s")
CAN_SFF_MASK = 0x7FF
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FILTER = 1
EXPECTED_MODES = {1: 4, 2: 4, 3: 4, 4: 1, 5: 4}
MODE_NAMES = {
    0: "No mode",
    1: "Profile Position",
    3: "Profile Velocity",
    4: "Profile Torque",
    6: "Homing",
    8: "Cyclic Synchronous Position",
    9: "Cyclic Synchronous Velocity",
    10: "Cyclic Synchronous Torque",
}


class TestFailure(Exception):
    pass


class Reporter:
    def __init__(self, path):
        self.path = path
        self.stream = open(path, "w", encoding="utf-8")

    def line(self, message=""):
        print(message)
        self.stream.write(message + "\n")
        self.stream.flush()

    def close(self):
        self.stream.close()


def run_text(*args):
    return subprocess.check_output(args, stderr=subprocess.STDOUT).decode(
        "utf-8", errors="replace"
    )


def arm_processes():
    found = []
    own_pid = os.getpid()
    proc_root = "/proc"
    for name in os.listdir(proc_root):
        if not name.isdigit() or int(name) == own_pid:
            continue
        try:
            with open(os.path.join(proc_root, name, "cmdline"), "rb") as stream:
                cmd = stream.read().replace(b"\x00", b" ").decode("utf-8", "replace")
        except (OSError, IOError):
            continue
        if "mainpulator/test_node" in cmd or "mainpulatorlaunch.launch" in cmd:
            found.append("pid={} {}".format(name, cmd.strip()))
    return found


def can_status(interface):
    return run_text("ip", "-details", "-statistics", "link", "show", interface)


def can_state(status):
    match = re.search(r"\bstate\s+(ERROR-ACTIVE|ERROR-WARNING|ERROR-PASSIVE|BUS-OFF|STOPPED|SLEEPING)\b", status)
    return match.group(1) if match else "UNKNOWN"


def can_counters(status):
    lines = status.splitlines()
    names = ("re-started", "bus-errors", "arbit-lost", "error-warn", "error-pass", "bus-off")
    for index, line in enumerate(lines[:-1]):
        if all(name in line for name in names):
            values = re.findall(r"\d+", lines[index + 1])
            if len(values) >= len(names):
                return {name: int(value) for name, value in zip(names, values)}
    return {name: 0 for name in names}


def netdev_error_counters(interface):
    base = os.path.join("/sys/class/net", interface, "statistics")
    values = []
    for name in ("rx_errors", "tx_errors"):
        with open(os.path.join(base, name), "r", encoding="ascii") as stream:
            values.append(int(stream.read().strip()))
    return tuple(values)


def format_data(data):
    return "".join("{:02X}".format(value) for value in data)


def statusword_state(value):
    states = (
        (0x004F, 0x0000, "Not ready to switch on"),
        (0x004F, 0x0040, "Switch on disabled"),
        (0x006F, 0x0021, "Ready to switch on"),
        (0x006F, 0x0023, "Switched on"),
        (0x006F, 0x0027, "Operation enabled"),
        (0x006F, 0x0007, "Quick stop active"),
        (0x004F, 0x000F, "Fault reaction active"),
        (0x004F, 0x0008, "Fault"),
    )
    for mask, expected, name in states:
        if value & mask == expected:
            return name
    return "Unknown"


class SdoClient:
    def __init__(self, interface, timeout, retries, gap, reporter):
        self.timeout = timeout
        self.retries = retries
        self.gap = gap
        self.next_request_at = 0.0
        self.reporter = reporter
        self.interface = interface
        self.last_link_errors = netdev_error_counters(interface)
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b"".join(
            struct.pack("=II", 0x580 + node, CAN_SFF_MASK) for node in EXPECTED_MODES
        )
        self.sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filters)
        self.sock.bind((interface,))

    def close(self):
        self.sock.close()

    def _drain(self):
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recv(CAN_FRAME.size)
        except (BlockingIOError, socket.error):
            pass
        finally:
            self.sock.setblocking(True)

    def _pace(self):
        delay = self.next_request_at - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    def _mark_response(self):
        self.next_request_at = time.monotonic() + self.gap

    def _report_link_error_delta(self, context):
        current = netdev_error_counters(self.interface)
        rx_delta = current[0] - self.last_link_errors[0]
        tx_delta = current[1] - self.last_link_errors[1]
        self.last_link_errors = current
        if rx_delta > 0 or tx_delta > 0:
            self.reporter.line(
                "    LINK_ERROR {} rx_errors+{} tx_errors+{}".format(
                    context, rx_delta, tx_delta
                )
            )

    def _exchange(self, node, request, expected_index, expected_subindex, write):
        request_id = 0x600 + node
        response_id = 0x580 + node
        last_reason = "timeout"
        for attempt in range(1, self.retries + 2):
            self._pace()
            context = "J{} 0x{:04X}:{:02X} attempt {}/{}".format(
                node, expected_index, expected_subindex, attempt, self.retries + 1
            )
            self._report_link_error_delta("before " + context)
            self._drain()
            self.sock.send(CAN_FRAME.pack(request_id, 8, request))
            deadline = time.monotonic() + self.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.sock.settimeout(remaining)
                try:
                    raw = self.sock.recv(CAN_FRAME.size)
                except socket.timeout:
                    break
                can_id, dlc, payload = CAN_FRAME.unpack(raw)
                can_id &= CAN_SFF_MASK
                payload = payload[:dlc]
                if can_id != response_id or len(payload) < 4:
                    continue
                index = payload[1] | (payload[2] << 8)
                subindex = payload[3]
                if index != expected_index or subindex != expected_subindex:
                    continue
                self.reporter.line(
                    "    RX {:03X}#{}".format(can_id, format_data(payload))
                )
                self._report_link_error_delta("during " + context)
                if payload[0] == 0x80:
                    abort = struct.unpack("<I", payload[4:8])[0]
                    raise TestFailure(
                        "J{} SDO abort 0x{:08X} at 0x{:04X}:{:02X}".format(
                            node, abort, expected_index, expected_subindex
                        )
                    )
                if write:
                    if payload[0] != 0x60:
                        raise TestFailure(
                            "J{} unexpected SDO write response 0x{:02X}".format(node, payload[0])
                        )
                    self._mark_response()
                    return payload
                command = payload[0]
                if (command & 0xE0) != 0x40 or not (command & 0x02):
                    raise TestFailure(
                        "J{} unsupported SDO upload response 0x{:02X}".format(node, command)
                    )
                size = 4 - ((command >> 2) & 0x03)
                self._mark_response()
                return payload[4 : 4 + size]
            last_reason = "timeout on attempt {}/{}".format(attempt, self.retries + 1)
        raise TestFailure(
            "J{} {} waiting for 0x{:04X}:{:02X}".format(
                node, last_reason, expected_index, expected_subindex
            )
        )

    def upload(self, node, index, subindex=0):
        request = struct.pack("<BHB4x", 0x40, index, subindex)
        self.reporter.line(
            "    TX {:03X}#{}".format(0x600 + node, format_data(request))
        )
        return self._exchange(node, request, index, subindex, False)

    def download_u8(self, node, index, value, subindex=0):
        request = struct.pack("<BHBB3x", 0x2F, index, subindex, value & 0xFF)
        self.reporter.line(
            "    TX {:03X}#{}".format(0x600 + node, format_data(request))
        )
        self._exchange(node, request, index, subindex, True)

    def download_u16(self, node, index, value, subindex=0):
        request = struct.pack("<BHBH2x", 0x2B, index, subindex, value & 0xFFFF)
        self.reporter.line(
            "    TX {:03X}#{}".format(0x600 + node, format_data(request))
        )
        self._exchange(node, request, index, subindex, True)


def as_unsigned(data):
    return int.from_bytes(data, byteorder="little", signed=False)


def as_signed(data):
    return int.from_bytes(data, byteorder="little", signed=True)


def read_joint(client, node):
    statusword = as_unsigned(client.upload(node, 0x6041))
    requested_mode = as_signed(client.upload(node, 0x6060))
    displayed_mode = as_signed(client.upload(node, 0x6061))
    position = as_signed(client.upload(node, 0x6064))
    rated_current = as_unsigned(client.upload(node, 0x6075))
    return {
        "statusword": statusword,
        "requested_mode": requested_mode,
        "displayed_mode": displayed_mode,
        "position": position,
        "rated_current": rated_current,
    }


def poll_mode(client, node, expected, timeout):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = as_signed(client.upload(node, 0x6061))
        if last == expected:
            return last
        time.sleep(0.02)
    raise TestFailure(
        "J{} mode display stayed at {} instead of {} for {:.3f}s".format(
            node, last, expected, timeout
        )
    )


def poll_safe_state(client, node, timeout):
    safe_states = {"Switch on disabled", "Ready to switch on"}
    deadline = time.monotonic() + timeout
    last_word = None
    last_state = None
    while time.monotonic() < deadline:
        last_word = as_unsigned(client.upload(node, 0x6041))
        last_state = statusword_state(last_word)
        if last_state in safe_states:
            return last_word, last_state
        if last_state not in {"Fault", "Fault reaction active", "Not ready to switch on"}:
            raise TestFailure(
                "J{} entered unsafe state 0x{:04X} ({}) after Fault Reset".format(
                    node, last_word, last_state
                )
            )
        time.sleep(0.02)
    raise TestFailure(
        "J{} did not leave fault safely: last status=0x{:04X} ({})".format(
            node, last_word if last_word is not None else 0,
            last_state if last_state is not None else "no response"
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Safe five-joint CANopen power-on acceptance test"
    )
    parser.add_argument("-i", "--interface", default="can0")
    parser.add_argument("--rounds", type=int, default=10,
                        help="statusword reliability reads per joint (default: 10)")
    parser.add_argument("--timeout", type=float, default=0.25,
                        help="timeout for one SDO attempt in seconds (default: 0.25)")
    parser.add_argument("--retries", type=int, default=1,
                        help="retries after an SDO timeout (default: 1)")
    parser.add_argument("--gap", type=float, default=0.02,
                        help="minimum gap between SDO transactions in seconds (default: 0.02; use 0 for stress)")
    parser.add_argument("--mode-test", action="store_true",
                        help="clean-preflight, Fault Reset if needed, then write/poll modes; never enables motion")
    parser.add_argument("--mode-timeout", type=float, default=1.0,
                        help="mode-display poll timeout in seconds (default: 1.0)")
    parser.add_argument("--log-dir", default="/home/night/robot/logs")
    args = parser.parse_args()
    if (args.rounds < 1 or args.timeout <= 0 or args.retries < 0 or
            args.gap < 0 or args.mode_timeout <= 0):
        parser.error("rounds/timeouts must be positive; retries/gap must be non-negative")
    return args


def main():
    args = parse_args()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    try:
        os.makedirs(args.log_dir, exist_ok=True)
        log_path = os.path.join(args.log_dir, "can_poweron_{}.log".format(stamp))
    except OSError:
        log_path = "/tmp/can_poweron_{}.log".format(stamp)
    reporter = Reporter(log_path)
    client = None
    failures = []
    try:
        reporter.line("CAN_POWERON_TEST {}".format(stamp))
        reporter.line("interface={} mode_test={} rounds={} gap_sec={}".format(
            args.interface, args.mode_test, args.rounds, args.gap
        ))
        running = arm_processes()
        if running:
            reporter.line("RESULT: BLOCKED - arm control is already running")
            for process in running:
                reporter.line("  {}".format(process))
            return 3

        before = can_status(args.interface)
        before_state = can_state(before)
        before_counters = can_counters(before)
        reporter.line("CAN before: state={} counters={}".format(before_state, before_counters))
        if before_state != "ERROR-ACTIVE":
            reporter.line("RESULT: BLOCKED - CAN must be ERROR-ACTIVE before testing")
            return 3

        client = SdoClient(
            args.interface, args.timeout, args.retries, args.gap, reporter
        )
        joint_data = {}
        reporter.line("\n[1] Strictly matched SDO reads")
        for node in EXPECTED_MODES:
            reporter.line("  J{}:".format(node))
            try:
                data = read_joint(client, node)
                joint_data[node] = data
                reporter.line(
                    "    status=0x{:04X} ({}) mode_set={} mode_display={} position={} rated_current={}".format(
                        data["statusword"], statusword_state(data["statusword"]),
                        data["requested_mode"], data["displayed_mode"],
                        data["position"], data["rated_current"]
                    )
                )
            except TestFailure as error:
                failures.append(str(error))
                reporter.line("    FAIL: {}".format(error))

        if args.mode_test:
            reporter.line("\n[2] Safe Fault Reset and mode write/poll test")
            safe_states = {"Switch on disabled", "Ready to switch on"}
            missing = [node for node in EXPECTED_MODES if node not in joint_data]
            pre_mode_status = can_status(args.interface)
            pre_mode_state = can_state(pre_mode_status)
            pre_mode_counters = can_counters(pre_mode_status)
            pre_mode_deltas = {
                name: pre_mode_counters.get(name, 0) - before_counters.get(name, 0)
                for name in before_counters
            }
            dirty_names = (
                "bus-errors", "error-warn", "error-pass", "bus-off", "re-started"
            )
            dirty = {
                name: pre_mode_deltas.get(name, 0)
                for name in dirty_names if pre_mode_deltas.get(name, 0) > 0
            }
            if missing or pre_mode_state != "ERROR-ACTIVE" or dirty:
                reason = "preflight state={} counter_delta={} unreadable joints={}".format(
                    pre_mode_state, dirty, missing
                )
                failures.append("mode test blocked: " + reason)
                reporter.line("  BLOCKED: {}".format(reason))
            else:
                reset_failed = False
                for node in EXPECTED_MODES:
                    state = statusword_state(joint_data[node]["statusword"])
                    try:
                        if state == "Fault":
                            reporter.line("  J{} Fault Reset (0x6040=0x0080):".format(node))
                            client.download_u16(node, 0x6040, 0x0080)
                            word, state = poll_safe_state(client, node, args.mode_timeout)
                            joint_data[node]["statusword"] = word
                            reporter.line(
                                "    PASS: status=0x{:04X} ({})".format(word, state)
                            )
                        elif state not in safe_states:
                            raise TestFailure(
                                "J{} unsafe state 0x{:04X} ({}) before mode write".format(
                                    node, joint_data[node]["statusword"], state
                                )
                            )
                    except TestFailure as error:
                        reset_failed = True
                        failures.append(str(error))
                        reporter.line("    FAIL: {}".format(error))

                if not reset_failed:
                    for node, expected in EXPECTED_MODES.items():
                        reporter.line("  J{} -> {} ({}):".format(
                            node, expected, MODE_NAMES.get(expected, "unknown")
                        ))
                        try:
                            client.download_u8(node, 0x6060, expected)
                            actual = poll_mode(client, node, expected, args.mode_timeout)
                            reporter.line("    PASS: 0x6061={}".format(actual))
                        except TestFailure as error:
                            failures.append(str(error))
                            reporter.line("    FAIL: {}".format(error))

        reporter.line("\n[3] SDO reliability test (0x6041)")
        for node in EXPECTED_MODES:
            success = 0
            for _ in range(args.rounds):
                try:
                    client.upload(node, 0x6041)
                    success += 1
                except TestFailure as error:
                    failures.append(str(error))
            reporter.line("  J{}: {}/{}".format(node, success, args.rounds))

        after = can_status(args.interface)
        after_state = can_state(after)
        after_counters = can_counters(after)
        deltas = {
            name: after_counters.get(name, 0) - before_counters.get(name, 0)
            for name in before_counters
        }
        reporter.line("\nCAN after: state={} counter_delta={}".format(after_state, deltas))
        if after_state != "ERROR-ACTIVE":
            failures.append("CAN ended in {}".format(after_state))
        for name in ("bus-errors", "error-warn", "error-pass", "bus-off", "re-started"):
            if deltas.get(name, 0) > 0:
                failures.append("{} increased by {}".format(name, deltas[name]))

        reporter.line("\nLOG={}".format(log_path))
        if failures:
            reporter.line("RESULT: FAIL ({} issue(s))".format(len(failures)))
            for failure in failures:
                reporter.line("  - {}".format(failure))
            return 1
        reporter.line("RESULT: PASS")
        return 0
    except (OSError, subprocess.CalledProcessError) as error:
        reporter.line("RESULT: ERROR - {}".format(error))
        return 2
    finally:
        if client is not None:
            client.close()
        reporter.close()


if __name__ == "__main__":
    sys.exit(main())
