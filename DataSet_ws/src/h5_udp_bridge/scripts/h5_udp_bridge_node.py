#!/usr/bin/env python3
from __future__ import print_function

import binascii
import json
import select
import socket
import struct
import time

try:
    import Queue as queue
except ImportError:
    import queue

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState


FRAME_SIZE = 293
CRC_OFFSET = 289
MODE_OFFSET = 144
ORDER_OFFSET = 145
NOTE_OFFSET = 225
NOTE_SIZE = 64

STATUS_READY = 1
STATUS_EXECUTING = 2
STATUS_FAILED = 9
STATUS_CANNOT_EXECUTE = 10
STATUS_EMERGENCY = 15


try:
    text_type = unicode
except NameError:
    text_type = str


def crc32(data):
    return binascii.crc32(data) & 0xFFFFFFFF


def to_text(value):
    if value is None:
        return ""
    if isinstance(value, text_type):
        return value
    try:
        return value.decode("utf-8", "ignore")
    except AttributeError:
        return text_type(value)


def fixed_text(value, size):
    text = to_text(value)
    data = text.encode("utf-8")[:size]
    return data.ljust(size, b"\x00")


def float_array(payload, name, size, fill):
    values = payload.get(name)
    if values is None:
        return [fill] * size
    values = list(values)
    if len(values) < size:
        values.extend([fill] * (size - len(values)))
    return [float(v) for v in values[:size]]


def at(values, index, default=0.0):
    try:
        return float(values[index])
    except (IndexError, TypeError, ValueError):
        return default


def stamp_ms(msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    try:
        if stamp and stamp.to_sec() > 0:
            return int(stamp.to_sec() * 1000)
    except Exception:
        pass
    return int(time.time() * 1000)


def parse_command_frame(data, source):
    if len(data) != FRAME_SIZE:
        raise ValueError("bad frame length: %d, expected %d" % (len(data), FRAME_SIZE))

    expected_crc = struct.unpack_from("<I", data, CRC_OFFSET)[0]
    actual_crc = crc32(data[:CRC_OFFSET])
    mode = struct.unpack_from("<B", data, MODE_OFFSET)[0]
    note = data[NOTE_OFFSET:NOTE_OFFSET + NOTE_SIZE].split(b"\x00", 1)[0]

    return {
        "source": "%s:%d" % source,
        "ind": struct.unpack_from("<I", data, 0)[0],
        "time": struct.unpack_from("<Q", data, 4)[0],
        "mode": mode,
        "selector": mode & 0x0F,
        "emergency_stop": bool(mode & 0x80),
        "joint_space": bool(mode & 0x40),
        "elbow_control": bool(mode & 0x20),
        "compliance": bool(mode & 0x10),
        "order": list(struct.unpack_from("<16f", data, ORDER_OFFSET)),
        "note": to_text(note),
        "crc_expected": expected_crc,
        "crc_actual": actual_crc,
        "crc_ok": expected_crc == actual_crc,
        "recv_time": time.time(),
    }


def pack_telemetry_frame(payload, fallback_selector):
    selector = int(payload.get("selector", fallback_selector)) & 0x0F
    if "mode" in payload:
        mode = int(payload.get("mode")) & 0xFF
    else:
        status = int(payload.get("status", STATUS_READY)) & 0x0F
        mode = ((status << 4) | selector) & 0xFF

    parts = [
        struct.pack("<I", int(payload.get("ind", 0)) & 0xFFFFFFFF),
        struct.pack("<Q", int(payload.get("time", int(time.time() * 1000))) & 0xFFFFFFFFFFFFFFFF),
        struct.pack("<7f", *float_array(payload, "angle", 7, -1.0)),
        struct.pack("<7f", *float_array(payload, "current", 7, -1.0)),
        struct.pack("<7f", *float_array(payload, "torque", 7, -1.0)),
        struct.pack("<6f", *float_array(payload, "pose_ee", 6, -1.0)),
        struct.pack("<6f", *float_array(payload, "pose_elbow", 6, -1.0)),
        struct.pack("<B", mode),
        struct.pack("<16f", *float_array(payload, "order", 16, -1.0)),
        b"\xFF" * 16,
        fixed_text(payload.get("note", "h5_udp_bridge"), NOTE_SIZE),
    ]
    body = b"".join(parts)
    return body + struct.pack("<I", crc32(body))


class H5UdpBridge(object):
    def __init__(self):
        self.listen_ip = rospy.get_param("~listen_ip", "0.0.0.0")
        self.listen_port = int(rospy.get_param("~listen_port", 14551))
        self.target_ip = rospy.get_param("~telemetry_target_ip", "")
        self.target_port = int(rospy.get_param("~telemetry_target_port", 14550))
        self.drop_bad_crc = bool(rospy.get_param("~drop_bad_crc", True))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 1.0))
        self.send_heartbeat = bool(rospy.get_param("~send_heartbeat", True))
        self.heartbeat_hz = float(rospy.get_param("~heartbeat_hz", 2.0))
        self.learn_telemetry_target = bool(rospy.get_param("~learn_telemetry_target", True))

        command_topic = rospy.get_param("~command_topic", "/h5/arm_command")
        status_topic = rospy.get_param("~status_topic", "/h5/udp_status")
        telemetry_topic = rospy.get_param("~telemetry_topic", "/arm/h5_telemetry")
        matlab_dataplot_topic = rospy.get_param("~matlab_dataplot_topic", "/Matlab/dataplot")
        imitation_state_topic = rospy.get_param("~imitation_state_topic", "/robot/imitation_state")

        self.command_pub = rospy.Publisher(command_topic, String, queue_size=20)
        self.status_pub = rospy.Publisher(status_topic, String, queue_size=20)
        self.teleop_pub = rospy.Publisher("/pub_joint_state", Imu, queue_size=20)
        self.telemetry_sub = rospy.Subscriber(telemetry_topic, String, self.on_telemetry, queue_size=20)
        self.matlab_dataplot_sub = rospy.Subscriber(
            matlab_dataplot_topic, JointState, self.on_matlab_dataplot, queue_size=20
        )
        self.imitation_state_sub = rospy.Subscriber(
            imitation_state_topic, JointState, self.on_imitation_state, queue_size=20
        )
        self.tx_queue = queue.Queue()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

        self.last_command_time = 0.0
        self.last_timeout_report = 0.0
        self.last_ind = 0
        self.last_selector = 0
        self.telemetry_ind = 0
        self.last_imitation_state_time = 0.0

    def next_telemetry_ind(self):
        value = self.telemetry_ind
        self.telemetry_ind = (self.telemetry_ind + 1) & 0xFFFFFFFF
        return value

    def publish_json(self, publisher, payload):
        publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_status(self, level, event, message, extra=None):
        payload = {
            "level": level,
            "event": event,
            "message": message,
            "stamp": time.time(),
            "last_ind": self.last_ind,
        }
        if extra:
            payload.update(extra)
        self.publish_json(self.status_pub, payload)

    def on_telemetry(self, msg):
        try:
            payload = json.loads(msg.data) if msg.data else {}
            self.tx_queue.put(payload)
        except Exception as exc:
            self.publish_status("ERROR", "BAD_TELEMETRY_JSON", str(exc))

    def on_matlab_dataplot(self, msg):
        if time.time() - self.last_imitation_state_time < 1.0:
            return
        # test_node.cpp publishes /Matlab/dataplot as:
        # position[2]=joint4 actual, [3:6]=expected q, [6:9]=actual q.
        # velocity[0:3]=expected dq, [3:6]=actual dq.
        # effort[0:3]=force feedback, [6:9]=q error.
        position = list(msg.position)
        velocity = list(msg.velocity)
        effort = list(msg.effort)

        actual_q = [at(position, 6), at(position, 7), at(position, 8)]
        expected_q = [at(position, 3), at(position, 4), at(position, 5)]
        actual_dq = [at(velocity, 3), at(velocity, 4), at(velocity, 5)]
        expected_dq = [at(velocity, 0), at(velocity, 1), at(velocity, 2)]
        force_feedback = [at(effort, 0), at(effort, 1), at(effort, 2)]
        q_error = [at(effort, 6), at(effort, 7), at(effort, 8)]

        payload = {
            "ind": self.next_telemetry_ind(),
            "time": stamp_ms(msg),
            "angle": actual_q + [at(position, 2)] + expected_q,
            "current": actual_dq + [0.0] + expected_dq,
            "torque": force_feedback + [0.0] + q_error,
            "pose_ee": [0.0] * 6,
            "pose_elbow": [0.0] * 6,
            "status": STATUS_EXECUTING,
            "note": "Matlab/dataplot",
        }
        self.tx_queue.put(payload)

    def on_imitation_state(self, msg):
        # test_node.cpp publishes /robot/imitation_state as current q/dq:
        # position[0:3]=actual q, position[3]=joint4, position[4]=joint5, velocity[0:3]=actual dq.
        position = list(msg.position)
        velocity = list(msg.velocity)
        self.last_imitation_state_time = time.time()
        payload = {
            "ind": self.next_telemetry_ind(),
            "time": stamp_ms(msg),
            "angle": [at(position, 0), at(position, 1), at(position, 2), at(position, 3), at(position, 4, -1.0), -1.0, -1.0],
            "current": [at(velocity, 0), at(velocity, 1), at(velocity, 2), at(velocity, 3, 0.0), at(velocity, 4, -1.0), -1.0, -1.0],
            "torque": [-1.0] * 7,
            "pose_ee": [0.0] * 6,
            "pose_elbow": [0.0] * 6,
            "status": STATUS_EXECUTING,
            "note": "robot/imitation_state",
        }
        self.tx_queue.put(payload)

    def send_telemetry(self, payload):
        if not self.target_ip:
            return
        frame = pack_telemetry_frame(payload, self.last_selector)
        self.sock.sendto(frame, (self.target_ip, self.target_port))

    def handle_command(self, data, source):
        command = parse_command_frame(data, source)
        if not command["crc_ok"]:
            self.publish_status(
                "WARNING",
                "BAD_CRC",
                "dropped UDP command with bad CRC",
                {"source": command["source"], "ind": command["ind"]},
            )
            if self.drop_bad_crc:
                return

        self.last_command_time = time.time()
        self.last_ind = command["ind"]
        self.last_selector = command["selector"]
        if self.learn_telemetry_target and source and (source[0] != self.target_ip or source[1] != self.target_port):
            self.target_ip = source[0]
            self.target_port = int(source[1])
            self.publish_status(
                "INFO",
                "TELEMETRY_TARGET_LEARNED",
                "updated telemetry target from UDP command source",
                {"target_ip": self.target_ip, "target_port": self.target_port},
            )
        self.publish_json(self.command_pub, command)

        # 转发为期望关节角到 /pub_joint_state
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        # 将UDP传来的 order 数据映射为期望关节角
        imu_msg.orientation.x = command["order"][0]  # expect_q1
        imu_msg.orientation.y = command["order"][1]  # expect_q2
        imu_msg.orientation.z = command["order"][2]  # expect_q3
        imu_msg.orientation.w = command["order"][3]  # expect_q4
        imu_msg.angular_velocity.x = command["order"][4]   # expect_dq1
        imu_msg.angular_velocity.y = command["order"][5]   # expect_dq2
        imu_msg.angular_velocity.z = command["order"][6]   # expect_dq3
        imu_msg.linear_acceleration.x = command["order"][7]  # expect_ddq1
        imu_msg.linear_acceleration.y = command["order"][8]  # expect_ddq2
        imu_msg.linear_acceleration.z = command["order"][9]  # expect_ddq3
        imu_msg.orientation_covariance[0] = at(command["order"], 10, 0.0)  # KB_D gripper: 1 close, 0 idle, -1 open
        self.teleop_pub.publish(imu_msg)

        if command["emergency_stop"]:
            self.publish_status("ERROR", "ESTOP_COMMAND", "received emergency stop command", {"source": command["source"]})

    def spin(self):
        rospy.loginfo("h5_udp_bridge listening on %s:%d", self.listen_ip, self.listen_port)
        if self.target_ip:
            rospy.loginfo("h5_udp_bridge telemetry target %s:%d", self.target_ip, self.target_port)

        next_heartbeat = 0.0
        timeout = 1.0 / max(1.0, self.heartbeat_hz)
        while not rospy.is_shutdown():
            readable, _, _ = select.select([self.sock], [], [], 0.05)
            if readable:
                try:
                    data, source = self.sock.recvfrom(4096)
                    self.handle_command(data, source)
                except Exception as exc:
                    self.publish_status("ERROR", "UDP_RX_ERROR", str(exc))

            while True:
                try:
                    payload = self.tx_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    self.send_telemetry(payload)
                except Exception as exc:
                    self.publish_status("ERROR", "UDP_TX_ERROR", str(exc))

            now = time.time()
            if self.send_heartbeat and now >= next_heartbeat:
                try:
                    self.send_telemetry({"status": STATUS_READY, "note": "ready"})
                except Exception as exc:
                    self.publish_status("ERROR", "HEARTBEAT_TX_ERROR", str(exc))
                next_heartbeat = now + timeout

            if self.last_command_time and now - self.last_command_time > self.command_timeout_sec:
                if now - self.last_timeout_report > 2.0:
                    self.publish_status("WARNING", "COMMAND_TIMEOUT", "no H5 UDP command received recently")
                    self.last_timeout_report = now


if __name__ == "__main__":
    rospy.init_node("h5_udp_bridge")
    H5UdpBridge().spin()
