import binascii
import struct
import unittest

from control_ui.control_arbitration import (
    H5_CRC_OFFSET,
    H5_FRAME_SIZE,
    arbitrate_ws_frame,
    client_id_bytes,
    normalize_client_id,
    parse_arm_frame,
)


CLIENT_A = "00112233445566778899aabbccddeeff"
CLIENT_B = "102132435465768798a9bacbdcedfe0f"


def make_frame(source="keyboard", owner=CLIENT_A, mode=0, corrupt_crc=False):
    body = b"".join(
        [
            struct.pack("<I", 1),
            struct.pack("<Q", 2),
            struct.pack("<7f", *([-1.0] * 7)),
            struct.pack("<7f", *([-1.0] * 7)),
            struct.pack("<7f", *([-1.0] * 7)),
            struct.pack("<6f", *([-1.0] * 6)),
            struct.pack("<6f", *([-1.0] * 6)),
            struct.pack("<B", mode),
            struct.pack("<16f", *([0.0] * 16)),
            client_id_bytes(owner),
            ("ui:" + source).encode("utf-8").ljust(64, b"\x00"),
        ]
    )
    assert len(body) == H5_CRC_OFFSET
    crc = binascii.crc32(body) & 0xFFFFFFFF
    if corrupt_crc:
        crc ^= 0xFFFFFFFF
    frame = body + struct.pack("<I", crc)
    assert len(frame) == H5_FRAME_SIZE
    return frame


class ControlArbitrationTests(unittest.TestCase):
    def test_client_id_validation(self):
        self.assertEqual(normalize_client_id(CLIENT_A), CLIENT_A)
        self.assertEqual(normalize_client_id("00112233-4455-6677-8899-aabbccddeeff"), CLIENT_A)
        self.assertIsNone(normalize_client_id("0" * 32))
        self.assertIsNone(normalize_client_id("not-an-id"))

    def test_frame_parser(self):
        frame = parse_arm_frame(make_frame("controller_delta"))
        self.assertEqual(frame.source, "controller_delta")
        self.assertEqual(frame.owner_id, CLIENT_A)

    def test_accepts_only_active_owner_and_source(self):
        payload = make_frame("keyboard")
        self.assertTrue(arbitrate_ws_frame(payload, "keyboard", CLIENT_A, True).accepted)
        self.assertEqual(arbitrate_ws_frame(payload, "gamepad", CLIENT_A, True).reason, "inactive_source")
        self.assertEqual(arbitrate_ws_frame(payload, "keyboard", CLIENT_B, True).reason, "wrong_owner")

    def test_stale_and_bad_frames_are_rejected(self):
        payload = make_frame("keyboard")
        self.assertEqual(arbitrate_ws_frame(payload, "keyboard", CLIENT_A, False).reason, "stale_telemetry")
        self.assertEqual(arbitrate_ws_frame(make_frame(corrupt_crc=True), "keyboard", CLIENT_A, True).reason, "bad_crc")
        self.assertEqual(arbitrate_ws_frame(payload[:-1], "keyboard", CLIENT_A, True).reason, "bad_length")

    def test_emergency_bypasses_owner_and_staleness(self):
        payload = make_frame("keyboard", owner=CLIENT_B, mode=0x80)
        decision = arbitrate_ws_frame(payload, "idle", None, False)
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.reason, "emergency_stop")

    def test_external_source_never_forwards_ui_motion(self):
        payload = make_frame("teleop")
        decision = arbitrate_ws_frame(payload, "teleop", CLIENT_A, True)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "external_source_has_no_ui_motion_frame")


if __name__ == "__main__":
    unittest.main()
