import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "DataSet_ws/src/mainpulator/src/test_node.cpp").read_text(encoding="utf-8")
LOW_LEVEL_SOURCE = (ROOT / "DataSet_ws/src/mainpulator/src/mainpulator_control.cpp").read_text(encoding="utf-8")


class JointVelocityDecodeContractTests(unittest.TestCase):
    def test_can_velocity_is_decoded_as_signed_little_endian_int32(self):
        start = SOURCE.index("double DecodeSignedJointVelocity")
        end = SOURCE.index("int main(", start)
        method = SOURCE[start:end]
        self.assertIn("message.dlc < 4", method)
        self.assertIn("std::uint32_t", method)
        self.assertIn("std::int32_t signed_counts", method)
        self.assertIn("message.data[3]) << 24", method)

    def test_low_level_control_file_remains_unchanged(self):
        start = LOW_LEVEL_SOURCE.index("void mainpulator:: current_velocity")
        end = LOW_LEVEL_SOURCE.index("void mainpulator:: ActualCurrent", start)
        self.assertIn("int val = 0", LOW_LEVEL_SOURCE[start:end])


if __name__ == "__main__":
    unittest.main()
