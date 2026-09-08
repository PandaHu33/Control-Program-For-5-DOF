import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PARAM_SOURCE = (ROOT / "DataSet_ws/src/mainpulator/src/mainpulator_param.cpp").read_text(encoding="utf-8")
BRIDGE_SOURCE = (ROOT / "DataSet_ws/src/h5_udp_bridge/scripts/h5_udp_bridge_node.py").read_text(encoding="utf-8")


class CanStartupResponseContractTests(unittest.TestCase):
    def test_canopen_responses_are_matched_to_the_request(self):
        self.assertIn("ResponseMatchesLastRequest", PARAM_SOURCE)
        self.assertIn("response_id == request_id - 0x80", PARAM_SOURCE)
        self.assertIn("response.data[1] == last_request.data[1]", PARAM_SOURCE)
        self.assertIn("Timed out waiting for CAN response", PARAM_SOURCE)

    def test_initialization_failures_use_nonzero_exit_status(self):
        self.assertNotIn("state error joint", PARAM_SOURCE)
        self.assertIn("exit(EXIT_FAILURE)", PARAM_SOURCE)

    def test_bridge_callback_state_precedes_subscriber_registration(self):
        init = BRIDGE_SOURCE[BRIDGE_SOURCE.index("    def __init__(self):"):BRIDGE_SOURCE.index("    def set_active_source", BRIDGE_SOURCE.index("    def __init__(self):"))]
        self.assertLess(init.index("self.telemetry_ind = 0"), init.index("rospy.Subscriber("))
        self.assertLess(init.index("self.last_imitation_state_time = 0.0"), init.index("rospy.Subscriber("))


if __name__ == "__main__":
    unittest.main()
