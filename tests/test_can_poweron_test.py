import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "robot" / "can_poweron_test.py"
SPEC = importlib.util.spec_from_file_location("can_poweron_test", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_statusword_state_operation_enabled():
    assert MODULE.statusword_state(0x0027) == "Operation enabled"


def test_statusword_state_switch_on_disabled_with_extra_bits():
    assert MODULE.statusword_state(0x0640) == "Switch on disabled"


def test_can_state_and_counters():
    status = """can <FD> state ERROR-ACTIVE (berr-counter tx 0 rx 0)
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      14         2          0          21         20         13
    """
    assert MODULE.can_state(status) == "ERROR-ACTIVE"
    assert MODULE.can_counters(status) == {
        "re-started": 14,
        "bus-errors": 2,
        "arbit-lost": 0,
        "error-warn": 21,
        "error-pass": 20,
        "bus-off": 13,
    }


def test_expected_joint_modes_match_runtime_contract():
    assert MODULE.EXPECTED_MODES == {1: 4, 2: 4, 3: 4, 4: 1, 5: 4}


def test_sdo_client_pacing_uses_configured_gap(monkeypatch):
    client = object.__new__(MODULE.SdoClient)
    client.gap = 0.02
    client.next_request_at = 0.0
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: 10.0)
    client._mark_response()
    assert client.next_request_at == 10.02


def test_link_error_delta_reports_only_changes(monkeypatch):
    messages = []
    reporter = type("Reporter", (), {"line": lambda self, value: messages.append(value)})()
    client = object.__new__(MODULE.SdoClient)
    client.interface = "can0"
    client.reporter = reporter
    client.last_link_errors = (10, 20)
    monkeypatch.setattr(MODULE, "netdev_error_counters", lambda interface: (13, 20))
    client._report_link_error_delta("during J4 0x6061:00 attempt 1/2")
    assert messages == [
        "    LINK_ERROR during J4 0x6061:00 attempt 1/2 rx_errors+3 tx_errors+0"
    ]
