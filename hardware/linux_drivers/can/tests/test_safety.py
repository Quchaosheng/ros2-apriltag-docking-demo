import pytest

from hardware.linux_drivers.can.safety import MotorSafetyGate, SafetyError


def test_gate_starts_stopped_until_heartbeat_and_command_arrive():
    gate = MotorSafetyGate()
    assert gate.should_stop(0.0)
    gate.observe_heartbeat(0.0)
    assert gate.should_stop(0.0)
    gate.record_command(0.0)
    assert not gate.should_stop(0.0)


def test_stale_heartbeat_forces_stop():
    gate = MotorSafetyGate(heartbeat_timeout_s=0.5, command_timeout_s=1.0)
    gate.observe_heartbeat(1.0)
    gate.record_command(1.0)
    assert not gate.should_stop(1.5)
    assert gate.should_stop(1.51)


def test_stale_command_forces_stop_even_with_heartbeat():
    gate = MotorSafetyGate(command_timeout_s=0.25)
    gate.observe_heartbeat(1.0)
    gate.record_command(1.0)
    assert gate.should_stop(1.26)


def test_invalid_time_and_timeout_are_rejected():
    with pytest.raises(SafetyError):
        MotorSafetyGate(heartbeat_timeout_s=0)
    gate = MotorSafetyGate()
    with pytest.raises(SafetyError):
        gate.observe_heartbeat(-1.0)
