import pytest

from hardware.linux_drivers.can.protocol import (
    CanFrame,
    MotorFeedback,
    ProtocolError,
    decode_feedback,
    encode_feedback,
    encode_stop_command,
    encode_velocity_command,
)


def test_velocity_command_has_expected_id_and_payload():
    frame = encode_velocity_command(1, -120, 7)
    assert frame.arbitration_id == 0x201
    assert frame.data.hex() == "ff88000000000007"


def test_stop_command_is_zero_velocity():
    frame = encode_stop_command(2, 9)
    assert frame.arbitration_id == 0x202
    assert frame.data[:2] == b"\x00\x00"


def test_feedback_round_trip():
    source = MotorFeedback(2, 123456, -80, 3)
    assert decode_feedback(encode_feedback(source)) == source


def test_bad_length_is_rejected():
    with pytest.raises(ProtocolError, match="8 bytes"):
        decode_feedback(CanFrame(0x181, b"\x00" * 7))


def test_bad_crc_is_rejected():
    frame = encode_feedback(MotorFeedback(1, 1, 2, 0))
    corrupted = CanFrame(frame.arbitration_id, frame.data[:-1] + b"\x00")
    with pytest.raises(ProtocolError, match="CRC"):
        decode_feedback(corrupted)


def test_unsupported_motor_id_is_rejected():
    with pytest.raises(ProtocolError, match="unsupported motor id"):
        encode_velocity_command(3, 0, 0)
