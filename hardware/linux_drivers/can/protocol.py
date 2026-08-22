"""Pure CAN protocol codec for the future motor-controller adapter.

The IDs and fields are provisional until the real MCU protocol is available.
Keeping this module dependency-free makes it testable on Windows and Linux.
"""

from dataclasses import dataclass
import struct


COMMAND_BASE_ID = 0x200
FEEDBACK_BASE_ID = 0x180
MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 2


class ProtocolError(ValueError):
    """Raised when a CAN frame is malformed or outside the contract."""


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class MotorFeedback:
    motor_id: int
    position_ticks: int
    velocity_mm_s: int
    status: int


def crc8(data: bytes) -> int:
    """Return CRC-8/ATM for a frame payload."""
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def _check_motor_id(motor_id: int) -> None:
    if motor_id < MOTOR_ID_MIN or motor_id > MOTOR_ID_MAX:
        raise ProtocolError(f"unsupported motor id: {motor_id}")


def encode_velocity_command(motor_id: int, velocity_mm_s: int, sequence: int) -> CanFrame:
    """Encode a signed velocity command into an 8-byte CAN frame."""
    _check_motor_id(motor_id)
    if not -32768 <= velocity_mm_s <= 32767:
        raise ProtocolError("velocity is outside int16 range")
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ProtocolError("sequence is outside uint32 range")
    payload = struct.pack(">hhI", velocity_mm_s, 0, sequence)
    return CanFrame(COMMAND_BASE_ID + motor_id, payload)


def encode_stop_command(motor_id: int, sequence: int) -> CanFrame:
    """Encode an explicit stop command."""
    return encode_velocity_command(motor_id, 0, sequence)


def decode_feedback(frame: CanFrame) -> MotorFeedback:
    """Decode an 8-byte motor feedback frame and validate its CRC."""
    motor_id = frame.arbitration_id - FEEDBACK_BASE_ID
    _check_motor_id(motor_id)
    if len(frame.data) != 8:
        raise ProtocolError("feedback payload must contain 8 bytes")
    if crc8(frame.data[:7]) != frame.data[7]:
        raise ProtocolError("feedback CRC mismatch")
    position_ticks, velocity_mm_s, status = struct.unpack(">ihB", frame.data[:7])
    return MotorFeedback(motor_id, position_ticks, velocity_mm_s, status)


def encode_feedback(feedback: MotorFeedback) -> CanFrame:
    """Encode feedback; this helper is used by the software motor simulator."""
    _check_motor_id(feedback.motor_id)
    if not -2147483648 <= feedback.position_ticks <= 2147483647:
        raise ProtocolError("position is outside int32 range")
    if not -32768 <= feedback.velocity_mm_s <= 32767:
        raise ProtocolError("velocity is outside int16 range")
    if not 0 <= feedback.status <= 255:
        raise ProtocolError("status is outside uint8 range")
    payload = struct.pack(">ihB", feedback.position_ticks, feedback.velocity_mm_s, feedback.status)
    return CanFrame(FEEDBACK_BASE_ID + feedback.motor_id, payload + bytes([crc8(payload)]))
