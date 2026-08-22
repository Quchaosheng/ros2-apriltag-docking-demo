"""Hardware-independent safety checks for a CAN motor adapter."""

from dataclasses import dataclass
import math
from typing import Optional


class SafetyError(ValueError):
    """Raised when a timestamp is invalid."""


def _validate_time(value: float) -> None:
    if not math.isfinite(value) or value < 0:
        raise SafetyError("time must be a finite, non-negative value")


@dataclass
class MotorSafetyGate:
    """Block motion when the MCU heartbeat or command freshness expires.

    ``now`` values must come from one monotonic clock. The gate deliberately
    has no ROS or CAN dependency so it can be tested before hardware exists.
    """

    heartbeat_timeout_s: float = 0.5
    command_timeout_s: float = 0.25
    last_heartbeat_s: Optional[float] = None
    last_command_s: Optional[float] = None

    def __post_init__(self) -> None:
        if self.heartbeat_timeout_s <= 0 or self.command_timeout_s <= 0:
            raise SafetyError("timeouts must be positive")

    def observe_heartbeat(self, now: float) -> None:
        _validate_time(now)
        self.last_heartbeat_s = now

    def record_command(self, now: float) -> None:
        _validate_time(now)
        self.last_command_s = now

    def heartbeat_ok(self, now: float) -> bool:
        _validate_time(now)
        if self.last_heartbeat_s is None:
            return False
        age = now - self.last_heartbeat_s
        return (
            0 <= age <= self.heartbeat_timeout_s
        )

    def command_fresh(self, now: float) -> bool:
        _validate_time(now)
        if self.last_command_s is None:
            return False
        age = now - self.last_command_s
        return (
            0 <= age <= self.command_timeout_s
        )

    def should_stop(self, now: float) -> bool:
        """Return whether the adapter must issue a stop command."""
        return not self.heartbeat_ok(now) or not self.command_fresh(now)
