"""Runtime command and odometry consistency checks."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .backends.ros2_cmd_vel import TwistCommand
from .faults import Fault
from .policy import Decision


class WatchdogState(str, Enum):
    HEALTHY = "HEALTHY"
    HOLD = "HOLD"


@dataclass(frozen=True)
class OdomSample:
    timestamp_s: float
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class WatchdogConfig:
    max_command_age_s: float = 0.5
    max_odom_age_s: float = 0.5
    velocity_error_tolerance: float = 0.25
    command_scale: float = 1.0


@dataclass(frozen=True)
class WatchdogResult:
    state: WatchdogState
    decision: Decision
    reason: str
    faults: list[Fault]


class RuntimeWatchdog:
    """Checks freshness and command-to-odometry consistency.

    This is a software watchdog, not a safety-rated emergency-stop system.
    """

    def __init__(self, config: WatchdogConfig | None = None) -> None:
        self.config = config or WatchdogConfig()

    def check(self, command: TwistCommand, odom: OdomSample | None, now_s: float) -> WatchdogResult:
        if command.issued_at_s is None:
            return self._hold("command_timestamp_missing", "command has no issuance timestamp")
        if now_s - command.issued_at_s > self.config.max_command_age_s:
            return self._hold("command_stale", "command exceeded freshness limit")
        if odom is None:
            return self._hold("odom_missing", "odometry sample is missing")
        if now_s - odom.timestamp_s > self.config.max_odom_age_s:
            return self._hold("odom_stale", "odometry sample exceeded freshness limit")

        expected = (
            command.linear_x * self.config.command_scale,
            command.linear_y * self.config.command_scale,
            command.angular_z * self.config.command_scale,
        )
        actual = (odom.linear_x, odom.linear_y, odom.angular_z)
        error = max(abs(left - right) for left, right in zip(expected, actual))
        if error > self.config.velocity_error_tolerance:
            return self._hold("odom_command_mismatch", f"maximum velocity error {error:.3f} exceeded tolerance")
        return WatchdogResult(WatchdogState.HEALTHY, Decision.APPROVED, "command and odometry are consistent", [])

    def _hold(self, code: str, detail: str) -> WatchdogResult:
        fault = Fault(code, "runtime_watchdog", detail)
        return WatchdogResult(WatchdogState.HOLD, Decision.RISK_UNKNOWN, detail, [fault])
