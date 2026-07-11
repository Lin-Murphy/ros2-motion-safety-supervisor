"""Generic ROS2 /cmd_vel adapter boundary.

This module intentionally avoids importing rclpy in V1 so the core project
remains runnable as plain Python. A later integration step can publish
TwistCommand values as geometry_msgs/msg/Twist.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Protocol

from ..actions import DriveAction, StopAction, TypedAction, WaitAction


DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"


@dataclass(frozen=True)
class TwistCommand:
    topic: str
    linear_x: float
    linear_y: float
    angular_z: float
    duration_s: float
    issued_at_s: float | None = None

    def to_dict(self) -> dict[str, float | str]:
        return asdict(self)


class CmdVelPublisher(Protocol):
    """Minimal publisher boundary for any ROS2 mobile base using cmd_vel."""

    def publish(self, command: TwistCommand) -> None:
        """Publish one velocity command."""


@dataclass
class DryRunCmdVelPublisher:
    """Records commands without requiring rclpy or a running ROS2 graph."""

    topic: str = DEFAULT_CMD_VEL_TOPIC
    published_commands: list[TwistCommand] = field(default_factory=list)

    def publish(self, command: TwistCommand) -> None:
        self.published_commands.append(command)


def to_twist_command(action: DriveAction, topic: str = DEFAULT_CMD_VEL_TOPIC) -> TwistCommand:
    return TwistCommand(
        topic=topic,
        linear_x=action.vx,
        linear_y=action.vy,
        angular_z=action.wz,
        duration_s=action.duration_s,
    )


def zero_twist(duration_s: float = 0.2, topic: str = DEFAULT_CMD_VEL_TOPIC) -> TwistCommand:
    return TwistCommand(
        topic=topic,
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        duration_s=duration_s,
    )


def action_to_twist_commands(action: TypedAction, topic: str = DEFAULT_CMD_VEL_TOPIC) -> tuple[TwistCommand, ...]:
    if isinstance(action, DriveAction):
        return (to_twist_command(action, topic=topic),)
    if isinstance(action, (StopAction, WaitAction)):
        return (zero_twist(duration_s=action.duration_s, topic=topic),)
    raise TypeError(f"unsupported action for cmd_vel conversion: {action!r}")
