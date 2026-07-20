"""ROS2 runtime backend boundary without importing rclpy in the core."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .ros2_cmd_vel import DEFAULT_CMD_VEL_TOPIC, TwistCommand, zero_twist


@dataclass
class Ros2CmdVelBackend:
    """Adapts a ROS2 message publisher to the generic command backend.

    The injected callback is responsible for converting ``TwistCommand`` into
    ``geometry_msgs/msg/Twist``. Keeping that conversion outside this package
    lets the safety core run and test without a ROS2 installation.
    """

    publish_callback: Callable[[TwistCommand], None]
    topic: str = DEFAULT_CMD_VEL_TOPIC
    availability_callback: Callable[[], bool] | None = None
    name: str = "ros2_cmd_vel"

    def publish(self, command: TwistCommand) -> None:
        if not self.available():
            raise RuntimeError("ROS2 command backend unavailable")
        self.publish_callback(command)

    def hold(self, duration_s: float = 0.2) -> None:
        self.publish(zero_twist(duration_s=duration_s, topic=self.topic))

    def available(self) -> bool:
        return True if self.availability_callback is None else self.availability_callback()
