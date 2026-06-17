"""Future ROS2 /cmd_vel adapter boundary.

This module intentionally avoids importing rclpy in V1 so the core project
remains runnable as plain Python. A later integration step can map approved
DriveAction values to geometry_msgs/msg/Twist.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..actions import DriveAction


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float
    linear_y: float
    angular_z: float
    duration_s: float


def to_twist_command(action: DriveAction) -> TwistCommand:
    return TwistCommand(
        linear_x=action.vx,
        linear_y=action.vy,
        angular_z=action.wz,
        duration_s=action.duration_s,
    )
