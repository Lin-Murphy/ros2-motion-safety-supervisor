"""Optional rclpy node for live ROS2 command supervision.

The safety core remains importable without ROS2. This module is the thin edge
adapter that converts ROS2 messages into the existing typed command and scene
models.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .actions import DriveAction
from .episode import write_episode
from .execution import GuardedCmdVelExecutor
from .predictor import BaseMotionState, Bounds, Pose2D, Scene
from .predictors.registry import build_predictor
from .replay import ReplayEngine
from .backends.ros2_runtime import Ros2CmdVelBackend


def twist_to_action(message: Any, duration_s: float) -> DriveAction:
    """Convert a geometry_msgs/msg/Twist-like object to a typed action."""

    return DriveAction(
        vx=float(message.linear.x),
        vy=float(message.linear.y),
        wz=float(message.angular.z),
        duration_s=duration_s,
    )


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract planar yaw from a ROS quaternion."""

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odometry_to_pose(message: Any) -> Pose2D:
    """Convert a nav_msgs/msg/Odometry-like object to a planar pose."""

    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return Pose2D(
        x=float(position.x),
        y=float(position.y),
        yaw=quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    )


def odometry_to_base_motion(message: Any) -> BaseMotionState:
    """Convert a nav_msgs/msg/Odometry-like message to base-motion evidence."""

    twist = message.twist.twist
    stamp = message.header.stamp
    return BaseMotionState(
        linear_x=float(twist.linear.x),
        linear_y=float(twist.linear.y),
        angular_z=float(twist.angular.z),
        timestamp_s=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
    )


def _require_ros2() -> tuple[Any, Any, Any, Any]:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
    except ImportError as exc:  # pragma: no cover - requires a ROS2 runtime.
        raise RuntimeError(
            "The live ROS2 node requires rclpy, geometry_msgs, and nav_msgs; "
            "run it inside a sourced ROS2 workspace."
        ) from exc
    return rclpy, Twist, Odometry, Node


def main() -> None:
    rclpy, Twist, Odometry, Node = _require_ros2()

    class MotionSafetySupervisorNode(Node):
        def __init__(self) -> None:
            super().__init__("motion_safety_supervisor")
            self.declare_parameter("candidate_topic", "/cmd_vel_candidate")
            self.declare_parameter("safe_topic", "/cmd_vel_safe")
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("predictor", "kinematic")
            self.declare_parameter("command_duration_s", 0.2)
            self.declare_parameter("max_observation_age_s", 0.5)
            self.declare_parameter("expected_command_delay_s", 0.1)
            self.declare_parameter("event_path", "reports/runtime_episode.json")
            self.declare_parameter("min_x", -2.0)
            self.declare_parameter("max_x", 2.0)
            self.declare_parameter("min_y", -1.2)
            self.declare_parameter("max_y", 1.2)

            self._pose: Pose2D | None = None
            self._base_motion: BaseMotionState | None = None
            self._odom_time_s: float | None = None
            self._safe_publisher = self.create_publisher(
                Twist,
                self.get_parameter("safe_topic").value,
                10,
            )
            self._odom_subscription = self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self._on_odom,
                10,
            )
            self._command_subscription = self.create_subscription(
                Twist,
                self.get_parameter("candidate_topic").value,
                self._on_command,
                10,
            )
            predictor_name = str(self.get_parameter("predictor").value)
            engine = ReplayEngine(
                predictor=build_predictor(predictor_name),
                predictor_name=predictor_name,
            )
            self._executor = GuardedCmdVelExecutor(
                engine=engine,
                backend=Ros2CmdVelBackend(self._publish_safe, topic=self.get_parameter("safe_topic").value),
                topic=self.get_parameter("safe_topic").value,
            )
            self.get_logger().info("motion safety supervisor ready")

        def _on_odom(self, message: Any) -> None:
            self._pose = odometry_to_pose(message)
            self._base_motion = odometry_to_base_motion(message)
            self._odom_time_s = self._base_motion.timestamp_s

        def _on_command(self, message: Any) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            observation_age = None if self._odom_time_s is None else max(0.0, now - self._odom_time_s)
            scene = Scene(
                pose=self._pose,
                bounds=Bounds(
                    float(self.get_parameter("min_x").value),
                    float(self.get_parameter("max_x").value),
                    float(self.get_parameter("min_y").value),
                    float(self.get_parameter("max_y").value),
                ),
                obstacles=(),
                observation_age_s=observation_age,
                max_observation_age_s=float(self.get_parameter("max_observation_age_s").value),
                base_motion=self._base_motion,
                expected_command_delay_s=float(self.get_parameter("expected_command_delay_s").value),
            )
            action = twist_to_action(message, float(self.get_parameter("command_duration_s").value))
            result = self._executor.execute(
                f"ros2_{int(now * 1000)}",
                [action],
                scene,
            )
            event_path = Path(str(self.get_parameter("event_path").value))
            write_episode(event_path, result.episode)
            self.get_logger().info(f"decision={result.decision.value} reason={result.reason}")

        def _publish_safe(self, command: Any) -> None:
            message = Twist()
            message.linear.x = command.linear_x
            message.linear.y = command.linear_y
            message.angular.z = command.angular_z
            self._safe_publisher.publish(message)

    rclpy.init()
    node = MotionSafetySupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
