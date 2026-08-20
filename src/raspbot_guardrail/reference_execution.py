"""Independent reference execution model for offline ground truth."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin

from .actions import DriveAction, StopAction, TypedAction
from .predictor import PredictedPoint, Scene


class ReferenceOutcome:
    SAFE = "SAFE"
    COLLISION = "COLLISION"
    BOUNDARY_VIOLATION = "BOUNDARY_VIOLATION"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ReferenceExecutionResult:
    outcome: str
    trajectory: tuple[PredictedPoint, ...]
    min_clearance: float | None
    failure_time_s: float | None
    model_trace: dict[str, float | str | None]


class ReferenceExecutionModel:
    """Executes a command with delay and acceleration-limited motion.

    This is a ground-truth approximation for offline experiments. It is kept
    separate from the predictor so a predictor cannot validate itself using
    the same rollout assumptions.
    """

    def __init__(
        self,
        dt_s: float = 0.05,
        command_delay_s: float = 0.15,
        max_linear_accel: float = 0.8,
        max_linear_deceleration: float | None = None,
        max_angular_accel: float = 1.5,
        velocity_scale: float = 1.0,
        robot_radius: float = 0.18,
    ) -> None:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if command_delay_s < 0.0:
            raise ValueError("command_delay_s must not be negative")
        if max_linear_accel <= 0.0 or max_angular_accel <= 0.0:
            raise ValueError("acceleration limits must be positive")
        if max_linear_deceleration is not None and max_linear_deceleration <= 0.0:
            raise ValueError("max_linear_deceleration must be positive")
        if velocity_scale <= 0.0 or robot_radius < 0.0:
            raise ValueError("velocity_scale must be positive and robot_radius non-negative")
        self.dt_s = dt_s
        self.command_delay_s = command_delay_s
        self.max_linear_accel = max_linear_accel
        self.max_linear_deceleration = max_linear_accel if max_linear_deceleration is None else max_linear_deceleration
        self.max_angular_accel = max_angular_accel
        self.velocity_scale = velocity_scale
        self.robot_radius = robot_radius

    def execute(self, scene: Scene, action: TypedAction) -> ReferenceExecutionResult:
        if scene.pose is None or scene.bounds is None:
            return ReferenceExecutionResult(
                ReferenceOutcome.UNKNOWN,
                tuple(),
                None,
                None,
                self._trace(action, "missing_pose_or_bounds"),
            )
        if isinstance(action, StopAction):
            return self._execute_stop(scene, action)
        if not isinstance(action, DriveAction):
            point = PredictedPoint(0.0, scene.pose.x, scene.pose.y, scene.pose.yaw)
            return ReferenceExecutionResult(
                ReferenceOutcome.SAFE,
                (point,),
                self._clearance(scene, point.x, point.y),
                None,
                self._trace(action, None),
            )

        x, y, yaw = scene.pose.x, scene.pose.y, scene.pose.yaw
        actual_vx = actual_vy = actual_wz = 0.0
        trajectory: list[PredictedPoint] = []
        min_clearance: float | None = None
        horizon_s = self.command_delay_s + action.duration_s
        steps = max(1, int(horizon_s / self.dt_s + 0.999999))

        for step in range(1, steps + 1):
            t = min(step * self.dt_s, horizon_s)
            command_active = t > self.command_delay_s
            target_vx = action.vx * self.velocity_scale if command_active else 0.0
            target_vy = action.vy * self.velocity_scale if command_active else 0.0
            target_wz = action.wz * self.velocity_scale if command_active else 0.0
            actual_vx = self._approach(actual_vx, target_vx, self.max_linear_accel * self.dt_s)
            actual_vy = self._approach(actual_vy, target_vy, self.max_linear_accel * self.dt_s)
            actual_wz = self._approach(actual_wz, target_wz, self.max_angular_accel * self.dt_s)

            x += self.dt_s * (actual_vx * cos(yaw) - actual_vy * sin(yaw))
            y += self.dt_s * (actual_vx * sin(yaw) + actual_vy * cos(yaw))
            yaw += self.dt_s * actual_wz
            point = PredictedPoint(round(t, 6), x, y, yaw)
            trajectory.append(point)

            if not self._inside_bounds(scene, x, y):
                return ReferenceExecutionResult(
                    ReferenceOutcome.BOUNDARY_VIOLATION,
                    tuple(trajectory),
                    min_clearance,
                    point.t,
                    self._trace(action, "boundary_violation"),
                )

            clearance = self._clearance(scene, x, y)
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                if clearance < 0.0:
                    return ReferenceExecutionResult(
                        ReferenceOutcome.COLLISION,
                        tuple(trajectory),
                        min_clearance,
                        point.t,
                        self._trace(action, "collision"),
                    )

        return ReferenceExecutionResult(
            ReferenceOutcome.SAFE,
            tuple(trajectory),
            min_clearance,
            None,
            self._trace(action, None),
        )

    def _execute_stop(self, scene: Scene, action: StopAction) -> ReferenceExecutionResult:
        """Independently integrate delayed deceleration from observed motion."""

        assert scene.pose is not None
        assert scene.bounds is not None
        if scene.base_motion is None:
            return ReferenceExecutionResult(
                ReferenceOutcome.UNKNOWN,
                tuple(),
                None,
                None,
                self._trace(action, "missing_base_motion"),
            )

        x, y, yaw = scene.pose.x, scene.pose.y, scene.pose.yaw
        actual_vx = scene.base_motion.linear_x * self.velocity_scale
        actual_vy = scene.base_motion.linear_y * self.velocity_scale
        actual_wz = scene.base_motion.angular_z * self.velocity_scale
        initial_speed = hypot(actual_vx, actual_vy)
        settle_s = max(action.duration_s, initial_speed / self.max_linear_deceleration + 2.0 * self.dt_s)
        horizon_s = self.command_delay_s + settle_s
        steps = max(1, int(horizon_s / self.dt_s + 0.999999))
        trajectory: list[PredictedPoint] = []
        min_clearance: float | None = None

        for step in range(1, steps + 1):
            t = min(step * self.dt_s, horizon_s)
            if t > self.command_delay_s:
                actual_vx = self._approach(actual_vx, 0.0, self.max_linear_deceleration * self.dt_s)
                actual_vy = self._approach(actual_vy, 0.0, self.max_linear_deceleration * self.dt_s)
                actual_wz = self._approach(actual_wz, 0.0, self.max_angular_accel * self.dt_s)

            x += self.dt_s * (actual_vx * cos(yaw) - actual_vy * sin(yaw))
            y += self.dt_s * (actual_vx * sin(yaw) + actual_vy * cos(yaw))
            yaw += self.dt_s * actual_wz
            point = PredictedPoint(round(t, 6), x, y, yaw)
            trajectory.append(point)

            if not self._inside_bounds(scene, x, y):
                return ReferenceExecutionResult(
                    ReferenceOutcome.BOUNDARY_VIOLATION,
                    tuple(trajectory),
                    min_clearance,
                    point.t,
                    self._trace(action, "boundary_violation"),
                )

            clearance = self._clearance(scene, x, y)
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                if clearance < 0.0:
                    return ReferenceExecutionResult(
                        ReferenceOutcome.COLLISION,
                        tuple(trajectory),
                        min_clearance,
                        point.t,
                        self._trace(action, "collision"),
                    )

            if t > self.command_delay_s and actual_vx == actual_vy == actual_wz == 0.0:
                break

        return ReferenceExecutionResult(
            ReferenceOutcome.SAFE,
            tuple(trajectory),
            min_clearance,
            None,
            self._trace(action, None),
        )

    def _approach(self, current: float, target: float, maximum_step: float) -> float:
        delta = target - current
        if abs(delta) <= maximum_step:
            return target
        return current + maximum_step if delta > 0.0 else current - maximum_step

    def _inside_bounds(self, scene: Scene, x: float, y: float) -> bool:
        assert scene.bounds is not None
        r = self.robot_radius
        return scene.bounds.min_x + r <= x <= scene.bounds.max_x - r and scene.bounds.min_y + r <= y <= scene.bounds.max_y - r

    def _clearance(self, scene: Scene, x: float, y: float) -> float | None:
        if not scene.obstacles:
            return None
        return min(hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius - self.robot_radius for obstacle in scene.obstacles)

    def _trace(self, action: TypedAction, outcome: str | None) -> dict[str, float | str | None]:
        return {
            "model": "reference_acceleration_limited_v1",
            "dt_s": self.dt_s,
            "command_delay_s": self.command_delay_s,
            "max_linear_accel": self.max_linear_accel,
            "max_linear_deceleration": self.max_linear_deceleration,
            "max_angular_accel": self.max_angular_accel,
            "velocity_scale": self.velocity_scale,
            "robot_radius": self.robot_radius,
            "outcome": outcome,
            "action_type": action.action_type,
        }
