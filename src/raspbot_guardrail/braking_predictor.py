"""Conservative analytical stopping-envelope predictor for mobile bases."""

from __future__ import annotations

from math import cos, hypot, isfinite, sin
from typing import Any

from .actions import TypedAction
from .policy import Decision
from .predictor import KinematicRiskPredictor, PredictedPoint, PredictionResult, Scene


class BrakingEnvelopePredictor:
    """Check whether observed base momentum can stop inside the supplied scene.

    This is intentionally an analytical upper-bound calculation, not an
    execution simulator. The independent reference execution model remains the
    only source of offline benchmark outcomes.
    """

    def __init__(
        self,
        minimum_deceleration: float = 0.5,
        robot_radius: float = 0.18,
        clearance_margin: float = 0.05,
        dt_s: float = 0.1,
    ) -> None:
        if minimum_deceleration <= 0.0:
            raise ValueError("minimum_deceleration must be positive")
        if robot_radius < 0.0 or clearance_margin < 0.0:
            raise ValueError("robot_radius and clearance_margin must be non-negative")
        self.minimum_deceleration = minimum_deceleration
        self.robot_radius = robot_radius
        self.clearance_margin = clearance_margin
        self.kinematic = KinematicRiskPredictor(
            dt_s=dt_s,
            robot_radius=robot_radius,
            clearance_margin=clearance_margin,
        )

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        if scene.pose is None or scene.bounds is None:
            return self._unknown(scene, action, "pose or bounds missing", "missing_pose_or_bounds")
        if scene.observation_age_s is None or scene.observation_age_s > scene.max_observation_age_s:
            return self._unknown(scene, action, "scene observation stale or absent", "stale_or_absent_observation")
        if scene.base_motion is None or scene.base_motion.timestamp_s is None:
            return self._unknown(scene, action, "base-motion evidence missing or unstamped", "missing_or_unstamped_base_motion")
        if scene.expected_command_delay_s is None or scene.expected_command_delay_s < 0.0:
            return self._unknown(scene, action, "expected command delay missing or invalid", "missing_or_invalid_command_delay")

        base_motion = scene.base_motion
        values = (
            base_motion.linear_x,
            base_motion.linear_y,
            base_motion.angular_z,
            base_motion.timestamp_s,
            scene.expected_command_delay_s,
        )
        if not all(isfinite(value) for value in values):
            return self._unknown(scene, action, "base-motion evidence contains non-finite values", "non_finite_base_motion")

        baseline = self.kinematic.predict(scene, action)
        if baseline.decision != Decision.APPROVED:
            trace = self._trace(scene, action, baseline, None, "kinematic_baseline_rejected")
            return PredictionResult(
                baseline.decision,
                baseline.reason,
                baseline.trajectory,
                baseline.min_clearance,
                trace,
            )

        yaw = scene.pose.yaw
        world_vx = base_motion.linear_x * cos(yaw) - base_motion.linear_y * sin(yaw)
        world_vy = base_motion.linear_x * sin(yaw) + base_motion.linear_y * cos(yaw)
        speed = hypot(world_vx, world_vy)
        reaction_distance = speed * scene.expected_command_delay_s
        braking_distance = speed * speed / (2.0 * self.minimum_deceleration)
        stopping_distance = reaction_distance + braking_distance

        if speed == 0.0:
            stop_x, stop_y = scene.pose.x, scene.pose.y
        else:
            direction_x, direction_y = world_vx / speed, world_vy / speed
            stop_x = scene.pose.x + direction_x * stopping_distance
            stop_y = scene.pose.y + direction_y * stopping_distance

        envelope = {
            "world_velocity": {"x": world_vx, "y": world_vy},
            "linear_speed": speed,
            "reaction_distance": reaction_distance,
            "braking_distance": braking_distance,
            "stopping_distance": stopping_distance,
            "start": {"x": scene.pose.x, "y": scene.pose.y},
            "stop": {"x": stop_x, "y": stop_y},
        }

        if not self._inside_bounds(scene, stop_x, stop_y):
            trace = self._trace(scene, action, baseline, envelope, "braking_envelope_boundary_violation")
            return PredictionResult(
                Decision.REJECTED,
                "braking envelope leaves configured bounds",
                baseline.trajectory,
                baseline.min_clearance,
                trace,
            )

        envelope_clearance = self._segment_clearance(scene, scene.pose.x, scene.pose.y, stop_x, stop_y)
        min_clearance = self._minimum(baseline.min_clearance, envelope_clearance)
        if envelope_clearance is not None and envelope_clearance < self.clearance_margin:
            trace = self._trace(scene, action, baseline, envelope, "braking_envelope_clearance_below_margin")
            return PredictionResult(
                Decision.REJECTED,
                "braking envelope reaches obstacle clearance margin",
                baseline.trajectory,
                min_clearance,
                trace,
            )

        trace = self._trace(scene, action, baseline, envelope, None)
        return PredictionResult(
            Decision.APPROVED,
            "kinematic path and braking envelope stay within scene constraints",
            baseline.trajectory,
            min_clearance,
            trace,
        )

    def _unknown(self, scene: Scene, action: TypedAction, reason: str, risk_trigger: str) -> PredictionResult:
        return PredictionResult(Decision.RISK_UNKNOWN, reason, tuple(), None, self._trace(scene, action, None, None, risk_trigger))

    def _inside_bounds(self, scene: Scene, x: float, y: float) -> bool:
        assert scene.bounds is not None
        radius = self.robot_radius
        return (
            scene.bounds.min_x + radius <= x <= scene.bounds.max_x - radius
            and scene.bounds.min_y + radius <= y <= scene.bounds.max_y - radius
        )

    def _segment_clearance(self, scene: Scene, x0: float, y0: float, x1: float, y1: float) -> float | None:
        if not scene.obstacles:
            return None
        return min(
            self._point_to_segment_distance(obstacle.x, obstacle.y, x0, y0, x1, y1)
            - obstacle.radius
            - self.robot_radius
            for obstacle in scene.obstacles
        )

    def _point_to_segment_distance(self, px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
        dx, dy = x1 - x0, y1 - y0
        length_squared = dx * dx + dy * dy
        if length_squared == 0.0:
            return hypot(px - x0, py - y0)
        fraction = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / length_squared))
        return hypot(px - (x0 + fraction * dx), py - (y0 + fraction * dy))

    def _minimum(self, first: float | None, second: float | None) -> float | None:
        if first is None:
            return second
        if second is None:
            return first
        return min(first, second)

    def _trace(
        self,
        scene: Scene,
        action: TypedAction,
        baseline: PredictionResult | None,
        envelope: dict[str, Any] | None,
        risk_trigger: str | None,
    ) -> dict[str, Any]:
        base_motion = scene.base_motion
        return {
            "model": "braking_envelope_analytical_v1",
            "model_family": "conservative_analytical_stop_bound",
            "risk_trigger": risk_trigger,
            "equations": [
                "reaction_distance = current_linear_speed * expected_command_delay_s",
                "braking_distance = current_linear_speed^2 / (2 * minimum_deceleration)",
                "stopping_distance = reaction_distance + braking_distance",
            ],
            "checks": [
                "kinematic candidate trajectory stays within the supplied scene",
                "analytical stopping envelope stays inside configured bounds",
                "analytical stopping envelope preserves obstacle clearance margin",
                "base motion, timestamp, delay assumption, pose, and scene observation are available",
            ],
            "scene_evidence": {
                "pose_present": scene.pose is not None,
                "bounds_present": scene.bounds is not None,
                "obstacle_count": len(scene.obstacles),
                "observation_age_s": scene.observation_age_s,
                "max_observation_age_s": scene.max_observation_age_s,
                "base_motion": None if base_motion is None else {
                    "linear_x": base_motion.linear_x,
                    "linear_y": base_motion.linear_y,
                    "angular_z": base_motion.angular_z,
                    "timestamp_s": base_motion.timestamp_s,
                },
            },
            "execution_assumptions": {
                "expected_command_delay_s": scene.expected_command_delay_s,
                "minimum_deceleration": self.minimum_deceleration,
                "robot_radius": self.robot_radius,
                "clearance_margin": self.clearance_margin,
            },
            "braking_envelope": envelope,
            "kinematic_baseline": None if baseline is None else {
                "decision": baseline.decision.value,
                "reason": baseline.reason,
                "risk_trigger": baseline.model_trace.get("risk_trigger"),
            },
            "input_action": {
                "type": action.action_type,
                "duration_s": getattr(action, "duration_s", None),
            },
            "independence_note": "This predictor uses a closed-form stopping-distance bound; offline outcomes remain owned by ReferenceExecutionModel.",
        }
