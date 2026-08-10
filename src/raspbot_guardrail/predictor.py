"""Short-horizon kinematic risk predictor."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin
from typing import Any

from .actions import DriveAction, TypedAction
from .policy import Decision


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class Scene:
    pose: Pose2D | None
    bounds: Bounds | None
    obstacles: tuple[CircleObstacle, ...]
    observation_age_s: float | None = 0.0
    max_observation_age_s: float = 1.0


@dataclass(frozen=True)
class PredictedPoint:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PredictionResult:
    decision: Decision
    reason: str
    trajectory: tuple[PredictedPoint, ...]
    min_clearance: float | None
    model_trace: dict[str, Any]


class KinematicRiskPredictor:
    """Predicts short-horizon planar motion from normalized body commands."""

    def __init__(self, dt_s: float = 0.1, robot_radius: float = 0.18, clearance_margin: float = 0.05) -> None:
        self.dt_s = dt_s
        self.robot_radius = robot_radius
        self.clearance_margin = clearance_margin

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        if scene.pose is None or scene.bounds is None:
            return PredictionResult(Decision.RISK_UNKNOWN, "pose or bounds missing", tuple(), None, self._trace(action, scene, risk_trigger="missing_pose_or_bounds"))
        if scene.observation_age_s is None or scene.observation_age_s > scene.max_observation_age_s:
            return PredictionResult(Decision.RISK_UNKNOWN, "scene observation stale or absent", tuple(), None, self._trace(action, scene, risk_trigger="stale_or_absent_observation"))
        if not isinstance(action, DriveAction):
            point = PredictedPoint(0.0, scene.pose.x, scene.pose.y, scene.pose.yaw)
            return PredictionResult(Decision.APPROVED, "non-drive action has no motion risk", (point,), None, self._trace(action, scene, steps=1))

        trajectory: list[PredictedPoint] = []
        x = scene.pose.x
        y = scene.pose.y
        yaw = scene.pose.yaw
        t = 0.0
        min_clearance: float | None = None

        steps = max(1, int(action.duration_s / self.dt_s))
        for _ in range(steps):
            t += self.dt_s
            x += self.dt_s * (action.vx * cos(yaw) - action.vy * sin(yaw))
            y += self.dt_s * (action.vx * sin(yaw) + action.vy * cos(yaw))
            yaw += self.dt_s * action.wz
            trajectory.append(PredictedPoint(t, x, y, yaw))

            if not self._inside_bounds(scene.bounds, x, y):
                return PredictionResult(
                    Decision.REJECTED,
                    "predicted boundary violation",
                    tuple(trajectory),
                    min_clearance,
                    self._trace(action, scene, steps=steps, risk_trigger="boundary_violation"),
                )

            clearance = self._clearance(scene.obstacles, x, y)
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                if clearance < self.clearance_margin:
                    return PredictionResult(
                        Decision.REJECTED,
                        "predicted obstacle collision or low clearance",
                        tuple(trajectory),
                        min_clearance,
                        self._trace(action, scene, steps=steps, risk_trigger="clearance_below_margin"),
                    )

        return PredictionResult(
            Decision.APPROVED,
            "predicted path stays within scene constraints",
            tuple(trajectory),
            min_clearance,
            self._trace(action, scene, steps=steps),
        )

    def _inside_bounds(self, bounds: Bounds, x: float, y: float) -> bool:
        r = self.robot_radius
        return bounds.min_x + r <= x <= bounds.max_x - r and bounds.min_y + r <= y <= bounds.max_y - r

    def _clearance(self, obstacles: tuple[CircleObstacle, ...], x: float, y: float) -> float | None:
        if not obstacles:
            return None
        return min(hypot(x - obs.x, y - obs.y) - obs.radius - self.robot_radius for obs in obstacles)

    def _trace(
        self,
        action: TypedAction,
        scene: Scene,
        steps: int | None = None,
        risk_trigger: str | None = None,
    ) -> dict[str, Any]:
        trace: dict[str, Any] = {
            "model": "kinematic_unicycle_v1",
            "command_units": "normalized_v1",
            "dt_s": self.dt_s,
            "steps": steps,
            "robot_radius": self.robot_radius,
            "clearance_margin": self.clearance_margin,
            "risk_trigger": risk_trigger,
            "equations": [
                "x_next = x + dt * (vx * cos(yaw) - vy * sin(yaw))",
                "y_next = y + dt * (vx * sin(yaw) + vy * cos(yaw))",
                "yaw_next = yaw + dt * wz",
                "clearance = distance(predicted_point, obstacle_center) - obstacle_radius - robot_radius",
            ],
            "checks": [
                "predicted point remains inside configured bounds",
                "minimum obstacle clearance stays above clearance_margin",
                "pose and scene observation are present and fresh",
            ],
            "scene_evidence": {
                "pose_present": scene.pose is not None,
                "bounds_present": scene.bounds is not None,
                "obstacle_count": len(scene.obstacles),
                "observation_age_s": scene.observation_age_s,
                "max_observation_age_s": scene.max_observation_age_s,
            },
        }
        if isinstance(action, DriveAction):
            trace["input_action"] = {
                "type": "drive",
                "vx": action.vx,
                "vy": action.vy,
                "wz": action.wz,
                "duration_s": action.duration_s,
            }
            trace["horizon_s"] = None if steps is None else round(steps * self.dt_s, 3)
        else:
            trace["input_action"] = {"type": action.action_type, "duration_s": getattr(action, "duration_s", None)}
            trace["horizon_s"] = 0.0
        return trace
