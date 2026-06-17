"""Short-horizon kinematic risk predictor."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin

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


class KinematicRiskPredictor:
    """Predicts short-horizon planar motion from normalized body commands."""

    def __init__(self, dt_s: float = 0.1, robot_radius: float = 0.18, clearance_margin: float = 0.05) -> None:
        self.dt_s = dt_s
        self.robot_radius = robot_radius
        self.clearance_margin = clearance_margin

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        if scene.pose is None or scene.bounds is None:
            return PredictionResult(Decision.RISK_UNKNOWN, "pose or bounds missing", tuple(), None)
        if scene.observation_age_s is None or scene.observation_age_s > scene.max_observation_age_s:
            return PredictionResult(Decision.RISK_UNKNOWN, "scene observation stale or absent", tuple(), None)
        if not isinstance(action, DriveAction):
            point = PredictedPoint(0.0, scene.pose.x, scene.pose.y, scene.pose.yaw)
            return PredictionResult(Decision.APPROVED, "non-drive action has no motion risk", (point,), None)

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
                return PredictionResult(Decision.REJECTED, "predicted boundary violation", tuple(trajectory), min_clearance)

            clearance = self._clearance(scene.obstacles, x, y)
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                if clearance < self.clearance_margin:
                    return PredictionResult(Decision.REJECTED, "predicted obstacle collision or low clearance", tuple(trajectory), min_clearance)

        return PredictionResult(Decision.APPROVED, "predicted path stays within scene constraints", tuple(trajectory), min_clearance)

    def _inside_bounds(self, bounds: Bounds, x: float, y: float) -> bool:
        r = self.robot_radius
        return bounds.min_x + r <= x <= bounds.max_x - r and bounds.min_y + r <= y <= bounds.max_y - r

    def _clearance(self, obstacles: tuple[CircleObstacle, ...], x: float, y: float) -> float | None:
        if not obstacles:
            return None
        return min(hypot(x - obs.x, y - obs.y) - obs.radius - self.robot_radius for obs in obstacles)
