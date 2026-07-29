"""Small structured learned-risk predictor for the research extension."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp
from typing import Any

from .actions import DriveAction, TypedAction
from .policy import Decision
from .predictor import KinematicRiskPredictor, PredictionResult, PredictedPoint, Scene
from .reference_execution import ReferenceExecutionModel, ReferenceOutcome
from .research_benchmark import ResearchBenchmarkCase


@dataclass(frozen=True)
class LogisticState:
    weights: tuple[float, ...]
    means: tuple[float, ...]
    scales: tuple[float, ...]
    training_cases: int


class LearnedRiskPredictor:
    """A dependency-free logistic risk model over structured action features.

    This is intentionally a research baseline, not a complete world model. It
    predicts risk probability while the kinematic rollout remains the visible
    trajectory explanation.
    """

    def __init__(self, state: LogisticState | None = None, threshold: float = 0.5) -> None:
        self.state = state
        self.threshold = threshold
        self.kinematic = KinematicRiskPredictor()

    @classmethod
    def fit(cls, cases: tuple[ResearchBenchmarkCase, ...], epochs: int = 800, learning_rate: float = 0.08) -> "LearnedRiskPredictor":
        if not cases:
            raise ValueError("at least one training case is required")
        raw_features: list[tuple[float, ...]] = []
        labels: list[float] = []
        feature_builder = cls()
        for case in cases:
            raw_features.append(feature_builder._features(case.scene, case.action))
            reference_config = case.reference_config
            outcome = ReferenceExecutionModel(
                command_delay_s=reference_config["command_delay_s"],
                velocity_scale=reference_config["velocity_scale"],
                max_angular_accel=reference_config["max_angular_accel"],
                max_linear_accel=reference_config["max_linear_accel"],
            ).execute(case.scene, case.action).outcome
            labels.append(1.0 if outcome in {ReferenceOutcome.COLLISION, ReferenceOutcome.BOUNDARY_VIOLATION} else 0.0)

        means = tuple(sum(row[index] for row in raw_features) / len(raw_features) for index in range(len(raw_features[0])))
        scales = tuple(max(1e-6, (sum((row[index] - means[index]) ** 2 for row in raw_features) / len(raw_features)) ** 0.5) for index in range(len(raw_features[0])))
        features = [tuple((value - means[index]) / scales[index] for index, value in enumerate(row)) for row in raw_features]
        weights = [0.0] * (len(features[0]) + 1)

        for _ in range(epochs):
            gradients = [0.0] * len(weights)
            for row, label in zip(features, labels):
                probability = _sigmoid(weights[0] + sum(weight * value for weight, value in zip(weights[1:], row)))
                error = probability - label
                gradients[0] += error
                for index, value in enumerate(row, start=1):
                    gradients[index] += error * value
            for index, gradient in enumerate(gradients):
                weights[index] -= learning_rate * gradient / len(features)

        return cls(LogisticState(tuple(weights), means, scales, len(cases)))

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        if scene.pose is None or scene.bounds is None:
            return PredictionResult(Decision.RISK_UNKNOWN, "pose or bounds missing", tuple(), None, self._trace(None, "missing_pose_or_bounds"))
        if scene.observation_age_s is None or scene.observation_age_s > scene.max_observation_age_s:
            return PredictionResult(Decision.RISK_UNKNOWN, "scene observation stale or absent", tuple(), None, self._trace(None, "stale_or_absent_observation"))
        if self.state is None:
            return PredictionResult(Decision.RISK_UNKNOWN, "learned predictor is not fitted", tuple(), None, self._trace(None, "model_not_fitted"))
        if not isinstance(action, DriveAction):
            point = PredictedPoint(0.0, scene.pose.x, scene.pose.y, scene.pose.yaw)
            return PredictionResult(Decision.APPROVED, "non-drive action has no learned motion risk", (point,), None, self._trace(0.0, None))

        features = self._features(scene, action)
        normalized = tuple((value - mean) / scale for value, mean, scale in zip(features, self.state.means, self.state.scales))
        risk_probability = _sigmoid(self.state.weights[0] + sum(weight * value for weight, value in zip(self.state.weights[1:], normalized)))
        rollout = self.kinematic.predict(scene, action)
        decision = Decision.REJECTED if risk_probability >= self.threshold else Decision.APPROVED
        reason = f"learned risk probability {risk_probability:.3f} {'exceeds' if decision == Decision.REJECTED else 'stays below'} threshold"
        trace = self._trace(risk_probability, None)
        trace["features"] = list(features)
        trace["training_cases"] = self.state.training_cases
        return PredictionResult(decision, reason, rollout.trajectory, rollout.min_clearance, trace)

    def _features(self, scene: Scene, action: TypedAction) -> tuple[float, ...]:
        if not isinstance(action, DriveAction):
            return (0.0,) * 7
        rollout = self.kinematic.predict(scene, action)
        predicted_clearance = 1.0 if rollout.min_clearance is None else rollout.min_clearance
        nearest = 1.0
        if scene.pose is not None and scene.obstacles:
            nearest = min(((scene.pose.x - obstacle.x) ** 2 + (scene.pose.y - obstacle.y) ** 2) ** 0.5 - obstacle.radius for obstacle in scene.obstacles)
        return (
            abs(action.vx),
            abs(action.vy),
            abs(action.wz),
            action.duration_s,
            predicted_clearance,
            nearest,
            0.0 if scene.observation_age_s is None else scene.observation_age_s,
        )

    def _trace(self, risk_probability: float | None, risk_trigger: str | None) -> dict[str, Any]:
        return {
            "model": "learned_risk_logistic_v1",
            "model_family": "structured_logistic_risk",
            "risk_probability": risk_probability,
            "risk_trigger": risk_trigger,
            "trajectory_source": "kinematic_explanation_rollout",
        }


def build_default_learned_predictor() -> LearnedRiskPredictor:
    from .research_benchmark import generate_expanded_research_benchmark

    cases = tuple(case for case in generate_expanded_research_benchmark() if case.split == "train")
    return LearnedRiskPredictor.fit(cases)


def _sigmoid(value: float) -> float:
    if value >= 0:
        return 1.0 / (1.0 + exp(-value))
    positive = exp(value)
    return positive / (1.0 + positive)
