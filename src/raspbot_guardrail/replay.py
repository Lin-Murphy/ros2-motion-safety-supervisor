"""Deterministic replay engine."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import TypedAction
from .episode import Episode, ReplayEvent
from .policy import Decision, StaticPolicy
from .predictor import KinematicRiskPredictor, Scene


@dataclass(frozen=True)
class ReplayResult:
    episode: Episode
    final_decision: Decision


class ReplayEngine:
    def __init__(self, policy: StaticPolicy | None = None, predictor: KinematicRiskPredictor | None = None) -> None:
        self.policy = policy or StaticPolicy()
        self.predictor = predictor or KinematicRiskPredictor()

    def run(self, name: str, actions: list[TypedAction], scene: Scene) -> ReplayResult:
        events: list[ReplayEvent] = []
        budget = self.policy.validate_plan_budget(actions)
        if budget.decision != Decision.APPROVED:
            event = ReplayEvent(0, "plan", budget.decision.value, Decision.RISK_UNKNOWN.value, budget.decision.value, budget.reason, [], None, {})
            return ReplayResult(Episode(name, [event], self._metadata(scene)), budget.decision)

        final = Decision.APPROVED
        for index, action in enumerate(actions):
            static = self.policy.validate_action(action)
            if static.decision != Decision.APPROVED:
                final = static.decision
                events.append(ReplayEvent(index, action.action_type, static.decision.value, Decision.RISK_UNKNOWN.value, final.value, static.reason, [], None, {}))
                break

            predicted = self.predictor.predict(scene, action)
            final = predicted.decision
            trajectory = [
                {"t": point.t, "x": point.x, "y": point.y, "yaw": point.yaw}
                for point in predicted.trajectory
            ]
            events.append(
                ReplayEvent(
                    index=index,
                    action_type=action.action_type,
                    static_decision=static.decision.value,
                    predictive_decision=predicted.decision.value,
                    final_decision=final.value,
                    reason=predicted.reason,
                    trajectory=trajectory,
                    min_clearance=predicted.min_clearance,
                    model_trace=predicted.model_trace,
                )
            )
            if final != Decision.APPROVED:
                break

        return ReplayResult(Episode(name, events, self._metadata(scene)), final)

    def _metadata(self, scene: Scene) -> dict[str, object]:
        metadata = {
            "stop_event_added": True,
            "evidence_label": "offline_simulated",
            "scene": {
                "pose": None if scene.pose is None else {"x": scene.pose.x, "y": scene.pose.y, "yaw": scene.pose.yaw},
                "bounds": None if scene.bounds is None else {
                    "min_x": scene.bounds.min_x,
                    "max_x": scene.bounds.max_x,
                    "min_y": scene.bounds.min_y,
                    "max_y": scene.bounds.max_y,
                },
                "obstacles": [
                    {"x": obstacle.x, "y": obstacle.y, "radius": obstacle.radius}
                    for obstacle in scene.obstacles
                ],
            },
        }
        return metadata
