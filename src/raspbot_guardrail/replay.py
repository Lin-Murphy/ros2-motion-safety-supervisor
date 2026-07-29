"""Deterministic replay engine."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import TypedAction
from .safety_decision_engine import SafetyDecisionEngine
from .episode import Episode, ReplayEvent
from .faults import Fault
from .policy import Decision, StaticPolicy
from .predictor import Pose2D, PredictedPoint, Scene
from .predictors.base import Predictor
from .predictors.registry import build_predictor


@dataclass(frozen=True)
class ReplayResult:
    episode: Episode
    final_decision: Decision


class ReplayEngine:
    def __init__(
        self,
        policy: StaticPolicy | None = None,
        predictor: Predictor | None = None,
        predictor_name: str | None = None,
        decision_engine: SafetyDecisionEngine | None = None,
    ) -> None:
        self.policy = policy or StaticPolicy()
        self.predictor = predictor or build_predictor("kinematic")
        self.predictor_name = predictor_name or ("custom" if predictor is not None else "kinematic")
        self.decision_engine = decision_engine or SafetyDecisionEngine()

    def run(self, name: str, actions: list[TypedAction], scene: Scene) -> ReplayResult:
        events: list[ReplayEvent] = []
        active_scene = scene
        budget = self.policy.validate_plan_budget(actions)
        if budget.decision != Decision.APPROVED:
            arbitration = self.decision_engine.decide(budget, prediction_skipped_reason="plan budget did not pass")
            event = ReplayEvent(
                0,
                "plan",
                budget.decision.value,
                Decision.RISK_UNKNOWN.value,
                budget.decision.value,
                arbitration.reason,
                [],
                None,
                {},
                arbitration.decision_path,
                [fault.to_dict() for fault in arbitration.faults],
            )
            return ReplayResult(Episode(name, [event], self._metadata(scene)), budget.decision)

        final = Decision.APPROVED
        for index, action in enumerate(actions):
            static = self.policy.validate_action(action)
            if static.decision != Decision.APPROVED:
                arbitration = self.decision_engine.decide(static)
                final = arbitration.decision
                events.append(
                    ReplayEvent(
                        index,
                        action.action_type,
                        static.decision.value,
                        Decision.RISK_UNKNOWN.value,
                        final.value,
                        arbitration.reason,
                        [],
                        None,
                        {},
                        arbitration.decision_path,
                        [fault.to_dict() for fault in arbitration.faults],
                    )
                )
                break

            fault = None
            try:
                predicted = self.predictor.predict(active_scene, action)
            except Exception as exc:  # noqa: BLE001 - fault boundary must contain plugin failures.
                fault = Fault("predictor_exception", self.predictor_name, str(exc))
                predicted = None
            arbitration = self.decision_engine.decide(static, predicted, fault=fault)
            final = arbitration.decision
            trajectory = [
                {"t": point.t, "x": point.x, "y": point.y, "yaw": point.yaw}
                for point in (() if predicted is None else predicted.trajectory)
            ]
            model_trace = {} if predicted is None else predicted.model_trace
            events.append(
                ReplayEvent(
                    index=index,
                    action_type=action.action_type,
                    static_decision=static.decision.value,
                    predictive_decision=Decision.RISK_UNKNOWN.value if predicted is None else predicted.decision.value,
                    final_decision=final.value,
                    reason=arbitration.reason,
                    trajectory=trajectory,
                    min_clearance=None if predicted is None else predicted.min_clearance,
                    model_trace=model_trace,
                    decision_path=arbitration.decision_path,
                    faults=[fault_item.to_dict() for fault_item in arbitration.faults],
                )
            )
            if final != Decision.APPROVED:
                break
            active_scene = self._scene_after_prediction(active_scene, tuple() if predicted is None else predicted.trajectory)

        return ReplayResult(Episode(name, events, self._metadata(scene)), final)

    def _scene_after_prediction(self, scene: Scene, trajectory: tuple[PredictedPoint, ...]) -> Scene:
        if scene.pose is None or not trajectory:
            return scene
        last = trajectory[-1]
        return Scene(
            pose=Pose2D(last.x, last.y, last.yaw),
            bounds=scene.bounds,
            obstacles=scene.obstacles,
            observation_age_s=scene.observation_age_s,
            max_observation_age_s=scene.max_observation_age_s,
        )

    def _metadata(self, scene: Scene) -> dict[str, object]:
        metadata = {
            "stop_event_added": True,
            "evidence_label": "offline_simulated",
            "predictor": self.predictor_name,
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

    def _decision_path(self, *steps: tuple[str, str, str]) -> list[dict[str, str]]:
        return [
            {
                "stage": stage,
                "result": result,
                "reason": reason,
            }
            for stage, result, reason in steps
        ]
