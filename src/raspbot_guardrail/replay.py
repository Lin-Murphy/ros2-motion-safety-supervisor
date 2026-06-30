"""Deterministic replay engine."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import TypedAction
from .episode import Episode, ReplayEvent
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
    ) -> None:
        self.policy = policy or StaticPolicy()
        self.predictor = predictor or build_predictor("kinematic")
        self.predictor_name = predictor_name or ("custom" if predictor is not None else "kinematic")

    def run(self, name: str, actions: list[TypedAction], scene: Scene) -> ReplayResult:
        events: list[ReplayEvent] = []
        active_scene = scene
        budget = self.policy.validate_plan_budget(actions)
        if budget.decision != Decision.APPROVED:
            event = ReplayEvent(
                0,
                "plan",
                budget.decision.value,
                Decision.RISK_UNKNOWN.value,
                budget.decision.value,
                budget.reason,
                [],
                None,
                {},
                self._decision_path(
                    ("plan_budget", budget.decision.value, budget.reason),
                    ("prediction", "SKIPPED", "plan budget did not pass"),
                    ("final", budget.decision.value, budget.reason),
                ),
            )
            return ReplayResult(Episode(name, [event], self._metadata(scene)), budget.decision)

        final = Decision.APPROVED
        for index, action in enumerate(actions):
            static = self.policy.validate_action(action)
            if static.decision != Decision.APPROVED:
                final = static.decision
                events.append(
                    ReplayEvent(
                        index,
                        action.action_type,
                        static.decision.value,
                        Decision.RISK_UNKNOWN.value,
                        final.value,
                        static.reason,
                        [],
                        None,
                        {},
                        self._decision_path(
                            ("static_policy", static.decision.value, static.reason),
                            ("prediction", "SKIPPED", "static policy did not pass"),
                            ("final", final.value, static.reason),
                        ),
                    )
                )
                break

            predicted = self.predictor.predict(active_scene, action)
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
                    decision_path=self._decision_path(
                        ("static_policy", static.decision.value, static.reason),
                        ("prediction", predicted.decision.value, predicted.reason),
                        ("final", final.value, predicted.reason),
                    ),
                )
            )
            if final != Decision.APPROVED:
                break
            active_scene = self._scene_after_prediction(active_scene, predicted.trajectory)

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
