"""Application-level motion safety supervisor facade."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import TypedAction
from .episode import Episode
from .ports import EventRecorder
from .predictor import Scene
from .replay import ReplayEngine, ReplayResult


@dataclass(frozen=True)
class SupervisorResult:
    replay: ReplayResult

    @property
    def decision(self):
        return self.replay.final_decision

    @property
    def episode(self) -> Episode:
        return self.replay.episode


class MotionSafetySupervisor:
    """Coordinates replayable safety evaluation through one application port."""

    def __init__(self, engine: ReplayEngine | None = None, recorder: EventRecorder | None = None) -> None:
        self.engine = engine or ReplayEngine()
        self.recorder = recorder

    def evaluate(self, name: str, actions: list[TypedAction], scene: Scene) -> SupervisorResult:
        result = self.engine.run(name, actions, scene)
        if self.recorder is not None:
            for event in result.episode.events:
                self.recorder.record(event)
        return SupervisorResult(result)
