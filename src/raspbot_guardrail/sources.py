"""Action source adapters for replay and future ROS2 inputs."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import TypedAction
from .ports import ActionSource
from .predictor import Scene


@dataclass(frozen=True)
class ReplayActionSource(ActionSource):
    _actions: list[TypedAction]
    _scene: Scene

    def actions(self) -> list[TypedAction]:
        return list(self._actions)

    def scene(self) -> Scene:
        return self._scene
