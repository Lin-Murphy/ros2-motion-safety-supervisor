"""Dependency-inversion ports for the motion safety supervisor."""

from __future__ import annotations

from typing import Protocol

from .actions import TypedAction
from .episode import ReplayEvent
from .predictor import Scene
from .backends.ros2_cmd_vel import TwistCommand


class ActionSource(Protocol):
    def actions(self) -> list[TypedAction]:
        """Return candidate actions for one execution session."""

    def scene(self) -> Scene:
        """Return the observation context for the candidate actions."""


class CommandBackend(Protocol):
    name: str

    def publish(self, command: TwistCommand) -> None:
        """Publish one command to an execution backend."""

    def hold(self, duration_s: float = 0.2) -> None:
        """Publish a zero-velocity hold."""

    def available(self) -> bool:
        """Return whether the backend can accept commands."""


class EventRecorder(Protocol):
    def record(self, event: ReplayEvent) -> None:
        """Record one structured gateway event."""
