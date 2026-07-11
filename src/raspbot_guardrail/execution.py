"""Guarded execution boundary for ROS2-style velocity commands."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .actions import TypedAction
from .backends.ros2_cmd_vel import (
    DEFAULT_CMD_VEL_TOPIC,
    CmdVelPublisher,
    DryRunCmdVelPublisher,
    TwistCommand,
    action_to_twist_commands,
    zero_twist,
)
from .backends.command import DryRunCommandBackend
from .episode import Episode
from .faults import Fault
from .policy import Decision
from .predictor import Scene
from .replay import ReplayEngine, ReplayResult


@dataclass(frozen=True)
class GuardedExecutionResult:
    decision: Decision
    reason: str
    backend: str
    topic: str
    episode: Episode
    published_commands: list[TwistCommand]
    faults: list[dict[str, str]] | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "decision": self.decision.value,
            "reason": self.reason,
            "backend": self.backend,
            "topic": self.topic,
            "published_commands": [command.to_dict() for command in self.published_commands],
            "faults": self.faults or [],
            "guardrail": {
                "name": self.episode.name,
                "predictor": self.episode.metadata.get("predictor", "unknown"),
                "events": [
                    {
                        "index": event.index,
                        "action_type": event.action_type,
                        "static_decision": event.static_decision,
                        "predictive_decision": event.predictive_decision,
                        "final_decision": event.final_decision,
                        "reason": event.reason,
                        "min_clearance": None if event.min_clearance is None else round(event.min_clearance, 3),
                        "risk_trigger": event.model_trace.get("risk_trigger") if event.model_trace else None,
                        "decision_path": event.decision_path,
                        "faults": event.faults,
                    }
                    for event in self.episode.events
                ],
            },
        }


class GuardedCmdVelExecutor:
    """Runs the guardrail before emitting generic ROS2 cmd_vel commands."""

    def __init__(
        self,
        engine: ReplayEngine | None = None,
        publisher: CmdVelPublisher | None = None,
        backend: DryRunCommandBackend | None = None,
        topic: str = DEFAULT_CMD_VEL_TOPIC,
        stop_duration_s: float = 0.2,
    ) -> None:
        self.engine = engine or ReplayEngine()
        self.backend = backend or DryRunCommandBackend(topic=topic, publisher=publisher or DryRunCmdVelPublisher(topic=topic))
        self.publisher = publisher or self.backend.publisher
        self.topic = topic
        self.stop_duration_s = stop_duration_s

    def execute(self, name: str, actions: list[TypedAction], scene: Scene) -> GuardedExecutionResult:
        replay = self.engine.run(name, actions, scene)
        published: list[TwistCommand] = []
        faults: list[dict[str, str]] = []

        try:
            if not self.backend.available():
                raise RuntimeError("command backend unavailable")
            if replay.final_decision == Decision.APPROVED:
                for action in actions:
                    for command in action_to_twist_commands(action, topic=self.topic):
                        self.backend.publish(command)
                        published.append(command)
                if not published or not _is_zero_velocity(published[-1]):
                    terminal_stop = zero_twist(duration_s=self.stop_duration_s, topic=self.topic)
                    self.backend.publish(terminal_stop)
                    published.append(terminal_stop)
            else:
                hold_command = zero_twist(duration_s=self.stop_duration_s, topic=self.topic)
                self.backend.hold(duration_s=self.stop_duration_s)
                published.append(hold_command)
        except Exception as exc:  # noqa: BLE001 - backend is a fault boundary.
            fault = Fault("backend_exception", self.backend.name, str(exc))
            faults.append(fault.to_dict())
            replay = ReplayResult(replay.episode, Decision.RISK_UNKNOWN)
            published = []
            try:
                hold_command = zero_twist(duration_s=self.stop_duration_s, topic=self.topic)
                if self.backend.available():
                    self.backend.hold(duration_s=self.stop_duration_s)
                    published.append(hold_command)
            except Exception as hold_exc:  # noqa: BLE001 - record failed fallback.
                faults.append(Fault("hold_failed", self.backend.name, str(hold_exc)).to_dict())

        return GuardedExecutionResult(
            decision=replay.final_decision,
            reason="backend fault; zero-velocity hold requested" if faults else _execution_reason(replay.episode, replay.final_decision),
            backend="dry_run_cmd_vel",
            topic=self.topic,
            episode=replay.episode,
            published_commands=published,
            faults=faults,
        )


def write_execution_json(path: Path, result: GuardedExecutionResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result.to_dict(), indent=2), encoding="utf-8")


def _execution_reason(episode: Episode, decision: Decision) -> str:
    if decision == Decision.APPROVED:
        return "all actions approved; commands emitted to dry-run cmd_vel backend"
    if not episode.events:
        return "no replay events"
    return episode.events[-1].reason


def _is_zero_velocity(command: TwistCommand) -> bool:
    return command.linear_x == 0.0 and command.linear_y == 0.0 and command.angular_z == 0.0
