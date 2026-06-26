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
from .episode import Episode
from .policy import Decision
from .predictor import Scene
from .replay import ReplayEngine


@dataclass(frozen=True)
class GuardedExecutionResult:
    decision: Decision
    reason: str
    backend: str
    topic: str
    episode: Episode
    published_commands: list[TwistCommand]

    def to_dict(self) -> dict[str, Any]:
        return {
            "decision": self.decision.value,
            "reason": self.reason,
            "backend": self.backend,
            "topic": self.topic,
            "published_commands": [command.to_dict() for command in self.published_commands],
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
        topic: str = DEFAULT_CMD_VEL_TOPIC,
        stop_duration_s: float = 0.2,
    ) -> None:
        self.engine = engine or ReplayEngine()
        self.publisher = publisher or DryRunCmdVelPublisher(topic=topic)
        self.topic = topic
        self.stop_duration_s = stop_duration_s

    def execute(self, name: str, actions: list[TypedAction], scene: Scene) -> GuardedExecutionResult:
        replay = self.engine.run(name, actions, scene)
        published: list[TwistCommand] = []

        if replay.final_decision == Decision.APPROVED:
            for action in actions:
                for command in action_to_twist_commands(action, topic=self.topic):
                    self.publisher.publish(command)
                    published.append(command)
            if not published or not _is_zero_velocity(published[-1]):
                terminal_stop = zero_twist(duration_s=self.stop_duration_s, topic=self.topic)
                self.publisher.publish(terminal_stop)
                published.append(terminal_stop)
        else:
            hold_command = zero_twist(duration_s=self.stop_duration_s, topic=self.topic)
            self.publisher.publish(hold_command)
            published.append(hold_command)

        return GuardedExecutionResult(
            decision=replay.final_decision,
            reason=_execution_reason(replay.episode, replay.final_decision),
            backend="dry_run_cmd_vel",
            topic=self.topic,
            episode=replay.episode,
            published_commands=published,
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
