"""Typed action schema for mobile-base command plans."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


class ActionParseError(ValueError):
    """Raised when an action plan cannot be parsed into typed actions."""


@dataclass(frozen=True)
class DriveAction:
    vx: float
    vy: float
    wz: float
    duration_s: float

    @property
    def action_type(self) -> str:
        return "drive"


@dataclass(frozen=True)
class StopAction:
    duration_s: float = 0.2

    @property
    def action_type(self) -> str:
        return "stop"


@dataclass(frozen=True)
class WaitAction:
    duration_s: float

    @property
    def action_type(self) -> str:
        return "wait"


TypedAction = DriveAction | StopAction | WaitAction


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ActionParseError(f"{field} must be a number")
    return float(value)


def parse_action(raw: dict[str, Any]) -> TypedAction:
    action_type = raw.get("type")
    if action_type == "drive":
        command = raw.get("command", {})
        if not isinstance(command, dict):
            raise ActionParseError("drive.command must be an object")
        return DriveAction(
            vx=_number(command.get("vx"), "drive.command.vx"),
            vy=_number(command.get("vy", 0.0), "drive.command.vy"),
            wz=_number(command.get("wz"), "drive.command.wz"),
            duration_s=_number(command.get("duration_s"), "drive.command.duration_s"),
        )
    if action_type == "stop":
        duration_s = raw.get("duration_s", 0.2)
        return StopAction(duration_s=_number(duration_s, "stop.duration_s"))
    if action_type == "wait":
        return WaitAction(duration_s=_number(raw.get("duration_s"), "wait.duration_s"))
    raise ActionParseError(f"unknown action type: {action_type!r}")


def parse_plan(raw: Any) -> list[TypedAction]:
    if not isinstance(raw, dict):
        raise ActionParseError("plan must be a JSON object")
    actions = raw.get("actions")
    if not isinstance(actions, list):
        raise ActionParseError("plan.actions must be a list")
    return [parse_action(item) for item in actions]
