"""Typed motion-action schema and JSON plan parsing."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ActionParseError(ValueError):
    """Raised when an action plan cannot be parsed into typed actions."""


class MotionDomain(str, Enum):
    """Motion capability addressed by a typed action."""

    BASE = "base"
    ARM = "arm"
    COMPOSITE = "composite"
    CONTROL = "control"
    PLAN = "plan"


@dataclass(frozen=True)
class BaseVelocityAction:
    vx: float
    vy: float
    wz: float
    duration_s: float

    @property
    def action_type(self) -> str:
        return "base_velocity"

    @property
    def motion_domain(self) -> MotionDomain:
        return MotionDomain.BASE


@dataclass(frozen=True)
class DriveAction(BaseVelocityAction):
    """Backward-compatible name and wire format for base velocity motion."""

    @property
    def action_type(self) -> str:
        return "drive"


@dataclass(frozen=True)
class StopAction:
    duration_s: float = 0.2

    @property
    def action_type(self) -> str:
        return "stop"

    @property
    def motion_domain(self) -> MotionDomain:
        return MotionDomain.CONTROL


@dataclass(frozen=True)
class WaitAction:
    duration_s: float

    @property
    def action_type(self) -> str:
        return "wait"

    @property
    def motion_domain(self) -> MotionDomain:
        return MotionDomain.CONTROL


@dataclass(frozen=True)
class ArmJointAction:
    """A short-horizon arm command in position or velocity mode.

    Physical joint limits are intentionally deferred to the arm predictor.  The
    static policy validates only the structural contract and generic duration.
    """

    joint_names: tuple[str, ...]
    mode: str
    values: tuple[float, ...]
    duration_s: float

    @property
    def action_type(self) -> str:
        return "arm_joint"

    @property
    def motion_domain(self) -> MotionDomain:
        return MotionDomain.ARM


@dataclass(frozen=True)
class CompositeMotionAction:
    """Synchronized base and arm commands for a future whole-body predictor."""

    base_action: BaseVelocityAction | None
    arm_action: ArmJointAction | None

    @property
    def duration_s(self) -> float:
        """Return the base duration for plan accounting before validation.

        ``StaticPolicy`` verifies that both children exist and have the same
        duration before the action can reach a predictor.
        """

        if self.base_action is not None:
            return self.base_action.duration_s
        if self.arm_action is not None:
            return self.arm_action.duration_s
        return 0.0

    @property
    def action_type(self) -> str:
        return "composite_motion"

    @property
    def motion_domain(self) -> MotionDomain:
        return MotionDomain.COMPOSITE


TypedAction = BaseVelocityAction | StopAction | WaitAction | ArmJointAction | CompositeMotionAction


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ActionParseError(f"{field} must be a number")
    return float(value)


def _command(raw: dict[str, Any], action_type: str) -> dict[str, Any]:
    command = raw.get("command", {})
    if not isinstance(command, dict):
        raise ActionParseError(f"{action_type}.command must be an object")
    return command


def _string_tuple(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ActionParseError(f"{field} must be a list of strings")
    return tuple(value)


def _number_tuple(value: Any, field: str) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise ActionParseError(f"{field} must be a list of numbers")
    return tuple(_number(item, f"{field}[{index}]") for index, item in enumerate(value))


def _parse_base_velocity(raw: dict[str, Any], action_cls: type[BaseVelocityAction], action_type: str) -> BaseVelocityAction:
    command = _command(raw, action_type)
    return action_cls(
        vx=_number(command.get("vx"), f"{action_type}.command.vx"),
        vy=_number(command.get("vy", 0.0), f"{action_type}.command.vy"),
        wz=_number(command.get("wz"), f"{action_type}.command.wz"),
        duration_s=_number(command.get("duration_s"), f"{action_type}.command.duration_s"),
    )


def parse_action(raw: dict[str, Any]) -> TypedAction:
    if not isinstance(raw, dict):
        raise ActionParseError("action must be a JSON object")
    action_type = raw.get("type")
    if action_type == "drive":
        return _parse_base_velocity(raw, DriveAction, "drive")
    if action_type == "base_velocity":
        return _parse_base_velocity(raw, BaseVelocityAction, "base_velocity")
    if action_type == "stop":
        duration_s = raw.get("duration_s", 0.2)
        return StopAction(duration_s=_number(duration_s, "stop.duration_s"))
    if action_type == "wait":
        return WaitAction(duration_s=_number(raw.get("duration_s"), "wait.duration_s"))
    if action_type == "arm_joint":
        command = _command(raw, "arm_joint")
        mode = command.get("mode")
        if mode not in {"position", "velocity"}:
            raise ActionParseError("arm_joint.command.mode must be 'position' or 'velocity'")
        return ArmJointAction(
            joint_names=_string_tuple(command.get("joint_names"), "arm_joint.command.joint_names"),
            mode=mode,
            values=_number_tuple(command.get("values"), "arm_joint.command.values"),
            duration_s=_number(command.get("duration_s"), "arm_joint.command.duration_s"),
        )
    if action_type == "composite_motion":
        base_raw = raw.get("base")
        arm_raw = raw.get("arm")
        if not isinstance(base_raw, dict) or not isinstance(arm_raw, dict):
            raise ActionParseError("composite_motion.base and composite_motion.arm must be action objects")
        base_action = parse_action(base_raw)
        arm_action = parse_action(arm_raw)
        if not isinstance(base_action, BaseVelocityAction):
            raise ActionParseError("composite_motion.base must be a base velocity action")
        if not isinstance(arm_action, ArmJointAction):
            raise ActionParseError("composite_motion.arm must be an arm joint action")
        return CompositeMotionAction(base_action=base_action, arm_action=arm_action)
    raise ActionParseError(f"unknown action type: {action_type!r}")


def parse_plan(raw: Any) -> list[TypedAction]:
    if not isinstance(raw, dict):
        raise ActionParseError("plan must be a JSON object")
    actions = raw.get("actions")
    if not isinstance(actions, list):
        raise ActionParseError("plan.actions must be a list")
    return [parse_action(item) for item in actions]
