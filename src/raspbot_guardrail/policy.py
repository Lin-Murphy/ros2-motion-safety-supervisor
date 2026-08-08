"""Static command policy and guardrail decisions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite

from .actions import ArmJointAction, BaseVelocityAction, CompositeMotionAction, StopAction, TypedAction, WaitAction


class Decision(str, Enum):
    APPROVED = "APPROVED"
    REJECTED = "REJECTED"
    RISK_UNKNOWN = "RISK_UNKNOWN"


@dataclass(frozen=True)
class PolicyLimits:
    max_abs_vx: float = 0.6
    max_abs_vy: float = 0.6
    max_abs_wz: float = 1.2
    max_action_duration_s: float = 3.0
    max_plan_duration_s: float = 8.0


@dataclass(frozen=True)
class PolicyResult:
    decision: Decision
    reason: str


class StaticPolicy:
    """Rejects malformed or over-budget typed actions before prediction."""

    def __init__(self, limits: PolicyLimits | None = None) -> None:
        self.limits = limits or PolicyLimits()

    def validate_action(self, action: TypedAction) -> PolicyResult:
        if isinstance(action, BaseVelocityAction):
            duration = self._validate_duration(action.duration_s, "drive", positive=True)
            if duration is not None:
                return duration
            if not self._finite(action.vx) or not self._finite(action.vy) or not self._finite(action.wz):
                return PolicyResult(Decision.REJECTED, "drive command values must be finite")
            if abs(action.vx) > self.limits.max_abs_vx:
                return PolicyResult(Decision.REJECTED, "vx exceeds limit")
            if abs(action.vy) > self.limits.max_abs_vy:
                return PolicyResult(Decision.REJECTED, "vy exceeds limit")
            if abs(action.wz) > self.limits.max_abs_wz:
                return PolicyResult(Decision.REJECTED, "wz exceeds limit")
            return PolicyResult(Decision.APPROVED, "static drive policy passed")

        if isinstance(action, (StopAction, WaitAction)):
            duration = self._validate_duration(action.duration_s, action.action_type, positive=False)
            if duration is not None:
                return duration
            return PolicyResult(Decision.APPROVED, "static non-drive policy passed")

        if isinstance(action, ArmJointAction):
            duration = self._validate_duration(action.duration_s, "arm joint", positive=True)
            if duration is not None:
                return duration
            if action.mode not in {"position", "velocity"}:
                return PolicyResult(Decision.REJECTED, "arm joint mode must be position or velocity")
            if not action.joint_names:
                return PolicyResult(Decision.REJECTED, "arm joint names must not be empty")
            if len(action.joint_names) != len(action.values):
                return PolicyResult(Decision.REJECTED, "arm joint names and values must have equal length")
            if any(not isinstance(name, str) or not name.strip() for name in action.joint_names):
                return PolicyResult(Decision.REJECTED, "arm joint names must be non-empty strings")
            if len(set(action.joint_names)) != len(action.joint_names):
                return PolicyResult(Decision.REJECTED, "arm joint names must be unique")
            if any(not self._finite(value) for value in action.values):
                return PolicyResult(Decision.REJECTED, "arm joint values must be finite")
            return PolicyResult(Decision.APPROVED, "static arm joint policy passed")

        if isinstance(action, CompositeMotionAction):
            if action.base_action is None or action.arm_action is None:
                return PolicyResult(Decision.REJECTED, "composite action requires base and arm actions")
            if action.base_action.duration_s != action.arm_action.duration_s:
                return PolicyResult(Decision.REJECTED, "composite base and arm durations must match")
            base = self.validate_action(action.base_action)
            if base.decision != Decision.APPROVED:
                return PolicyResult(base.decision, f"composite base action invalid: {base.reason}")
            arm = self.validate_action(action.arm_action)
            if arm.decision != Decision.APPROVED:
                return PolicyResult(arm.decision, f"composite arm action invalid: {arm.reason}")
            return PolicyResult(Decision.APPROVED, "static composite motion policy passed")

        return PolicyResult(Decision.REJECTED, "unsupported action")

    def validate_plan_budget(self, actions: list[TypedAction]) -> PolicyResult:
        durations = [getattr(action, "duration_s", None) for action in actions]
        if any(not self._finite(value) or float(value) < 0.0 for value in durations):
            return PolicyResult(Decision.REJECTED, "plan contains an invalid duration")
        duration = sum(float(value) for value in durations)
        if duration > self.limits.max_plan_duration_s:
            return PolicyResult(Decision.REJECTED, "plan duration exceeds limit")
        return PolicyResult(Decision.APPROVED, "plan budget passed")

    def _validate_duration(self, value: float, label: str, *, positive: bool) -> PolicyResult | None:
        if not self._finite(value):
            return PolicyResult(Decision.REJECTED, f"{label} duration must be finite")
        if positive and value <= 0:
            return PolicyResult(Decision.REJECTED, f"{label} duration must be positive")
        if not positive and value < 0:
            return PolicyResult(Decision.REJECTED, "duration must be non-negative")
        if value > self.limits.max_action_duration_s:
            return PolicyResult(Decision.REJECTED, f"{label} duration exceeds limit")
        return None

    @staticmethod
    def _finite(value: object) -> bool:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and isfinite(float(value))
