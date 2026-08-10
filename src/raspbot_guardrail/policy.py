"""Static command policy and guardrail decisions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .actions import DriveAction, StopAction, TypedAction, WaitAction


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
        if isinstance(action, DriveAction):
            if action.duration_s <= 0:
                return PolicyResult(Decision.REJECTED, "drive duration must be positive")
            if action.duration_s > self.limits.max_action_duration_s:
                return PolicyResult(Decision.REJECTED, "drive duration exceeds limit")
            if abs(action.vx) > self.limits.max_abs_vx:
                return PolicyResult(Decision.REJECTED, "vx exceeds limit")
            if abs(action.vy) > self.limits.max_abs_vy:
                return PolicyResult(Decision.REJECTED, "vy exceeds limit")
            if abs(action.wz) > self.limits.max_abs_wz:
                return PolicyResult(Decision.REJECTED, "wz exceeds limit")
            return PolicyResult(Decision.APPROVED, "static drive policy passed")

        if isinstance(action, (StopAction, WaitAction)):
            if action.duration_s < 0:
                return PolicyResult(Decision.REJECTED, "duration must be non-negative")
            if action.duration_s > self.limits.max_action_duration_s:
                return PolicyResult(Decision.REJECTED, "duration exceeds limit")
            return PolicyResult(Decision.APPROVED, "static non-drive policy passed")

        return PolicyResult(Decision.REJECTED, "unsupported action")

    def validate_plan_budget(self, actions: list[TypedAction]) -> PolicyResult:
        duration = sum(getattr(action, "duration_s", 0.0) for action in actions)
        if duration > self.limits.max_plan_duration_s:
            return PolicyResult(Decision.REJECTED, "plan duration exceeds limit")
        return PolicyResult(Decision.APPROVED, "plan budget passed")
