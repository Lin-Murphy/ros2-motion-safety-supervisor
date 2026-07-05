"""Conservative decision arbitration for the motion safety gateway."""

from __future__ import annotations

from dataclasses import dataclass, field

from .faults import Fault
from .policy import Decision, PolicyResult
from .predictor import PredictionResult


@dataclass(frozen=True)
class ArbiterResult:
    decision: Decision
    reason: str
    fallback: str | None
    decision_path: list[dict[str, str]]
    faults: list[Fault] = field(default_factory=list)


class DecisionArbiter:
    """Combines deterministic policy, prediction, and component faults.

    The arbiter has no ROS2 or backend dependency. Unknown or failed evidence
    never becomes approval; execution layers should apply the fallback hold.
    """

    def arbitrate(
        self,
        policy: PolicyResult,
        prediction: PredictionResult | None = None,
        fault: Fault | None = None,
        prediction_skipped_reason: str | None = None,
    ) -> ArbiterResult:
        path = [
            {
                "stage": "static_policy",
                "result": policy.decision.value,
                "reason": policy.reason,
            }
        ]

        if policy.decision != Decision.APPROVED:
            path.append({
                "stage": "prediction",
                "result": "SKIPPED",
                "reason": prediction_skipped_reason or "static policy did not pass",
            })
            return self._result(policy.decision, policy.reason, path, fault)

        if fault is not None:
            reason = f"{fault.code}: {fault.detail}"
            path.append({"stage": "prediction", "result": Decision.RISK_UNKNOWN.value, "reason": reason})
            return self._result(Decision.RISK_UNKNOWN, reason, path, fault)

        if prediction is None:
            reason = prediction_skipped_reason or "prediction result missing"
            path.append({"stage": "prediction", "result": Decision.RISK_UNKNOWN.value, "reason": reason})
            return self._result(Decision.RISK_UNKNOWN, reason, path, None)

        path.append({
            "stage": "prediction",
            "result": prediction.decision.value,
            "reason": prediction.reason,
        })
        return self._result(prediction.decision, prediction.reason, path, None)

    def _result(
        self,
        decision: Decision,
        reason: str,
        path: list[dict[str, str]],
        fault: Fault | None,
    ) -> ArbiterResult:
        path.append({
            "stage": "final",
            "result": decision.value,
            "reason": reason,
        })
        return ArbiterResult(
            decision=decision,
            reason=reason,
            fallback="zero_velocity_hold" if decision != Decision.APPROVED else None,
            decision_path=path,
            faults=[] if fault is None else [fault],
        )
