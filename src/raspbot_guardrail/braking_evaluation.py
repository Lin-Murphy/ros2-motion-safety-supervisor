"""Compare stopping predictors against independent held-out execution outcomes."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .braking_benchmark import BrakingBenchmarkCase, generate_braking_benchmark
from .braking_predictor import BrakingEnvelopePredictor
from .policy import Decision
from .predictor import KinematicRiskPredictor
from .reference_execution import ReferenceExecutionModel, ReferenceOutcome


@dataclass(frozen=True)
class BrakingCaseResult:
    case_id: str
    split: str
    reference_outcome: str
    baseline_decision: str
    braking_decision: str
    baseline_dangerous_false_negative: bool
    braking_dangerous_false_negative: bool
    baseline_false_reject: bool
    braking_false_reject: bool


@dataclass(frozen=True)
class PredictorMetrics:
    dangerous_false_negatives: int
    false_rejects: int
    unknown_cases: int


@dataclass(frozen=True)
class BrakingEvaluationSummary:
    total: int
    dangerous_cases: int
    metrics: dict[str, PredictorMetrics]
    cases: list[BrakingCaseResult]

    def split_summary(self) -> dict[str, dict[str, dict[str, int]]]:
        result: dict[str, dict[str, dict[str, int]]] = {}
        for split in sorted({case.split for case in self.cases}):
            selected = [case for case in self.cases if case.split == split]
            dangerous = sum(case.reference_outcome in _DANGEROUS_OUTCOMES for case in selected)
            result[split] = {
                "cases": {"total": len(selected), "dangerous": dangerous},
                "kinematic": {
                    "dangerous_false_negatives": sum(case.baseline_dangerous_false_negative for case in selected),
                    "false_rejects": sum(case.baseline_false_reject for case in selected),
                },
                "braking_envelope": {
                    "dangerous_false_negatives": sum(case.braking_dangerous_false_negative for case in selected),
                    "false_rejects": sum(case.braking_false_reject for case in selected),
                },
            }
        return result

    def to_dict(self) -> dict[str, Any]:
        return {
            "evidence_label": "offline_simulated",
            "evaluation_protocol": "independent_discrete_stop_execution_v1",
            "total": self.total,
            "dangerous_cases": self.dangerous_cases,
            "metrics": {name: asdict(metrics) for name, metrics in self.metrics.items()},
            "splits": self.split_summary(),
            "cases": [asdict(case) for case in self.cases],
        }


_DANGEROUS_OUTCOMES = {ReferenceOutcome.COLLISION, ReferenceOutcome.BOUNDARY_VIOLATION}


def run_braking_evaluation(
    cases: tuple[BrakingBenchmarkCase, ...] | None = None,
) -> BrakingEvaluationSummary:
    selected = cases or generate_braking_benchmark()
    baseline = KinematicRiskPredictor()
    braking = BrakingEnvelopePredictor()
    results: list[BrakingCaseResult] = []

    for case in selected:
        config = case.reference_config
        reference = ReferenceExecutionModel(
            command_delay_s=config["command_delay_s"],
            max_linear_deceleration=config["max_linear_deceleration"],
            velocity_scale=config["velocity_scale"],
        ).execute(case.scene, case.action)
        baseline_decision = baseline.predict(case.scene, case.action).decision
        braking_decision = braking.predict(case.scene, case.action).decision
        dangerous = reference.outcome in _DANGEROUS_OUTCOMES
        results.append(
            BrakingCaseResult(
                case_id=case.case_id,
                split=case.split,
                reference_outcome=reference.outcome,
                baseline_decision=baseline_decision.value,
                braking_decision=braking_decision.value,
                baseline_dangerous_false_negative=dangerous and baseline_decision == Decision.APPROVED,
                braking_dangerous_false_negative=dangerous and braking_decision == Decision.APPROVED,
                baseline_false_reject=not dangerous and baseline_decision == Decision.REJECTED,
                braking_false_reject=not dangerous and braking_decision == Decision.REJECTED,
            )
        )

    return BrakingEvaluationSummary(
        total=len(results),
        dangerous_cases=sum(case.reference_outcome in _DANGEROUS_OUTCOMES for case in results),
        metrics={
            "kinematic": _metrics(results, "baseline"),
            "braking_envelope": _metrics(results, "braking"),
        },
        cases=results,
    )


def write_braking_evaluation_json(path: Path, summary: BrakingEvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary.to_dict(), indent=2), encoding="utf-8")


def write_braking_evaluation_markdown(path: Path, summary: BrakingEvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Braking Envelope Evaluation",
        "",
        "- Evidence label: `offline_simulated`",
        "- Protocol: independently implemented discrete delayed-stop execution",
        f"- Cases: {summary.total}",
        f"- Dangerous cases: {summary.dangerous_cases}",
        "",
        "| Predictor | Dangerous false negatives | False rejects | Unknown |",
        "| --- | ---: | ---: | ---: |",
    ]
    for name, metrics in summary.metrics.items():
        lines.append(
            f"| {name} | {metrics.dangerous_false_negatives} | {metrics.false_rejects} | {metrics.unknown_cases} |"
        )
    lines.extend(
        [
            "",
            "| Case | Split | Reference | Kinematic | Braking envelope | Kinematic FN | Braking FN | Kinematic FR | Braking FR |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
    )
    for case in summary.cases:
        lines.append(
            f"| {case.case_id} | {case.split} | {case.reference_outcome} | {case.baseline_decision} | "
            f"{case.braking_decision} | {'YES' if case.baseline_dangerous_false_negative else 'NO'} | "
            f"{'YES' if case.braking_dangerous_false_negative else 'NO'} | "
            f"{'YES' if case.baseline_false_reject else 'NO'} | {'YES' if case.braking_false_reject else 'NO'} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "The analytical predictor does not call or share the discrete reference execution loop.",
            "Held-out delay, deceleration, and velocity-scale settings are used only by the reference model.",
            "This is a deterministic offline comparison, not physical braking validation or certification.",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _metrics(cases: list[BrakingCaseResult], prefix: str) -> PredictorMetrics:
    if prefix == "baseline":
        return PredictorMetrics(
            dangerous_false_negatives=sum(case.baseline_dangerous_false_negative for case in cases),
            false_rejects=sum(case.baseline_false_reject for case in cases),
            unknown_cases=sum(case.baseline_decision == Decision.RISK_UNKNOWN.value for case in cases),
        )
    return PredictorMetrics(
        dangerous_false_negatives=sum(case.braking_dangerous_false_negative for case in cases),
        false_rejects=sum(case.braking_false_reject for case in cases),
        unknown_cases=sum(case.braking_decision == Decision.RISK_UNKNOWN.value for case in cases),
    )
