"""Evaluation of predictors against independent reference outcomes."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .policy import Decision
from .predictor import KinematicRiskPredictor
from .reference_execution import ReferenceExecutionModel, ReferenceOutcome
from .research_benchmark import ResearchBenchmarkCase, generate_research_benchmark


@dataclass(frozen=True)
class ResearchCaseResult:
    case_id: str
    split: str
    reference_outcome: str
    predicted_decision: str
    dangerous_false_negative: bool
    false_reject: bool
    unknown: bool
    reference_min_clearance: float | None
    predicted_min_clearance: float | None


@dataclass(frozen=True)
class ResearchEvaluationSummary:
    predictor: str
    total: int
    dangerous_cases: int
    dangerous_false_negatives: int
    false_rejects: int
    unknown_cases: int
    cases: list[ResearchCaseResult]

    @property
    def dangerous_false_negative_rate(self) -> float:
        return self.dangerous_false_negatives / self.dangerous_cases if self.dangerous_cases else 0.0

    @property
    def false_reject_rate(self) -> float:
        safe_cases = self.total - self.dangerous_cases
        return self.false_rejects / safe_cases if safe_cases else 0.0

    @property
    def unknown_rate(self) -> float:
        return self.unknown_cases / self.total if self.total else 0.0

    def split_summary(self) -> dict[str, dict[str, int]]:
        summary: dict[str, dict[str, int]] = {}
        for split in sorted({case.split for case in self.cases}):
            selected = [case for case in self.cases if case.split == split]
            dangerous = sum(case.reference_outcome in {ReferenceOutcome.COLLISION, ReferenceOutcome.BOUNDARY_VIOLATION} for case in selected)
            summary[split] = {
                "cases": len(selected),
                "dangerous_cases": dangerous,
                "dangerous_false_negatives": sum(case.dangerous_false_negative for case in selected),
                "false_rejects": sum(case.false_reject for case in selected),
                "unknown_cases": sum(case.unknown for case in selected),
            }
        return summary

    def to_dict(self) -> dict[str, Any]:
        return {
            "predictor": self.predictor,
            "total": self.total,
            "dangerous_cases": self.dangerous_cases,
            "dangerous_false_negatives": self.dangerous_false_negatives,
            "dangerous_false_negative_rate": self.dangerous_false_negative_rate,
            "false_rejects": self.false_rejects,
            "false_reject_rate": self.false_reject_rate,
            "unknown_cases": self.unknown_cases,
            "unknown_rate": self.unknown_rate,
            "splits": self.split_summary(),
            "cases": [asdict(case) for case in self.cases],
        }


def run_research_evaluation(
    cases: tuple[ResearchBenchmarkCase, ...] | None = None,
    predictor: Any | None = None,
    predictor_name: str = "kinematic",
) -> ResearchEvaluationSummary:
    selected = cases or generate_research_benchmark()
    model = predictor or KinematicRiskPredictor()
    results: list[ResearchCaseResult] = []

    for case in selected:
        config = case.reference_config
        reference = ReferenceExecutionModel(
            command_delay_s=config["command_delay_s"],
            velocity_scale=config["velocity_scale"],
            max_angular_accel=config["max_angular_accel"],
            max_linear_accel=config["max_linear_accel"],
        ).execute(case.scene, case.action)
        prediction = model.predict(case.scene, case.action)
        dangerous = reference.outcome in {ReferenceOutcome.COLLISION, ReferenceOutcome.BOUNDARY_VIOLATION}
        false_negative = dangerous and prediction.decision == Decision.APPROVED
        false_reject = not dangerous and prediction.decision == Decision.REJECTED
        results.append(
            ResearchCaseResult(
                case_id=case.case_id,
                split=case.split,
                reference_outcome=reference.outcome,
                predicted_decision=prediction.decision.value,
                dangerous_false_negative=false_negative,
                false_reject=false_reject,
                unknown=prediction.decision == Decision.RISK_UNKNOWN,
                reference_min_clearance=_round(reference.min_clearance),
                predicted_min_clearance=_round(prediction.min_clearance),
            )
        )

    dangerous_count = sum(case.reference_outcome in {ReferenceOutcome.COLLISION, ReferenceOutcome.BOUNDARY_VIOLATION} for case in results)
    return ResearchEvaluationSummary(
        predictor=predictor_name,
        total=len(results),
        dangerous_cases=dangerous_count,
        dangerous_false_negatives=sum(case.dangerous_false_negative for case in results),
        false_rejects=sum(case.false_reject for case in results),
        unknown_cases=sum(case.unknown for case in results),
        cases=results,
    )


def write_research_evaluation_json(path: Path, summary: ResearchEvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary.to_dict(), indent=2), encoding="utf-8")


def write_research_evaluation_markdown(path: Path, summary: ResearchEvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Research Benchmark Evaluation",
        "",
        f"- Predictor: {summary.predictor}",
        f"- Cases: {summary.total}",
        f"- Dangerous cases: {summary.dangerous_cases}",
        f"- Dangerous false negatives: {summary.dangerous_false_negatives} ({summary.dangerous_false_negative_rate:.1%})",
        f"- False rejects: {summary.false_rejects} ({summary.false_reject_rate:.1%})",
        f"- Unknown cases: {summary.unknown_cases} ({summary.unknown_rate:.1%})",
        "",
        "| Case | Split | Reference | Prediction | Dangerous FN | False reject | Unknown | Reference clearance | Predicted clearance |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for case in summary.cases:
        lines.append(
            "| "
            + " | ".join(
                [
                    case.case_id,
                    case.split,
                    case.reference_outcome,
                    case.predicted_decision,
                    "YES" if case.dangerous_false_negative else "NO",
                    "YES" if case.false_reject else "NO",
                    "YES" if case.unknown else "NO",
                    _clearance(case.reference_min_clearance),
                    _clearance(case.predicted_min_clearance),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Split Summary",
            "",
            "| Split | Cases | Dangerous | Dangerous FN | False rejects | Unknown |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for split, metrics in summary.split_summary().items():
        lines.append(
            f"| {split} | {metrics['cases']} | {metrics['dangerous_cases']} | "
            f"{metrics['dangerous_false_negatives']} | {metrics['false_rejects']} | "
            f"{metrics['unknown_cases']} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "Ground truth comes from the independent reference execution model.",
            "This is a controlled offline benchmark, not hardware validation or a",
            "statistically sufficient generalization study.",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _round(value: float | None) -> float | None:
    return None if value is None else round(value, 4)


def _clearance(value: float | None) -> str:
    return "" if value is None else f"{value:.4f}"
