"""Evaluation harness for replay cases."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .actions import parse_plan
from .episode import read_json
from .policy import Decision
from .replay import ReplayEngine
from .scenario import parse_scene


@dataclass(frozen=True)
class EvaluationCase:
    case_id: str
    plan_path: Path
    scenario_path: Path
    expected_decision: Decision
    category: str
    purpose: str


@dataclass(frozen=True)
class EvaluationCaseResult:
    case_id: str
    category: str
    purpose: str
    decision_event_index: int | None
    expected_decision: str
    actual_decision: str
    static_decision: str
    predictive_decision: str
    risk_trigger: str | None
    reason: str
    min_clearance: float | None
    passed: bool


@dataclass(frozen=True)
class EvaluationSummary:
    predictor: str
    total: int
    passed: int
    pass_rate: float
    cases: list[EvaluationCaseResult]

    def to_dict(self) -> dict[str, Any]:
        return {
            "predictor": self.predictor,
            "total": self.total,
            "passed": self.passed,
            "pass_rate": self.pass_rate,
            "cases": [asdict(case) for case in self.cases],
        }


def load_cases(manifest_path: Path) -> list[EvaluationCase]:
    raw = read_json(manifest_path)
    if not isinstance(raw, dict):
        raise ValueError("evaluation manifest must be a JSON object")
    cases = raw.get("cases")
    if not isinstance(cases, list):
        raise ValueError("evaluation manifest must contain a cases list")

    base = manifest_path.parent
    return [_parse_case(base, item) for item in cases]


def run_evaluation(manifest_path: Path, engine: ReplayEngine | None = None) -> EvaluationSummary:
    replay_engine = engine or ReplayEngine()
    results: list[EvaluationCaseResult] = []

    for case in load_cases(manifest_path):
        actions = parse_plan(read_json(case.plan_path))
        scene = parse_scene(read_json(case.scenario_path))
        replay = replay_engine.run(case.case_id, actions, scene)
        event = _decision_event(replay.episode.events)
        actual = replay.final_decision.value

        results.append(
            EvaluationCaseResult(
                case_id=case.case_id,
                category=case.category,
                purpose=case.purpose,
                decision_event_index=None if event is None else event.index,
                expected_decision=case.expected_decision.value,
                actual_decision=actual,
                static_decision="" if event is None else event.static_decision,
                predictive_decision="" if event is None else event.predictive_decision,
                risk_trigger=_risk_trigger(event),
                reason="" if event is None else event.reason,
                min_clearance=_round_optional(None if event is None else event.min_clearance),
                passed=actual == case.expected_decision.value,
            )
        )

    passed_count = sum(1 for result in results if result.passed)
    total = len(results)
    pass_rate = 0.0 if total == 0 else passed_count / total
    predictor_name = getattr(replay_engine, "predictor_name", "unknown")
    return EvaluationSummary(predictor=predictor_name, total=total, passed=passed_count, pass_rate=pass_rate, cases=results)


def write_evaluation_json(path: Path, summary: EvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary.to_dict(), indent=2), encoding="utf-8")


def write_evaluation_markdown(path: Path, summary: EvaluationSummary) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Guardrail Evaluation Report",
        "",
        f"- Predictor: {summary.predictor}",
        f"- Cases: {summary.total}",
        f"- Passed: {summary.passed}",
        f"- Pass rate: {summary.pass_rate:.0%}",
        "",
        "| Case | Category | Event | Expected | Actual | Static | Predictive | Trigger | Result | Min clearance | Reason |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for case in summary.cases:
        result = "PASS" if case.passed else "FAIL"
        lines.append(
            "| "
            + " | ".join(
                [
                    _cell(case.case_id),
                    _cell(case.category),
                    "" if case.decision_event_index is None else str(case.decision_event_index),
                    _cell(case.expected_decision),
                    _cell(case.actual_decision),
                    _cell(case.static_decision),
                    _cell(case.predictive_decision),
                    "" if case.risk_trigger is None else _cell(case.risk_trigger),
                    result,
                    _clearance_cell(case.min_clearance),
                    _cell(case.reason),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Scope",
            "",
            "This report validates deterministic V1 guardrail behaviour against explicit",
            "offline replay cases. It checks decision consistency, not real-world",
            "physical calibration.",
            "",
            "The collision-risk case is especially useful because the command passes the",
            "static policy but is rejected by the predictive layer.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _parse_case(base: Path, raw: Any) -> EvaluationCase:
    if not isinstance(raw, dict):
        raise ValueError("evaluation case must be an object")
    return EvaluationCase(
        case_id=_required_string(raw, "id"),
        plan_path=_resolve(base, _required_string(raw, "plan")),
        scenario_path=_resolve(base, _required_string(raw, "scenario")),
        expected_decision=Decision(_required_string(raw, "expected_decision")),
        category=_required_string(raw, "category"),
        purpose=_required_string(raw, "purpose"),
    )


def _resolve(base: Path, value: str) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return base / path


def _required_string(raw: dict[str, Any], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"evaluation case field {key!r} must be a non-empty string")
    return value


def _cell(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ")


def _round_optional(value: float | None) -> float | None:
    if value is None:
        return None
    return round(value, 3)


def _risk_trigger(event: Any) -> str | None:
    if event is None or not event.model_trace:
        return None
    trigger = event.model_trace.get("risk_trigger")
    if trigger is None:
        return None
    return str(trigger)


def _decision_event(events: list[Any]) -> Any:
    for event in events:
        if event.final_decision != Decision.APPROVED.value:
            return event
    if not events:
        return None
    return events[0]


def _clearance_cell(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.3f}"
