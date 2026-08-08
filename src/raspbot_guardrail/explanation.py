"""Human-readable guardrail explanations."""

from __future__ import annotations

from pathlib import Path

from .execution import GuardedExecutionResult
from .policy import Decision


def format_explanation(result: GuardedExecutionResult) -> str:
    predictor = result.episode.metadata.get("predictor", "unknown")
    lines = [
        f"# Guardrail Explanation: {result.episode.name}",
        "",
        f"- Decision: {result.decision.value}",
        f"- Reason: {result.reason}",
        f"- Predictor: {predictor}",
        f"- Topic: {result.topic}",
        f"- Dry-run commands: {len(result.published_commands)}",
        "",
        "## Decision Path",
        "",
    ]

    for event in result.episode.events:
        lines.append(f"### Event {event.index}: {event.action_type} [{event.motion_domain}]")
        if event.decision_path:
            for step_index, step in enumerate(event.decision_path, start=1):
                lines.append(
                    f"{step_index}. {step['stage']}: {step['result']} - {step['reason']}"
                )
        else:
            lines.append("No decision path recorded.")

        if event.min_clearance is not None or event.model_trace:
            lines.append("")
            lines.append("Risk details:")
            if event.min_clearance is not None:
                lines.append(f"- min_clearance: {event.min_clearance:.3f}")
            if event.model_trace:
                risk_trigger = event.model_trace.get("risk_trigger")
                clearance_margin = event.model_trace.get("clearance_margin")
                horizon_s = event.model_trace.get("horizon_s")
                lines.append(f"- risk_trigger: {risk_trigger}")
                lines.append(f"- clearance_margin: {clearance_margin}")
                lines.append(f"- horizon_s: {horizon_s}")
        lines.append("")

    lines.extend(
        [
            "## Dry-Run Command Policy",
            "",
        ]
    )
    if result.decision == Decision.APPROVED:
        lines.append(
            "The guardrail approved the action sequence, so the dry-run backend "
            "emitted motion commands and ensured a final zero-velocity stop."
        )
    else:
        lines.append(
            "The guardrail did not approve the action sequence, so the dry-run "
            "backend emitted only a zero-velocity hold command."
        )

    lines.append("")
    lines.append("## Generated Commands")
    lines.append("")
    for index, command in enumerate(result.published_commands):
        lines.append(
            f"{index}. topic={command.topic}, linear_x={command.linear_x:.3f}, "
            f"linear_y={command.linear_y:.3f}, angular_z={command.angular_z:.3f}, "
            f"duration_s={command.duration_s:.3f}"
        )
    lines.append("")
    return "\n".join(lines)


def write_explanation(path: Path, result: GuardedExecutionResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(format_explanation(result), encoding="utf-8")
