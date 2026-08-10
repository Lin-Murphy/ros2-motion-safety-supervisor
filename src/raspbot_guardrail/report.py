"""Standalone HTML report generation."""

from __future__ import annotations

from html import escape
from json import dumps
from pathlib import Path

from .episode import Episode


def write_html_report(path: Path, episode: Episode) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    summary_html = _render_decision_summary(episode)
    trajectory_svg = _render_trajectory_svg(episode)
    decision_path_html = _render_decision_paths(episode)
    trace_html = _render_prediction_trace(episode)
    rows = "\n".join(
        "<tr>"
        f"<td>{event.index}</td>"
        f"<td>{escape(event.action_type)}</td>"
        f"<td>{escape(event.motion_domain)}</td>"
        f"<td>{escape(event.static_decision)}</td>"
        f"<td>{escape(event.predictive_decision)}</td>"
        f"<td>{escape(event.final_decision)}</td>"
        f"<td>{escape(event.reason)}</td>"
        f"<td>{'' if event.min_clearance is None else f'{event.min_clearance:.3f}'}</td>"
        "</tr>"
        for event in episode.events
    )
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{escape(episode.name)} replay report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 32px; line-height: 1.45; }}
    .panel {{ border: 1px solid #d0d0d0; padding: 16px; margin: 20px 0; }}
    .summary-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }}
    .metric {{ border: 1px solid #d8d8d8; background: #fafafa; padding: 12px; }}
    .metric-label {{ color: #555; font-size: 12px; text-transform: uppercase; }}
    .metric-value {{ margin-top: 6px; font-weight: 700; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border: 1px solid #ccc; padding: 8px; text-align: left; }}
    th {{ background: #f2f2f2; }}
    code {{ background: #f6f6f6; padding: 2px 4px; }}
    pre {{ background: #f6f6f6; padding: 12px; overflow-x: auto; }}
  </style>
</head>
<body>
  <h1>{escape(episode.name)} replay report</h1>
  <section class="panel">
    <h2>Decision Summary</h2>
    {summary_html}
  </section>
  <section class="panel">
    <h2>Predicted 2D Trajectory</h2>
    {trajectory_svg}
  </section>
  <section class="panel">
    <h2>Decision Path</h2>
    {decision_path_html}
  </section>
  <section class="panel">
    <h2>Prediction Trace</h2>
    {trace_html}
  </section>
  <table>
    <thead>
      <tr>
        <th>#</th>
        <th>Action</th>
        <th>Domain</th>
        <th>Static</th>
        <th>Predictive</th>
        <th>Final</th>
        <th>Reason</th>
        <th>Min clearance</th>
      </tr>
    </thead>
    <tbody>{rows}</tbody>
  </table>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def _render_decision_summary(episode: Episode) -> str:
    if not episode.events:
        return "<p>No replay events were recorded.</p>"

    final_event = episode.events[-1]
    final_decision = final_event.final_decision
    min_clearance = _minimum_clearance(episode)
    risk_trigger = _first_risk_trigger(episode)
    command_policy = _command_policy(final_decision)
    metrics = [
        ("Final decision", f"<code>{escape(final_decision)}</code>"),
        ("Reason", escape(final_event.reason)),
        ("Predictor", f"<code>{escape(str(episode.metadata.get('predictor', 'unknown')))}</code>"),
        ("Evidence label", f"<code>{escape(str(episode.metadata.get('evidence_label', 'unknown')))}</code>"),
        ("Risk trigger", f"<code>{escape(risk_trigger)}</code>"),
        ("Min clearance", escape("n/a" if min_clearance is None else f"{min_clearance:.3f}")),
        ("ROS2 command policy", escape(command_policy)),
    ]
    items = "\n".join(
        "<div class=\"metric\">"
        f"<div class=\"metric-label\">{escape(label)}</div>"
        f"<div class=\"metric-value\">{value}</div>"
        "</div>"
        for label, value in metrics
    )
    return f"<div class=\"summary-grid\">{items}</div>"


def _minimum_clearance(episode: Episode) -> float | None:
    clearances = [event.min_clearance for event in episode.events if event.min_clearance is not None]
    if not clearances:
        return None
    return min(clearances)


def _first_risk_trigger(episode: Episode) -> str:
    for event in episode.events:
        trigger = event.model_trace.get("risk_trigger") if event.model_trace else None
        if trigger:
            return str(trigger)
    return "none"


def _command_policy(final_decision: str) -> str:
    if final_decision == "APPROVED":
        return "approved actions may pass to the generic /cmd_vel adapter, followed by a terminal stop"
    return "rejected or uncertain actions map to a zero-velocity hold command"


def _render_trajectory_svg(episode: Episode) -> str:
    scene = episode.metadata.get("scene")
    if not isinstance(scene, dict):
        return "<p>No scene metadata available.</p>"
    bounds = scene.get("bounds")
    if not isinstance(bounds, dict):
        return "<p>No bounds available for 2D rendering.</p>"

    min_x = float(bounds["min_x"])
    max_x = float(bounds["max_x"])
    min_y = float(bounds["min_y"])
    max_y = float(bounds["max_y"])
    width = 640
    height = 360
    pad = 28

    def sx(x: float) -> float:
        return pad + (x - min_x) / (max_x - min_x) * (width - 2 * pad)

    def sy(y: float) -> float:
        return height - pad - (y - min_y) / (max_y - min_y) * (height - 2 * pad)

    shapes: list[str] = [
        f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" role="img" aria-label="Predicted trajectory">',
        f'<rect x="{pad}" y="{pad}" width="{width - 2 * pad}" height="{height - 2 * pad}" fill="#fff" stroke="#333" />',
    ]

    obstacles = scene.get("obstacles", [])
    if isinstance(obstacles, list):
        for obstacle in obstacles:
            if isinstance(obstacle, dict):
                ox = sx(float(obstacle["x"]))
                oy = sy(float(obstacle["y"]))
                radius = float(obstacle["radius"]) / (max_x - min_x) * (width - 2 * pad)
                shapes.append(f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="{radius:.1f}" fill="#f8d7da" stroke="#b00020" />')

    pose = scene.get("pose")
    if isinstance(pose, dict):
        px = sx(float(pose["x"]))
        py = sy(float(pose["y"]))
        shapes.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5" fill="#111" />')
        shapes.append(f'<text x="{px + 8:.1f}" y="{py - 8:.1f}" font-size="12">start</text>')

    colors = {"APPROVED": "#0a7f3f", "REJECTED": "#b00020", "RISK_UNKNOWN": "#8a6d00"}
    for event in episode.events:
        if not event.trajectory:
            continue
        points = " ".join(f'{sx(point["x"]):.1f},{sy(point["y"]):.1f}' for point in event.trajectory)
        color = colors.get(str(event.final_decision), "#1f5fbf")
        shapes.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="3" />')
        last = event.trajectory[-1]
        shapes.append(f'<circle cx="{sx(last["x"]):.1f}" cy="{sy(last["y"]):.1f}" r="4" fill="{color}" />')

    shapes.append('<text x="28" y="350" font-size="12">Trajectory uses normalized V1 command units.</text>')
    shapes.append("</svg>")
    return "\n".join(shapes)


def _render_prediction_trace(episode: Episode) -> str:
    chunks: list[str] = []
    for event in episode.events:
        if not event.model_trace:
            continue
        trace_payload = dict(event.model_trace)
        if event.state_trace:
            trace_payload["state_trace"] = event.state_trace
        trace = dumps(trace_payload, indent=2)
        chunks.append(f"<h3>Event #{event.index}: {escape(event.action_type)} [{escape(event.motion_domain)}]</h3>")
        chunks.append(f"<pre>{escape(trace)}</pre>")
    if not chunks:
        return "<p>No predictive model trace was recorded for this episode.</p>"
    return "\n".join(chunks)


def _render_decision_paths(episode: Episode) -> str:
    chunks: list[str] = []
    for event in episode.events:
        if not event.decision_path:
            continue
        chunks.append(f"<h3>Event #{event.index}: {escape(event.action_type)} [{escape(event.motion_domain)}]</h3>")
        chunks.append("<ol>")
        for step in event.decision_path:
            stage = escape(step.get("stage", "unknown"))
            result = escape(step.get("result", "unknown"))
            reason = escape(step.get("reason", ""))
            chunks.append(f"<li><strong>{stage}</strong>: <code>{result}</code> - {reason}</li>")
        chunks.append("</ol>")
    if not chunks:
        return "<p>No decision path was recorded for this episode.</p>"
    return "\n".join(chunks)
