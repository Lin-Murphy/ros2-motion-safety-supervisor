"""Command line entry point for replay examples."""

from __future__ import annotations

import argparse
from pathlib import Path

from .actions import parse_plan
from .episode import read_json, write_episode
from .evaluation import run_evaluation, write_evaluation_json, write_evaluation_markdown
from .replay import ReplayEngine
from .report import write_html_report
from .scenario import parse_scene


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m raspbot_guardrail")
    sub = parser.add_subparsers(dest="command", required=True)

    replay = sub.add_parser("replay", help="run a deterministic guardrail replay")
    replay.add_argument("plan", type=Path)
    replay.add_argument("scenario", type=Path)
    replay.add_argument("--episode", type=Path, default=Path("reports/episode.json"))
    replay.add_argument("--html", type=Path, default=Path("reports/replay.html"))

    evaluate = sub.add_parser("evaluate", help="run a manifest of guardrail evaluation cases")
    evaluate.add_argument("manifest", type=Path)
    evaluate.add_argument("--json", type=Path, default=Path("reports/evaluation.json"))
    evaluate.add_argument("--markdown", type=Path, default=Path("reports/evaluation.md"))

    args = parser.parse_args()
    if args.command == "replay":
        actions = parse_plan(read_json(args.plan))
        scene = parse_scene(read_json(args.scenario))
        result = ReplayEngine().run(args.plan.stem, actions, scene)
        write_episode(args.episode, result.episode)
        write_html_report(args.html, result.episode)
        print(f"final_decision={result.final_decision.value}")
        print(f"episode={args.episode}")
        print(f"html={args.html}")
    elif args.command == "evaluate":
        summary = run_evaluation(args.manifest)
        write_evaluation_json(args.json, summary)
        write_evaluation_markdown(args.markdown, summary)
        print(f"passed={summary.passed}/{summary.total}")
        print(f"pass_rate={summary.pass_rate:.0%}")
        print(f"json={args.json}")
        print(f"markdown={args.markdown}")
        if summary.passed != summary.total:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
