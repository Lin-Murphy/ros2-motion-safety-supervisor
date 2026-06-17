"""Command line entry point for replay examples."""

from __future__ import annotations

import argparse
from pathlib import Path

from .actions import parse_plan
from .episode import read_json, write_episode
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


if __name__ == "__main__":
    main()
