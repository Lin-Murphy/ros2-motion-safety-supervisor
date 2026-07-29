"""Command line entry point for replay examples."""

from __future__ import annotations

import argparse
from pathlib import Path

from .actions import parse_plan
from .episode import read_json, write_episode
from .evaluation import run_evaluation, write_evaluation_json, write_evaluation_markdown
from .execution import GuardedCmdVelExecutor, write_execution_json
from .explanation import format_explanation, write_explanation
from .predictors.registry import available_predictors, build_predictor
from .replay import ReplayEngine
from .report import write_html_report
from .research_benchmark import generate_expanded_research_benchmark, generate_research_benchmark
from .research_evaluation import run_research_evaluation, write_research_evaluation_json, write_research_evaluation_markdown
from .scenario import parse_scene


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m raspbot_guardrail")
    sub = parser.add_subparsers(dest="command", required=True)

    replay = sub.add_parser("replay", help="run a deterministic guardrail replay")
    replay.add_argument("plan", type=Path)
    replay.add_argument("scenario", type=Path)
    replay.add_argument("--episode", type=Path, default=Path("reports/episode.json"))
    replay.add_argument("--html", type=Path, default=Path("reports/replay.html"))
    replay.add_argument("--predictor", choices=available_predictors(), default="kinematic")

    evaluate = sub.add_parser("evaluate", help="run a manifest of guardrail evaluation cases")
    evaluate.add_argument("manifest", type=Path)
    evaluate.add_argument("--json", type=Path, default=Path("reports/evaluation.json"))
    evaluate.add_argument("--markdown", type=Path, default=Path("reports/evaluation.md"))
    evaluate.add_argument("--predictor", choices=available_predictors(), default="kinematic")

    research_evaluate = sub.add_parser("research-evaluate", help="evaluate a predictor against the independent research benchmark")
    research_evaluate.add_argument("--json", type=Path, default=Path("reports/research_evaluation.json"))
    research_evaluate.add_argument("--markdown", type=Path, default=Path("reports/research_evaluation.md"))
    research_evaluate.add_argument("--predictor", choices=available_predictors(), default="kinematic")
    research_evaluate.add_argument("--benchmark", choices=("seed", "expanded"), default="seed")

    dry_run = sub.add_parser("dry-run", help="run guardrail and emit generic ROS2 cmd_vel commands")
    dry_run.add_argument("plan", type=Path)
    dry_run.add_argument("scenario", type=Path)
    dry_run.add_argument("--predictor", choices=available_predictors(), default="kinematic")
    dry_run.add_argument("--topic", default="/cmd_vel")
    dry_run.add_argument("--output", type=Path, default=Path("reports/cmd_vel_dry_run.json"))

    explain = sub.add_parser("explain", help="explain a guardrail decision path")
    explain.add_argument("plan", type=Path)
    explain.add_argument("scenario", type=Path)
    explain.add_argument("--predictor", choices=available_predictors(), default="kinematic")
    explain.add_argument("--topic", default="/cmd_vel")
    explain.add_argument("--output", type=Path)

    args = parser.parse_args()
    if args.command == "replay":
        actions = parse_plan(read_json(args.plan))
        scene = parse_scene(read_json(args.scenario))
        engine = ReplayEngine(predictor=build_predictor(args.predictor), predictor_name=args.predictor)
        result = engine.run(args.plan.stem, actions, scene)
        write_episode(args.episode, result.episode)
        write_html_report(args.html, result.episode)
        print(f"final_decision={result.final_decision.value}")
        print(f"predictor={args.predictor}")
        print(f"episode={args.episode}")
        print(f"html={args.html}")
    elif args.command == "evaluate":
        engine = ReplayEngine(predictor=build_predictor(args.predictor), predictor_name=args.predictor)
        summary = run_evaluation(args.manifest, engine=engine)
        write_evaluation_json(args.json, summary)
        write_evaluation_markdown(args.markdown, summary)
        print(f"passed={summary.passed}/{summary.total}")
        print(f"pass_rate={summary.pass_rate:.0%}")
        print(f"predictor={args.predictor}")
        print(f"json={args.json}")
        print(f"markdown={args.markdown}")
        if summary.passed != summary.total:
            raise SystemExit(1)
    elif args.command == "dry-run":
        actions = parse_plan(read_json(args.plan))
        scene = parse_scene(read_json(args.scenario))
        engine = ReplayEngine(predictor=build_predictor(args.predictor), predictor_name=args.predictor)
        executor = GuardedCmdVelExecutor(engine=engine, topic=args.topic)
        result = executor.execute(args.plan.stem, actions, scene)
        write_execution_json(args.output, result)
        print(f"final_decision={result.decision.value}")
        print(f"predictor={args.predictor}")
        print(f"topic={result.topic}")
        print(f"published_commands={len(result.published_commands)}")
        print(f"output={args.output}")
    elif args.command == "research-evaluate":
        cases = generate_research_benchmark() if args.benchmark == "seed" else generate_expanded_research_benchmark()
        summary = run_research_evaluation(
            cases=cases,
            predictor=build_predictor(args.predictor),
            predictor_name=args.predictor,
        )
        write_research_evaluation_json(args.json, summary)
        write_research_evaluation_markdown(args.markdown, summary)
        print(f"dangerous_false_negatives={summary.dangerous_false_negatives}/{summary.dangerous_cases}")
        print(f"false_rejects={summary.false_rejects}")
        print(f"unknown_rate={summary.unknown_rate:.0%}")
        print(f"json={args.json}")
        print(f"markdown={args.markdown}")
    elif args.command == "explain":
        actions = parse_plan(read_json(args.plan))
        scene = parse_scene(read_json(args.scenario))
        engine = ReplayEngine(predictor=build_predictor(args.predictor), predictor_name=args.predictor)
        executor = GuardedCmdVelExecutor(engine=engine, topic=args.topic)
        result = executor.execute(args.plan.stem, actions, scene)
        if args.output is not None:
            write_explanation(args.output, result)
        print(format_explanation(result))


if __name__ == "__main__":
    main()
