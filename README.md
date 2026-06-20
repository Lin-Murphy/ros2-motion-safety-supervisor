# Raspbot Action Guardrail & Replay Lab

A safety, prediction, and replay layer for ROS2 mobile robot command pipelines.

This project implements an explicit software boundary around mobile-base
commands. Starting from a Raspbot V2 control-chain study, it turns candidate
actions into typed plans, validates them, predicts short-horizon risk, and
records replayable episodes for debugging and analysis.

The core pipeline is:

```text
typed action plan
-> static validation
-> short-horizon risk prediction
-> guardrail decision
-> replay log
-> ROS2 /cmd_vel adapter boundary
```

## Engineering Question

Many robot demos connect high-level behaviors directly to execution. That makes
it hard to inspect whether a command is malformed, over-budget, under-observed,
or risky in the current scene.

This project asks a narrower engineering question:

> Can a small, interpretable predictive layer reject unsafe or insufficiently
> observed mobile-robot commands before they enter a ROS2 velocity-command
> pipeline?

## What It Does

- Parses candidate robot actions into a typed action schema.
- Applies static command checks such as duration and speed limits.
- Predicts a short-horizon 2D trajectory from pose, scene, and candidate action.
- Rejects commands that collide with obstacles or leave the configured bounds.
- Marks decisions as `RISK_UNKNOWN` when required observation evidence is
  missing.
- Writes replayable JSON episodes, prediction traces, and standalone HTML
  reports.

## Current Scope

V1 uses a deterministic, interpretable predictor rather than a learned world
model. It predicts command-level consequences over a short horizon:

```text
pose + scene + candidate action -> predicted trajectory -> risk decision
```

Decisions are deliberately conservative:

- `APPROVED`: valid command and predicted safe in the supplied scene.
- `REJECTED`: malformed, over-budget, collision-predicted, or boundary-violating.
- `RISK_UNKNOWN`: required pose or scene evidence is missing or stale.

Each replay writes a JSON episode and a standalone HTML report with a 2D
trajectory view, obstacle markers, final decision, and minimum predicted
clearance where available.

## Example Outputs

- `reports/clear_drive.html`: an approved action in a simple room scenario.
- `reports/collision_risk.html`: a rejected action with predicted obstacle risk.
- `reports/unknown_scene.html`: a conservative `RISK_UNKNOWN` decision when
  required scene evidence is missing.
- `reports/evaluation.md`: a compact pass/fail report for the V1 guardrail
  case set.

## Quick Start

Install the local package:

```bash
python -m pip install -e .
```

Run the safe example:

```bash
python -m raspbot_guardrail replay examples/plans/clear_drive.json examples/scenarios/simple_room.json --html reports/clear_drive.html
```

Run the red-team example:

```bash
python -m raspbot_guardrail replay examples/plans/collision_risk.json examples/scenarios/simple_room.json --html reports/collision_risk.html
```

Run the evaluation suite:

```bash
python -m raspbot_guardrail evaluate examples/evaluation_cases.json --json reports/evaluation.json --markdown reports/evaluation.md
```

Run tests:

```bash
python -m unittest discover tests
```

## Repository Map

- `src/raspbot_guardrail/actions.py`: typed action schema and plan parsing.
- `src/raspbot_guardrail/policy.py`: static command validation.
- `src/raspbot_guardrail/predictor.py`: deterministic short-horizon trajectory
  and risk prediction.
- `src/raspbot_guardrail/evaluation.py`: manifest-driven evaluation harness.
- `src/raspbot_guardrail/replay.py`: replay engine and episode generation.
- `src/raspbot_guardrail/report.py`: standalone HTML report generation.
- `src/raspbot_guardrail/backends/ros2_cmd_vel.py`: ROS2 `/cmd_vel` adapter
  boundary.
- `examples/`: replay inputs for safe, risky, and under-observed cases.
- `docs/`: control-chain evidence, command policy, data model, and validation
  notes.
- `docs/prediction-model.md`: the V1 rollout equations, clearance check, and
  trace output.

## Validation Harness

The evaluation manifest in `examples/evaluation_cases.json` checks V1 behaviour
against safe, risky, and under-observed command cases. It reports expected
versus actual decisions and records both static and predictive decisions.

The most important red-team case is `collision_risk_predictive_reject`: the
candidate action passes static limits, but the predicted trajectory crosses the
obstacle clearance margin, so the predictive layer rejects it.

## Evidence Labels

Documentation uses explicit evidence labels:

- `hardware_validated`: verified in earlier Raspbot learning sessions.
- `source_inspected`: confirmed by local source inspection.
- `offline_simulated`: produced by this replay engine and tests.
- `integration_pending`: designed adapter or validation work not yet completed.

## Raspbot Connection

The Raspbot V2 command boundary behind this project is:

```text
teleop/custom node
-> /cmd_vel
-> /driver_node
-> motor-driver path
-> wheels
```

This project uses `/cmd_vel` as the future ROS2 adapter contract while keeping
the core package runnable as plain Python.

## Boundaries

- V1 focuses on command-level guardrails, not full autonomy.
- The predictor is deterministic and interpretable, not a learned world model.
- Generated decisions are engineering checks, not formal safety certification.
- Normalized V1 command values are not claimed as calibrated physical
  velocities.

## Story

I used the Raspbot V2 learning workspace to identify the command path, camera
path, data-recording caveats, and dynamic execution boundary. This project
turns those lessons into a reusable robotics-engineering artifact: typed
actions, conservative validation, short-horizon risk prediction, and replayable
episodes.
