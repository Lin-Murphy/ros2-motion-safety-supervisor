# Raspbot Predictive Guardrail & Replay Lab

A safety, prediction, and replay layer for ROS2 mobile robot command pipelines.

This project builds an explicit software boundary around mobile-base commands.
Starting from a Raspbot V2 control-chain study, it replaces ad-hoc action
execution with typed action plans, validates proposed commands, estimates
short-horizon risk, and records replayable episodes for debugging and analysis.

The public contribution is not vendor demo code. It is an independently authored
command pipeline:

```text
typed action plan
-> static validation
-> short-horizon risk prediction
-> guardrail decision
-> replay log
-> ROS2 /cmd_vel adapter boundary
```

## Why This Exists

Many robot demos connect high-level behaviors directly to execution. That makes
it hard to inspect whether a command is malformed, over-budget, under-observed,
or risky in the current scene.

This project asks a narrower engineering question:

> Can a small, interpretable predictive layer reject unsafe or insufficiently
> observed mobile-robot commands before they enter a ROS2 velocity-command
> pipeline?

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

Run tests:

```bash
python -m unittest discover tests
```

## Evidence Labels

Documentation uses explicit evidence labels:

- `hardware_validated`: verified in earlier Raspbot learning sessions.
- `source_inspected`: confirmed by local source inspection.
- `offline_simulated`: produced by this replay engine and tests.
- `integration_pending`: designed adapter or validation work not yet completed.

## Raspbot Connection

The Raspbot V2 command boundary studied earlier is:

```text
teleop/custom node
-> /cmd_vel
-> /driver_node
-> motor-driver path
-> wheels
```

This project uses `/cmd_vel` as the future ROS2 adapter contract while keeping
the core package runnable as plain Python.

## What This Is Not

- Not a copy of Yahboom vendor source.
- Not a physical safety certification.
- Not a VLA, DQN, or large world-model training project.
- Not a claim that normalized V1 commands are calibrated physical units.

## Story

I used the Raspbot V2 learning workspace to identify the command path, camera
path, data-recording caveats, and dynamic execution boundary. This project
turns those lessons into a reusable robotics-engineering artifact: typed
actions, conservative validation, short-horizon risk prediction, and replayable
episodes.
