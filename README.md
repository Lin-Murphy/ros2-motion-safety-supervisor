# ROS2 Motion Safety Supervisor

A portfolio-quality, command-level safety boundary for ROS2 mobile robots.

This project implements an explicit software boundary between candidate robot
actions and execution. It turns actions into typed plans, validates them,
predicts short-horizon risk, handles missing or failed evidence conservatively,
and records replayable episodes for debugging and analysis.

## Portfolio Summary

**Problem.** A velocity command can be syntactically valid while still being
risky because the robot has residual motion, command delay, stale observation,
or insufficient stopping space.

**Contribution.** This repository makes that boundary explicit: candidate
commands pass through static policy, interpretable prediction, conservative
decision logic, and execution evidence before reaching a ROS2 command adapter.
The main technical contribution is an analytical braking envelope evaluated
against separately implemented, held-out stop execution conditions.

**Evidence.** The repository contains replayable red-team cases, deterministic
tests, an independent offline stop benchmark, and a deliberately isolated ROS2
failure-path smoke-test protocol. It does not claim live hardware safety,
calibrated physical stopping performance, or certification.

For the full engineering narrative, see
[`docs/portfolio-case-study.md`](docs/portfolio-case-study.md).

The core pipeline is:

```text
typed action plan
-> static validation
-> pluggable prediction model
-> guardrail decision
-> request / backend acceptance / later observation evidence
-> generic ROS2 /cmd_vel adapter boundary
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
- Records current base-motion evidence and configured execution-delay assumptions
  alongside every replay decision.
- Rejects commands that collide with obstacles or leave the configured bounds.
- Marks decisions as `RISK_UNKNOWN` when required observation evidence is
  missing.
- Converts approved actions to generic `/cmd_vel`-style dry-run commands.
- Separates requested safe commands, adapter-level acceptance, and later
  observed motion in execution evidence.
- Evaluates the guardrail against a deterministic 10-case regression benchmark
  and a 100-case research benchmark with held-out execution and scene shifts.
- Writes replayable JSON episodes, prediction traces, and standalone HTML
  reports.
- Summarizes final decision, risk trigger, clearance, and ROS2 command policy in
  the generated reports.

## Current Scope

V1 uses deterministic, interpretable predictors rather than a learned world
model. The kinematic baseline predicts the candidate command trajectory; the
braking-envelope predictor additionally checks whether observed momentum has
enough space to stop:

```text
pose + scene + candidate action -> predicted trajectory -> risk decision
```

Decisions are deliberately conservative:

- `APPROVED`: valid command and predicted safe in the supplied scene.
- `REJECTED`: malformed, over-budget, collision-predicted, or boundary-violating.
- `RISK_UNKNOWN`: required pose or scene evidence is missing or stale.

Each replay writes a JSON episode and a standalone HTML report with a 2D
trajectory view, obstacle markers, final decision, risk trigger, command
policy, and minimum predicted clearance where available.

## Evidence Snapshot

| Evidence | Current result | Meaning |
| --- | --- | --- |
| Unit suite | 34 deterministic tests | Core policy, prediction, replay, fault, execution-evidence, and ROS2 message-adapter boundaries. |
| Held-out stop benchmark | Baseline: 6 dangerous false negatives; braking envelope: 2, with 1 false reject | A controlled synthetic comparison against a separately implemented delayed-deceleration execution model. |
| Execution evidence | Candidate, decision, requested command, adapter acceptance, and optional later motion are separate fields | A successful adapter call is not presented as proof of robot motion or stopping. |
| ROS2 smoke test | Protocol ready; live run pending | Isolated missing-odometry failure-path check; not a hardware validation. |

The two residual braking-benchmark failures (`combined_escape` and
`velocity_scale_collision`) are retained as visible limitations. They are the
reason this project does not claim a learned world model or real-world safety
performance.

## Reproduce the Core Evidence

```powershell
python -m pip install -e .
python -m unittest discover tests
python -m raspbot_guardrail braking-evaluate --json "$env:TEMP\braking_evaluation.json" --markdown "$env:TEMP\braking_evaluation.md"
python -m raspbot_guardrail replay examples/plans/stop.json examples/scenarios/braking_momentum_risk.json --predictor braking_envelope --html "$env:TEMP\braking_momentum_risk.html"
```

The last command generates the momentum-risk red-team report. All outputs are
offline/replay evidence, not physical robot runs.

Use `%TEMP%` (as above) or `reports/local/` for exploratory output. Only
deliberately selected, reproducible report artifacts should be committed.

## Portfolio Tour

Read these in order for the shortest complete review:

1. [`docs/portfolio-case-study.md`](docs/portfolio-case-study.md): problem,
   architecture, results, limitations, and reproducibility.
2. [`docs/architecture.md`](docs/architecture.md): component responsibilities
   and fail-closed semantics.
3. [`docs/braking-evaluation.md`](docs/braking-evaluation.md): independent
   held-out stop protocol and current result.
4. [`docs/ros2-smoke-test.md`](docs/ros2-smoke-test.md): safe isolated runtime
   integration check for a sourced robot environment.

## Repository Layout

```text
src/raspbot_guardrail/  safety core, predictors, replay, ROS2 adapter
examples/               reproducible safe and red-team inputs
tests/                  deterministic unit and integration-boundary tests
reports/                generated replay and benchmark outputs
docs/                   four focused public documents
scripts/                isolated ROS2 smoke test
```

## Evaluation Benchmark

The evaluation manifest in `examples/evaluation_cases.json` checks V1 behaviour
against safe, static-limit, predictive-risk, under-observed, stale-observation,
and multi-action sequence cases. It reports expected versus actual decisions,
the decision event, static and predictive decisions, risk trigger, reason, and
minimum predicted clearance where available.

The most important red-team case is `collision_risk_predictive_reject`: the
candidate action passes static limits, but the predicted trajectory crosses the
obstacle clearance margin, so the predictive layer rejects it.

The separate braking benchmark evaluates stop commands against an independently
implemented delayed-deceleration execution model. In its current fixed offline
cases, the braking envelope reduces dangerous false negatives from 6 to 2 at
the cost of one false reject. See
[`docs/braking-evaluation.md`](docs/braking-evaluation.md) for the protocol and
limitations.

The sequence case is also important: `multi_action_second_step_predictive_reject`
shows that replay advances the pose after an approved first action and can
reject a later action in the same plan.

## Model Integration

The prediction layer is model-pluggable. The portfolio claim rests on the
interpretable `kinematic` baseline and analytical `braking_envelope` predictor.
The repository also retains experimental `learned_risk` and conservative
`fusion` paths behind the same `predict(scene, action)` interface, but they are
not control dependencies or headline results. Any future learned component
needs representative recorded data and an independent held-out improvement over
the analytical baseline.

## Base-Motion Evidence

Scenes can carry the latest observed base velocity (`linear_x`, `linear_y`,
`angular_z`), its timestamp, and an expected command-delay assumption. The
current kinematic baseline records this evidence without claiming to model a
physical stop. Predictors that depend on it must request it explicitly and
fail closed when it is missing or unstamped.

## Development Direction

The project is deliberately focused on **mobile-base velocity safety**. The
next engineering question is whether an execution-aware braking-envelope
predictor can reduce dangerous approvals under held-out delay and braking
changes, without making the supervisor reject an impractical number of safe
commands.

```text
candidate /cmd_vel + current base motion + scene
    -> static validation
    -> kinematic baseline and braking-envelope prediction
    -> conservative decision and zero-velocity hold when uncertain
    -> replayable evaluation against independent execution conditions
```

Arm, 3D collision, MoveIt, whole-body motion, and a learned world model are not
current project deliverables. Extensions need a separate problem statement and
evidence before they belong in this repository.

## Design FAQ

### How is this different from Nav2 safety features?

Nav2 addresses navigation and runtime velocity filtering. For example, Nav2
Collision Monitor filters `cmd_vel` using sensor-defined safety zones, while
Keepout Filters represent spatial restrictions in the navigation costmap.

This project addresses a different boundary: it is a model-pluggable command
gateway that validates candidate actions, predicts short-horizon consequences,
combines model and fault results, and produces replayable evidence. It can be
combined with Nav2 rather than replacing it:

```text
Nav2 planner/controller
        -> candidate command
        -> Motion Safety Supervisor
        -> Nav2 Collision Monitor or base controller
        -> mobile base
```

The short version is: Nav2 mainly asks whether a velocity command is safe in
the current navigation safety zones; this project asks how to build an
extensible command safety boundary where different predictors, fault handling,
and offline evaluation share one interface.

## Hardware and ROS2 Context

The project was informed by hands-on Raspbot V2 system work. The following
robot and ROS2 paths were verified during that work:

- SSH, host networking, power status, and the host/container split;
- ROS2 Humble running in the robot's Docker workspace;
- `bringup.launch.py` starting the chassis driver;
- `/cmd_vel` using `geometry_msgs/msg/Twist`;
- keyboard teleoperation publishing commands that moved the mecanum base;
- the command path from `/cmd_vel` through the base driver and hardware I2C
  boundary;
- camera capture through `/dev/video0` and an `/image_raw` image topic using
  `sensor_msgs/msg/Image`.

These observations justify the generic ROS2 command and observation boundaries
used by this repository. They do not mean that this repository's safety
supervisor is already a live ROS2 node. The current supervisor integration is
interface-validated; the live node, topic wiring, runtime watchdog deployment,
and hardware acceptance test remain integration work.

## Evidence Labels

Documentation uses explicit evidence labels:

- `hardware_validated`: verified in an earlier mobile-robot learning setup.
- `source_inspected`: confirmed by local source inspection.
- `offline_simulated`: produced by this replay engine and tests.
- `integration_pending`: designed adapter or validation work not yet completed.

## Boundaries

- V1 focuses on command-level guardrails, not full autonomy.
- The predictor is deterministic and interpretable, not a learned world model.
- The optional ROS2 node is interface-complete but not yet hardware-accepted.
- Generated decisions are engineering checks, not formal safety certification.
- Normalized V1 command values are not claimed as calibrated physical
  velocities.

## Story

The project started from a practical robotics systems question: a high-level
action is not necessarily the same as the motion a robot will execute. The
supervisor turns that gap into a reusable engineering boundary with typed
actions, conservative validation, short-horizon prediction, fault handling,
and replayable evidence.
