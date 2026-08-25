# ROS2 Motion Safety Supervisor

A command-level safety boundary for ROS2 mobile robots. The project checks a
candidate velocity action before execution, predicts short-horizon risk,
handles missing evidence conservatively, and records replayable evidence.

## Problem

A velocity command can be valid but unsafe when the robot has residual motion,
command delay, stale observations, or insufficient stopping space. This project
asks:

> Can a small, interpretable layer reject unsafe or insufficiently observed
> mobile-base commands before they enter a ROS2 velocity pipeline?

## Pipeline

```text
candidate action / Twist
  -> static policy
  -> kinematic or braking-envelope predictor
  -> SafetyDecisionEngine
  -> safe command or zero-velocity hold
  -> backend and execution evidence
  -> replay / evaluation report
```

Predictors never publish commands directly. The core is ROS2-independent; the
optional edge node converts ROS2 `Twist` and `Odometry` messages at the
boundary.

## Results

| Evidence | Result | Boundary |
| --- | --- | --- |
| Deterministic test suite | 34/34 passed | Core, replay, faults, execution evidence, and ROS2 message adapters |
| V1 regression benchmark | 10/10 passed | Offline replay consistency |
| Held-out stop benchmark | Braking envelope reduces dangerous false negatives from 6 to 2, with 1 false reject | 12 controlled synthetic cases and an independent delayed-stop model |
| ROS2 smoke test | Protocol ready; live run pending | Isolated missing-odometry path, not hardware validation |

The braking benchmark still contains two dangerous approvals under combined
delay/deceleration and velocity-scale shifts. These are reported limitations,
not hidden failures.

## Reproduce

```powershell
python -m pip install -e .
python -m unittest discover tests
python -m raspbot_guardrail evaluate examples/evaluation_cases.json --json "$env:TEMP\evaluation.json" --markdown "$env:TEMP\evaluation.md"
python -m raspbot_guardrail braking-evaluate --json "$env:TEMP\braking_evaluation.json" --markdown "$env:TEMP\braking_evaluation.md"
python -m raspbot_guardrail replay examples/plans/stop.json examples/scenarios/braking_momentum_risk.json --predictor braking_envelope --html "$env:TEMP\braking_momentum_risk.html"
```

Use `%TEMP%` or `reports/local/` for exploratory output. Only selected,
reproducible evidence belongs in the committed `reports/` set.

## Public Project Tour

1. [Portfolio case study](docs/portfolio-case-study.md)
2. [Command-gateway architecture](docs/architecture.md)
3. [Independent braking evaluation](docs/braking-evaluation.md)
4. [Isolated ROS2 smoke test](docs/ros2-smoke-test.md)

## Repository Layout

```text
src/raspbot_guardrail/  safety core, predictors, replay, ROS2 adapter
examples/               safe, risky, and under-observed inputs
tests/                  deterministic verification
reports/                selected replay and regression evidence
docs/                   focused public documentation
scripts/                isolated ROS2 smoke test
```

## Scope

The completed vertical slice is mobile-base command safety: typed actions and
observations, static policy, interpretable prediction, independent offline
evaluation, fault-aware decisions, replay, and execution evidence.

Arm control, 3D collision, MoveIt, whole-body motion, direct motor-driver
control, a learned world model, safety-rated operation, and formal
certification are outside this project.

Learned-risk and fusion implementations remain experimental and are not part
of the main safety claim. Any future learned component needs representative
recorded data and an independent held-out improvement over the analytical
baseline.

## Evidence Boundary

`APPROVED` means the supplied checks passed. `REJECTED` means a known policy or
risk condition was detected. `RISK_UNKNOWN` means trustworthy safety could not
be established and requests a zero-velocity hold.

The repository records requested commands separately from backend-accepted
commands and later observed motion. Backend acceptance does not prove ROS
delivery, motor response, physical stopping, or hardware safety.

The optional ROS2 node and smoke-test protocol are interface work. The project
has not claimed live topic validation, calibrated Raspbot braking, emergency
stop capability, or certification.
