# ROS2 Motion Safety Supervisor

A small safety gate for ROS2 mobile robots. Before a velocity command reaches
the robot, it can approve the command, reject it, or hold the robot stopped
when the current state is unsafe or unknown. Every decision can be replayed
and inspected.

## Problem

A velocity command can look valid but still be unsafe when the robot is already
moving, observations are old, commands are delayed, or there is not enough
space to stop. This project adds a small, understandable safety layer before
the command enters the ROS2 pipeline.

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

The prediction modules only provide risk information; they never publish
commands directly. The safety core does not depend on ROS2. An optional edge
node converts ROS2 `Twist` and `Odometry` messages at the boundary.

## Results

| Evidence | Result | Boundary |
| --- | --- | --- |
| Deterministic test suite | 34/34 passed | Core, replay, faults, execution evidence, and ROS2 message adapters |
| V1 regression benchmark | 10/10 passed | Offline replay consistency |
| Independent stop benchmark | The braking model reduces missed dangerous stops from 6 to 2, with 1 unnecessary rejection | 12 controlled synthetic cases and a separate delayed-stop model |
| ROS2 smoke test | Protocol ready; live run pending | Isolated missing-odometry path, not hardware validation |

The benchmark still contains two dangerous approvals when delay, deceleration,
and velocity scale change together. These are reported limitations, not hidden
failures.

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

## Documentation

1. [Project case study](docs/project-case-study.md)
2. [System architecture](docs/architecture.md)
3. [Stop-risk evaluation](docs/braking-evaluation.md)
4. [ROS2 interface smoke test](docs/ros2-smoke-test.md)

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

The completed vertical slice focuses on mobile-base command safety: typed
commands and observations, simple policy checks, interpretable prediction,
offline evaluation, fault-aware decisions, replay, and execution evidence.

Arm control, 3D collision, MoveIt, whole-body motion, direct motor-driver
control, a learned world model, safety-rated operation, and formal
certification are outside this project.

The learned-risk and fusion modules are experimental and are not part of the
main safety claim. A future learned component would need representative data
and a separate test showing improvement over the analytical baseline.

## Evidence Boundary

`APPROVED` means the supplied checks passed. `REJECTED` means a known policy or
risk problem was detected. `RISK_UNKNOWN` means there is not enough trustworthy
evidence, so the system requests a zero-velocity hold.

The repository records requested commands separately from commands accepted by
the backend and from later observed motion. Backend acceptance alone does not
prove ROS delivery, motor response, physical stopping, or hardware safety.

The optional ROS2 node and smoke-test protocol are interface work. The project
has not claimed live topic validation, calibrated Raspbot braking, emergency
stop capability, or certification.
