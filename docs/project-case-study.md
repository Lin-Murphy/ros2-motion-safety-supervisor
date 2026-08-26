# ROS2 Motion Safety Supervisor: Project Case Study

## The Problem

Robot software often sends a high-level intent directly into a velocity-command
pipeline. That hides several different questions behind one publish call:

```text
Is the command valid?
Is it safe for the observed scene?
Does current momentum leave enough space to stop?
Did the software adapter accept the command?
Was later robot motion actually observed?
```

This project makes those questions explicit for a ROS2 mobile-base command
pipeline. It is a designed command-safety and evidence architecture, not a
repackaged vendor demo.

## System Boundary

```text
candidate action or Twist
  -> static command policy
  -> kinematic baseline + optional braking envelope
  -> SafetyDecisionEngine
  -> requested safe command or zero-velocity hold
  -> backend acceptance record
  -> optional later base-motion observation
  -> replayable episode and HTML explanation
```

The core is independent of ROS2. The optional edge node converts ROS2
`geometry_msgs/msg/Twist` and `nav_msgs/msg/Odometry` at the boundary, allowing
the policy and predictors to be replayed and tested without a running robot.

## Key Engineering Decisions

### Fail closed on missing evidence

`RISK_UNKNOWN` is a first-class decision. Missing pose, stale observation,
missing base-motion evidence for a braking check, predictor faults, and backend
faults do not silently become approval; the execution path requests a
zero-velocity hold.

### Keep prediction separate from execution truth

The analytical braking envelope estimates reaction distance plus braking
distance from observed velocity, configured delay, and a conservative minimum
deceleration. A separate `ReferenceExecutionModel` applies discrete delayed
deceleration to evaluate the predictor. The predictor is therefore not scored
against its own rollout.

### Keep software acceptance separate from physical observation

Every guarded execution records candidate action, decision, requested command,
backend-accepted command, backend status, and an optional later motion sample.
An accepted backend call establishes only adapter success, not motor response
or a physical stop.

## Controlled Result

The 12-case stop benchmark uses fixed nominal and held-out delay, deceleration,
combined-shift, and velocity-scale conditions. On its six dangerous cases:

| Predictor | Dangerous false negatives | False rejects |
| --- | ---: | ---: |
| Kinematic baseline | 6 | 0 |
| Analytical braking envelope | 2 | 1 |

The braking envelope catches several momentum-related risks that the baseline
approves. It still misses `combined_escape` and `velocity_scale_collision`, and
it rejects one safe nominal case. Those outcomes are retained as results, not
removed from the benchmark. Full protocol:
[`braking-evaluation.md`](braking-evaluation.md).

## Reproduce the Evidence

From a local checkout:

```powershell
python -m pip install -e .
python -m unittest discover tests
python -m raspbot_guardrail braking-evaluate --json "$env:TEMP\braking_evaluation.json" --markdown "$env:TEMP\braking_evaluation.md"
python -m raspbot_guardrail replay examples/plans/stop.json examples/scenarios/braking_momentum_risk.json --predictor braking_envelope --html "$env:TEMP\braking_momentum_risk.html"
```

The test suite covers the dependency-free core and message-like ROS2 conversion
helpers. The last command produces a replayable red-team report; it is not a
physical robot run.

## ROS2 Integration Status

The optional node owns a separate safe-output topic. The first live acceptance
step is an isolated smoke test with a deliberately absent odometry source:
missing odometry must produce `RISK_UNKNOWN` and a zero Twist on a topic that
is not connected to a motor driver. See
[`ros2-smoke-test.md`](ros2-smoke-test.md).

The current repository has not run that test on a live graph. It therefore does
not claim deployed watchdog behaviour, live topic validation, calibrated
Raspbot braking, emergency-stop capability, or safety certification.

## Scope and Next Extension

The completed vertical slice is mobile-base command safety:

- typed action and observation contracts;
- deterministic policy and predictor boundary;
- independent offline execution evaluation;
- replayable decision and execution evidence;
- optional ROS2 adapter plus a safe integration protocol.

LeRobot or XLeRobot work can later treat this as the explicit
`Action -> Safety -> Execution evidence` boundary. That is a separate project
integration step, not a claim that the current repository controls an arm,
uses 3D collision checking, or contains a world model.
