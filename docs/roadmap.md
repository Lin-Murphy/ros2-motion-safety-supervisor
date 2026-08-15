# Development Roadmap

## Project Focus

This repository is a **ROS2 Mobile-Base Motion Safety Supervisor**: a
fault-aware, predictor-pluggable command boundary for velocity commands, with
replayable decision evidence.

```text
candidate /cmd_vel
    -> static policy
    -> risk predictor
    -> SafetyDecisionEngine
    -> approved /cmd_vel or zero-velocity hold
    -> replay and evaluation evidence
```

It is not a replacement for Nav2, a PID/controller, hardware emergency stop,
or a safety certification process.

## Stage 0: Re-focus on the Mobile Base

Status: complete

The arm joint-space, 3D collision, MoveIt, and whole-body extensions are out
of scope. They belong in separate manipulation projects such as the planned
LeRobot and XLeRobot work, where they can be supported by relevant data and
hardware evidence.

The typed predictor contract remains, but it is an extension boundary rather
than a claim that the supervisor currently supports every robot morphology.

## Stage 1: Make Base-Motion Evidence Explicit

Status: complete

Extend the base observation contract to capture the state that affects a safe
stop:

- pose and current linear/angular velocity;
- observation timestamp and staleness;
- command duration and expected command delay;
- obstacle and configured-bound evidence;
- robot footprint and conservative safety margin.

Evidence:

- `BaseMotionState` with planar velocity and an odometry timestamp;
- scene/replay trace fields that distinguish supplied evidence from execution
  assumptions;
- ROS2 odometry conversion into the same core type;
- tests for missing required evidence and replayable high-speed evidence.

## Stage 2: Add a Braking-Envelope Predictor

Status: planned

Add an interpretable predictor that estimates a conservative stopping envelope:

```text
distance before braking + braking distance + margin
```

At a minimum, it accounts for current speed, worst-case command delay,
credible minimum braking deceleration, robot footprint, and scene clearance.
It returns a normal predictor decision and trace; it never publishes
`/cmd_vel` itself.

The core hypothesis is:

> Under held-out execution delay and braking changes, an execution-aware
> braking envelope reduces dangerous false negatives relative to the current
> kinematic baseline, at an explicit false-reject cost.

## Stage 3: Use Independent Held-Out Execution Conditions

Status: planned

The predictor must not be evaluated with the same dynamics and parameters it
uses to make decisions. The evaluation harness will use a separately
implemented execution model and hold out ranges of delay, acceleration,
deceleration, and velocity scale.

Measure:

- dangerous false negatives: commands approved by the supervisor that execute
  into a collision or boundary violation;
- false rejects: commands rejected although the held-out execution is safe;
- `RISK_UNKNOWN` rate and reason;
- decision latency and coverage over the scenario split.

If the braking-envelope predictor does not improve the first metric under this
protocol, it should remain a small baseline experiment rather than become a
large subsystem.

## Stage 4: Experimental Learned Risk (Only with Data)

Status: deferred

The existing learned-risk path is experimental. It should not become a project
headline or a control dependency until there is representative recorded base
motion data, strict train/validation/held-out splits, and an improvement over
the analytical baselines.

Low confidence, distribution shift, timeout, exception, or predictor
disagreement remains `RISK_UNKNOWN` with a zero-velocity hold.

## Stage 5: Minimal ROS2 Smoke Test

Status: integration pending

Keep ROS2 work at the adapter boundary:

- candidate `Twist` input;
- odometry and scene-observation input;
- separate safe output topic;
- watchdog fault to zero-velocity hold;
- a controlled topic-level smoke test before any hardware claim.

The test validates wiring and failure semantics, not physical safety or
certification.

## Deliberate Non-goals

- arm, 3D collision, MoveIt, and whole-body safety;
- a learned world model;
- replacing Nav2 Collision Monitor or navigation safety features;
- direct motor-driver control;
- safety-rated operation or formal certification.
