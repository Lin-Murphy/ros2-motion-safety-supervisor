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

Status: complete

Add an interpretable predictor that estimates a conservative stopping envelope:

```text
distance before braking + braking distance + margin
```

The implemented predictor accounts for current speed, configured command delay,
minimum credible deceleration, robot footprint, and scene clearance. It
combines the existing candidate-trajectory check with a closed-form stopping
envelope, returns a normal predictor decision and trace, and never publishes
`/cmd_vel` itself.

Evidence delivered:

- predictor registry integration and CLI selection via `braking_envelope`;
- fail-closed tests for missing base-motion evidence;
- a safe stopping-space test and a baseline-versus-braking red-team test;
- a replayable `stop` plan and momentum-risk scenario.

The core hypothesis is:

> Under held-out execution delay and braking changes, an execution-aware
> braking envelope reduces dangerous false negatives relative to the current
> kinematic baseline, at an explicit false-reject cost.

## Stage 3: Use Independent Held-Out Execution Conditions

Status: complete

The predictor is not evaluated with the same dynamics and parameters it uses
to make decisions. The evaluation harness uses a separately implemented,
discrete delayed-deceleration execution model and holds out delay,
deceleration, and velocity-scale settings.

Measure:

- dangerous false negatives: commands approved by the supervisor that execute
  into a collision or boundary violation;
- false rejects: commands rejected although the held-out execution is safe;
- `RISK_UNKNOWN` rate and reason;
- decision latency and coverage over the scenario split.

Current controlled result: among 6 dangerous stop cases, the kinematic baseline
has 6 dangerous false negatives; the braking envelope has 2, with 1 false
reject. The two remaining dangerous approvals are documented limitations under
combined and velocity-scale shifts, not hidden failures. See
[`braking-evaluation.md`](braking-evaluation.md).

## Stage 4: Record Decision, Dispatch, and Observation Separately

Status: complete

The execution result must preserve the difference between a command the
supervisor requested, a command the output adapter accepted, and motion that
was actually observed after dispatch. Adapter acceptance is not treated as
proof of motor response or a physical stop.

Evidence delivered:

- `execution_evidence` in each execution episode records candidate actions,
  supervisor decision, requested commands, adapter-accepted commands, backend
  status, and optional caller-supplied post-dispatch base motion;
- unavailable or failing backends retain the requested zero-velocity hold while
  showing that it was not accepted by the adapter;
- explanation and HTML reports label this boundary explicitly;
- unit tests cover approval, rejection, backend fault, and a supplied later
  motion observation.

## Stage 5: Experimental Learned Risk (Only with Data)

Status: deferred

The existing learned-risk path is experimental. It should not become a project
headline or a control dependency until there is representative recorded base
motion data, strict train/validation/held-out splits, and an improvement over
the analytical baselines.

Low confidence, distribution shift, timeout, exception, or predictor
disagreement remains `RISK_UNKNOWN` with a zero-velocity hold.

## Stage 6: Minimal ROS2 Smoke Test

Status: protocol ready; live run pending

Keep ROS2 work at the adapter boundary:

- candidate `Twist` input;
- odometry and scene-observation input;
- separate safe output topic;
- watchdog fault to zero-velocity hold;
- a controlled topic-level smoke test before any hardware claim.

An isolated zero-command protocol and script are ready in
[`ros2-smoke-test.md`](ros2-smoke-test.md). The local Windows environment has
no sourced ROS2 runtime and the known robot address is currently unreachable,
so no live graph result is claimed yet.

The test validates wiring and failure semantics, not physical safety or
certification.

## Deliberate Non-goals

- arm, 3D collision, MoveIt, and whole-body safety;
- a learned world model;
- replacing Nav2 Collision Monitor or navigation safety features;
- direct motor-driver control;
- safety-rated operation or formal certification.
