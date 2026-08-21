# Command Gateway Architecture

## Primary Positioning

This project is a modular and fault-aware command gateway for ROS2 mobile
robots. Predictive models are replaceable components inside the gateway; they
are not allowed to publish commands directly.

```text
ROS2 command source / replay input
        -> ActionSource
        -> action validation
        -> PolicyEvaluator
        -> Predictor plugin(s)
        -> SafetyDecisionEngine
        -> EventRecorder
        -> CommandBackend
        -> approved /cmd_vel or zero-velocity hold
```

The core domain logic remains independent of `rclpy`. ROS2 integration is an
adapter concern.

## Module Responsibilities

| Component | Responsibility | Must not do |
| --- | --- | --- |
| `ActionSource` | Supply typed candidate actions and observation context. | Publish motor commands. |
| `PolicyEvaluator` | Check deterministic command and plan constraints. | Perform model inference. |
| `Predictor` | Estimate short-horizon consequences and report assumptions. | Publish `/cmd_vel` or silently approve on failure. |
| `SafetyDecisionEngine` | Combine policy, predictor, and fault outcomes. | Depend on a specific ROS2 driver. |
| `CommandBackend` | Convert an approved action into an execution-side command. | Re-run safety policy. |
| `EventRecorder` | Persist structured decisions, faults, and execution outcomes. | Change the decision after recording it. |

The current `ReplayEngine`, `MotionSafetySupervisor`, predictor registry,
dry-run executor, runtime watchdog, event recorder, and ROS2 `/cmd_vel` boundary
implement these responsibilities. The research benchmark remains an adapter
around the same predictor contract.

## Evidence Contract

The safety decision and the execution outcome are deliberately separate:

```text
observed pose + base motion + scene + candidate action
        -> policy and predictor decision
        -> requested safe command or zero hold
        -> backend-accepted command
        -> optional later observed base motion
```

`Scene` owns pose, bounds, obstacles, observation freshness, and optional base
velocity with its timestamp. The braking envelope additionally requires a
command-delay assumption. Missing required evidence becomes `RISK_UNKNOWN`.

Execution records candidate actions, decision, requested commands,
adapter-accepted commands, backend faults, and any later caller-supplied base
motion. Adapter acceptance proves only that the software callback succeeded;
it does not prove ROS delivery, motor response, or a physical stop.

## Decision and Failure Semantics

The gateway has three externally visible decisions:

- `APPROVED`: all required checks passed.
- `REJECTED`: a known policy or risk condition was detected.
- `RISK_UNKNOWN`: the gateway cannot establish a trustworthy safe decision.

The default handling of `RISK_UNKNOWN` is a zero-velocity hold. The following
conditions must be observable and must not silently become approval:

- malformed action;
- missing or stale observation;
- predictor timeout;
- predictor exception;
- unavailable command backend;
- conflicting predictor decisions;
- event recording failure.

The exact response may distinguish `REJECTED` from `RISK_UNKNOWN`, but every
failure path must record a stable reason, component, and fallback action.

## Research Extension Boundary

The research path is deliberately downstream of the architecture:

```text
kinematic baseline
        -> braking-envelope predictor
        -> independent held-out execution conditions
        -> common benchmark and false-negative/false-reject metrics
        -> conservative decision policy
```

The braking-envelope predictor is a conservative analytical check, not a
second simulator: it must be evaluated against independently implemented
execution conditions whose delay and braking parameters are not its own
assumptions. Experimental learned predictors may improve evidence later, but
cannot bypass static policy, the decision engine, event recording, or the
backend hold policy. A world-model predictor is outside the current roadmap.

## Non-goals

This architecture does not claim to replace Nav2 Keepout Filter, Nav2 Collision
Monitor, a safety-rated controller, or formal safety certification. It provides
a reusable command-gateway boundary with explicit failure handling, replay, and
model evaluation.
