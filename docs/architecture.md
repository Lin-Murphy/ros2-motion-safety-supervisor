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
        -> DecisionArbiter
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
| `DecisionArbiter` | Combine policy, predictor, and fault outcomes. | Depend on a specific ROS2 driver. |
| `CommandBackend` | Convert an approved action into an execution-side command. | Re-run safety policy. |
| `EventRecorder` | Persist structured decisions, faults, and execution outcomes. | Change the decision after recording it. |

The current `ReplayEngine`, `MotionSafetySupervisor`, predictor registry,
dry-run executor, runtime watchdog, event recorder, and ROS2 `/cmd_vel` boundary
implement these responsibilities. The research benchmark remains an adapter
around the same predictor contract.

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
        -> independent reference execution
        -> learned action-conditioned predictor
        -> common benchmark and metrics
        -> conservative arbiter policy
```

The learned predictor can improve evidence, but it cannot bypass static policy,
the arbiter, event recording, or the backend hold policy. A future world-model
predictor must use the same predictor contract and evaluation boundary.

## Non-goals

This architecture does not claim to replace Nav2 Keepout Filter, Nav2 Collision
Monitor, a safety-rated controller, or formal safety certification. It provides
a reusable command-gateway boundary with explicit failure handling, replay, and
model evaluation.
