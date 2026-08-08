# Development Roadmap

The long-term direction is a motion safety supervisor for mobile robots and
mobile manipulators. The project grows from the current planar base-command
implementation toward joint-space and whole-body motion, while preserving the
same decision, fault, replay, and evaluation boundaries.

## Stage 1: Generalize Motion Actions

Status: complete

Add typed action forms without changing the existing base-command path:

- `BaseVelocityAction` for planar base motion;
- `ArmJointAction` for joint targets or joint velocities;
- `CompositeMotionAction` for synchronized base and arm actions.

Evidence:

- one common action validation contract;
- tests for malformed, over-limit, and mixed action plans;
- replay events that identify the motion domain.

The implemented schema keeps legacy `DriveAction` and the `"drive"` wire
format. `BaseVelocityAction`, `ArmJointAction`, and
`CompositeMotionAction` share static structural validation. Until an arm or
whole-body predictor is available, arm and composite actions become
`RISK_UNKNOWN`; they are never implicitly approved by the base predictor.

## Stage 2: Add Arm Joint-Space Prediction

Status: planned

Add an arm predictor that estimates short-horizon joint states and checks:

- joint position limits;
- joint velocity and acceleration limits;
- workspace limits;
- stale or missing joint-state evidence.

The existing `SafetyDecisionEngine` remains unchanged. Only the action and
predictor plugins are extended.

Evidence:

- deterministic arm replay cases;
- joint-limit and workspace reports;
- contract tests shared with the base predictor.

## Stage 3: Add Coordinated Base-and-Arm Motion

Status: planned

Evaluate synchronized base velocity and arm joint actions on one timeline. A
known rejection from either subsystem rejects the composite action; missing or
unsupported evidence from either subsystem becomes `RISK_UNKNOWN`.

Evidence:

- synchronized base-plus-arm replay episodes;
- per-subsystem and final decision evidence;
- tests proving that either subsystem can veto the combined action.

## Stage 4: Add a 3D Collision Boundary

Status: planned

Extend the scene representation from planar obstacles to robot and environment
geometry. The first useful 3D boundary should cover link-to-environment,
self-collision, base-to-arm collision, end-effector keep-out volumes, and
short-horizon discrete swept-volume checks. It is a lightweight, testable
predictor boundary, not a replacement for MoveIt's collision engine.

Evidence:

- synthetic link and obstacle scenes;
- collision, clearance, and unknown-state reports;
- explicit MoveIt integration boundary.

## Stage 5: Learned and Uncertainty-Aware Predictors

Status: planned

Extend the current structured `learned_risk` model with execution features for
both base and arm motion, and normalize replay/event evidence across motion
domains. Later versions may add:

- confidence estimates;
- out-of-distribution detection;
- learned dynamics prediction;
- an action-conditioned world-model adapter.

These models remain plugins. They can provide evidence, but they cannot publish
commands or bypass the decision engine. Low confidence, timeout, exception, or
model disagreement remains `RISK_UNKNOWN` with a zero-velocity or hold action.

Evidence:

- train/validation/held-out scenario splits;
- baseline versus learned predictor metrics;
- false-negative, false-reject, unknown-rate, and latency reports;
- no test-case leakage into model fitting.

## Optional Stage 6: Minimal ROS2 Smoke Test

Status: deferred and non-mainline

The existing optional `rclpy` node and `/cmd_vel` adapter are sufficient for
the current design boundary. At most one optional integration test may verify:

```text
candidate Twist publisher -> MotionSafetySupervisorNode -> safe Twist subscriber
```

No full ROS2 package, complex launch graph, MoveIt runtime, or hardware
deployment is part of this roadmap. Nav2 and MoveIt remain external systems
that may use this supervisor's decision and evidence boundaries.

## Final Deliverable Boundary

The intended portfolio result is:

- actual delivery: planar base motion, typed arm and composite actions, arm
  joint-space prediction, a common safety interface, replay, fault handling,
  and reproducible evaluation;
- explicit demonstration: a lightweight 3D collision-predictor extension
  boundary and optional minimal ROS2 smoke test;
- outside the claim: a complete industrial whole-body safety system, safety
  certification, or a replacement for Nav2 or MoveIt.
