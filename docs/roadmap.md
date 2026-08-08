# Development Roadmap

The long-term direction is a motion safety supervisor for mobile robots and
mobile manipulators. The project grows from the current planar base-command
implementation toward joint-space and whole-body motion, while preserving the
same decision, fault, replay, and evaluation boundaries.

## Stage 1: Generalize Motion Actions

Status: planned

Add typed action forms without changing the existing base-command path:

- `BaseVelocityAction` for planar base motion;
- `ArmJointAction` for joint targets or joint velocities;
- `CompositeMotionAction` for synchronized base and arm actions.

Evidence:

- one common action validation contract;
- tests for malformed, over-limit, and mixed action plans;
- replay events that identify the motion domain.

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

## Stage 3: Add a 3D Collision Boundary

Status: planned

Extend the scene representation from planar obstacles to robot and environment
geometry. The first useful 3D boundary should cover:

- link-to-environment collision;
- self-collision;
- base-to-arm collision;
- end-effector keep-out volumes;
- swept-volume checks over a short trajectory.

The project should define the adapter boundary and reproducible test cases. It
does not need to reimplement all of MoveIt's collision engine.

Evidence:

- 3D collision predictor contract;
- synthetic link and obstacle scenes;
- collision, clearance, and unknown-state reports;
- explicit MoveIt integration boundary.

## Stage 4: Whole-Body Motion

Status: planned

Evaluate coordinated actions such as a base moving while the arm changes
configuration. The supervisor should check the combined state over time rather
than approving the base and arm independently.

Evidence:

- synchronized base-plus-arm replay episodes;
- base-arm self-collision cases;
- tests proving that either subsystem can veto the combined action.

## Stage 5: Learned and Uncertainty-Aware Predictors

Status: planned

Extend the current structured `learned_risk` model with execution features for
both base and arm motion. Later versions may add:

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

## Stage 6: ROS2 and MoveIt Adapters

Status: in progress

Keep the core independent of ROS2 drivers and add adapters at the edge:

- `/cmd_vel` for mobile-base execution;
- `JointTrajectory` or a MoveIt Servo boundary for arm execution;
- Nav2 as a navigation source or downstream velocity-safety layer;
- MoveIt as a planning and collision-checking capability where appropriate.

The first optional `rclpy` node boundary now exists in
`src/raspbot_guardrail/ros2_node.py`. It converts candidate `Twist` messages,
consumes odometry, publishes a separate safe topic, and writes a replay
episode. Live graph validation, obstacle-topic input, runtime watchdog
deployment, and controlled hardware acceptance remain open tasks.

The project remains complementary to Nav2 and MoveIt. It provides a common
decision, fault, audit, and replay boundary rather than replacing either stack.

## Final Deliverable Boundary

The intended portfolio result is:

- actual delivery: planar base motion, arm joint-space motion, a common safety
  interface, replay, fault handling, and reproducible evaluation;
- explicit demonstration: a 3D collision-predictor extension boundary and
  ROS2/MoveIt adapter design;
- outside the claim: a complete industrial whole-body safety system, safety
  certification, or a replacement for Nav2 or MoveIt.
