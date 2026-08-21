# Runtime Integration Boundary

The runtime integration is intentionally an adapter around the dependency-free
safety core.

```text
ROS2 subscriber / command source
        -> CandidateAction
        -> MotionSafetySupervisor
        -> SafetyDecisionEngine
        -> Ros2CmdVelBackend
        -> geometry_msgs/msg/Twist publisher
```

The `Ros2CmdVelBackend` accepts an injected publish callback. A ROS2 node can
convert the generic `TwistCommand` to `geometry_msgs/msg/Twist` at the edge,
while replay and dry-run tests use the same core without importing `rclpy`.

## Output Ownership

The intended deployment topology has one safety-controlled output:

```text
/cmd_vel_candidate -> motion safety supervisor -> /cmd_vel_safe -> driver
```

The supervisor is the only component that should publish the final safe command
topic. A backend failure, stale command, stale odometry, or predictor fault
requests a zero-velocity hold and records a structured fault.

Episode metadata distinguishes a requested command from adapter-level command
acceptance. Neither proves motor response or a physical stop: a later base
motion observation must be supplied separately, otherwise that field remains
`not_collected`.

## Evidence Boundary

The underlying Raspbot V2 learning work verified the relevant hardware and ROS2
boundaries: ROS2 Humble in the Docker workspace, the chassis bringup, the
`geometry_msgs/msg/Twist` `/cmd_vel` path, the base driver chain, and the
camera `/image_raw` path. Those observations support the generic interfaces in
this repository.

The supervisor itself is interface-validated with injected callbacks, and the
repository now includes an optional `rclpy` node boundary. It does not yet
claim a live ROS2 graph validation, deployed runtime watchdog, or hardware
emergency-stop certification. Those require a controlled acceptance test.

## Minimal Live Node

The optional `raspbot_guardrail.ros2_node` module provides the first live node
boundary. It subscribes to a candidate `geometry_msgs/msg/Twist`, consumes
`nav_msgs/msg/Odometry`, evaluates the command through the existing safety core,
publishes to a separate safe topic, and writes the resulting episode.

It is intentionally optional: the core package remains runnable without
`rclpy`. Run it inside a sourced ROS2 environment after installing this package:

```bash
python -m raspbot_guardrail.ros2_node \
  --ros-args \
  -p candidate_topic:=/cmd_vel_candidate \
  -p safe_topic:=/cmd_vel_safe \
  -p odom_topic:=/odom
```

The first node boundary does not yet consume obstacle detections, and the
default `RISK_UNKNOWN` behaviour applies until a valid odometry pose is
available. A controlled hardware test is still required before treating it as
a deployment-ready safety component.

For the first live adapter check, use the isolated zero-command protocol in
[`ros2-smoke-test.md`](ros2-smoke-test.md). It confirms missing-observation
failure semantics without publishing to the production driver topic.
