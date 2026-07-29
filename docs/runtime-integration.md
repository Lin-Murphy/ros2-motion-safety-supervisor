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

## Evidence Boundary

The current runtime adapter is interface-validated with injected callbacks. It
does not claim a live ROS2 graph, Raspbot driver validation, or hardware
emergency-stop certification. Those require a ROS2 environment and a physical
integration test.
