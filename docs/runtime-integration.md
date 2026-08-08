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

The underlying Raspbot V2 learning work verified the relevant hardware and ROS2
boundaries: ROS2 Humble in the Docker workspace, the chassis bringup, the
`geometry_msgs/msg/Twist` `/cmd_vel` path, the base driver chain, and the
camera `/image_raw` path. Those observations support the generic interfaces in
this repository.

The supervisor itself is currently interface-validated with injected callbacks.
It does not yet claim a live ROS2 safety-supervisor node, deployed runtime
watchdog, or hardware emergency-stop certification. Those require a dedicated
ROS2 node integration and a controlled acceptance test.
