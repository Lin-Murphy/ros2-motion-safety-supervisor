# ROS2 Cmd Vel Contract

This project keeps the execution boundary generic for ROS2 mobile bases. Raspbot
is the motivating case study, but the guardrail output is a `/cmd_vel`-style
velocity intent rather than a board-specific motor command.

## Signal Path

```text
candidate action plan
-> typed action parser
-> static policy
-> predictor
-> guardrail decision
-> guarded cmd_vel executor
-> TwistCommand
-> ROS2 /cmd_vel publisher
```

V1 stops at dry-run `TwistCommand` output. It intentionally does not import
`rclpy`, publish live ROS2 topics, or depend on a specific robot driver.

## Generic Command

`TwistCommand` mirrors the fields needed by a ROS2 mobile-base velocity command:

```text
topic
linear_x
linear_y
angular_z
duration_s
```

The default topic is `/cmd_vel`, but the CLI accepts a custom topic for robots
that remap the velocity command input.

## Guarded Execution Rule

- If the final guardrail decision is `APPROVED`, the executor converts approved
  actions to `TwistCommand` values and ensures the sequence ends with a
  zero-velocity stop.
- If the final decision is `REJECTED` or `RISK_UNKNOWN`, the executor emits only
  a zero-velocity hold command.

This keeps the safety policy independent of the robot platform.

## Raspbot Mapping

For Raspbot, the later live path would be:

```text
APPROVED DriveAction
-> TwistCommand
-> geometry_msgs/msg/Twist
-> /cmd_vel
-> driver node
-> motor-driver path
-> wheels
```

The same guardrail can be reused for another ROS2 differential or mecanum base
as long as that robot accepts a compatible velocity-command topic.

## Dry-Run CLI

```bash
python -m raspbot_guardrail dry-run examples/plans/clear_drive.json examples/scenarios/simple_room.json --output reports/clear_drive_cmd_vel_dry_run.json
```

The output records the guardrail decision, selected predictor, topic, generated
commands, and a compact guardrail event summary.
