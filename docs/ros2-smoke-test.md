# Isolated ROS2 Zero-Command Smoke Test

## Purpose

This is a **ROS2 Runtime** adapter check for this data flow:

```text
isolated candidate Twist
  -> motion safety supervisor
  -> isolated safe Twist
  -> recorded episode
```

It deliberately configures an odometry topic with no publisher. The expected
result is `RISK_UNKNOWN` and one zero-velocity hold on an isolated safe topic.
It does not test robot movement, braking, driver response, or emergency stop.

## Runtime Boundary

The optional node converts `geometry_msgs/msg/Twist` into a typed candidate
action, consumes `nav_msgs/msg/Odometry` into pose and base-motion evidence,
and publishes only to a separate safe topic:

```text
/cmd_vel_candidate -> motion_safety_supervisor -> /cmd_vel_safe -> driver
                         ^
                       /odom
```

The smoke test replaces all three names with isolated topic names and leaves
its configured odometry topic without a publisher. This proves the
missing-observation failure path without connecting a candidate command to a
motor driver.

## Preconditions

- Run on the Raspberry Pi in a sourced ROS2 environment where `rclpy`,
  `geometry_msgs`, and `nav_msgs` import successfully.
- Start from this repository checkout; the script uses its `src` package.
- Keep the robot's production driver disconnected from both smoke-test topics.
- Never remap either smoke-test topic to `/cmd_vel`.
- Keep the robot stationary. This test should not create a motor command.

The script uses these fixed, isolated names:

```text
/cmd_vel_guardrail_smoke_candidate
/cmd_vel_guardrail_smoke_safe
/odom_guardrail_smoke_missing
```

Before it publishes anything, it prints verbose topic information and requires
you to type `ISOLATED`. Stop if any motor-driver node subscribes to either
`cmd_vel_guardrail_smoke` topic.

## Run

From the repository checkout on the Raspberry Pi:

```bash
bash scripts/ros2_zero_command_smoke.sh
```

The script starts the optional node, sends one nonzero candidate command only
to the isolated candidate topic, and waits for a message on the isolated safe
topic. Because the configured odometry topic has no publisher, the supervisor
must reject the command as `RISK_UNKNOWN` and send a zero Twist instead.

## Pass Conditions

The script exits successfully only when all of these are true:

1. The episode has `execution_evidence.supervisor_decision == RISK_UNKNOWN`.
2. It records exactly one backend-accepted command, with all three velocity
   components equal to zero.
3. The captured isolated ROS2 `Twist` also has all velocity components equal
   to zero.

It writes the episode and captured message under a timestamped
`reports/ros2_zero_command_smoke_*` directory. Keep these files as interface
evidence, but do not describe them as physical-safety validation.

## What This Leaves Unverified

- the live production `/cmd_vel` topology;
- an actual odometry source and its timestamps;
- an approved-command path on the robot;
- motor and braking response;
- watchdog deployment, hardware emergency stop, and certification.

Those require a later controlled acceptance test with the real ROS2 graph and
a safe physical test area.
