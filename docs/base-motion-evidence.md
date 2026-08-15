# Base-Motion Evidence Contract

## Goal

Before a predictor can reason about a physical stop, the command gateway must
record the base motion it actually observed rather than only the next candidate
`/cmd_vel` command.

This contract is part of the ROS2 Runtime to Robot Behaviors boundary:

```text
Odometry + scene observation + candidate Twist
    -> Scene(BaseMotionState, observation freshness, delay assumption)
    -> predictor evidence
    -> SafetyDecisionEngine
```

## Input Schema

`Scene` can include:

```json
{
  "observation_age_s": 0.1,
  "max_observation_age_s": 1.0,
  "base_motion": {
    "linear_x": 0.25,
    "linear_y": 0.0,
    "angular_z": 0.1,
    "timestamp_s": 42.0
  },
  "expected_command_delay_s": 0.1
}
```

- `base_motion` is observed state. In the ROS2 adapter it comes from
  `nav_msgs/msg/Odometry.twist.twist` and its header timestamp.
- `observation_age_s` is the gateway's current freshness calculation.
- `expected_command_delay_s` is a configured execution assumption, not a
  sensor observation.
- footprint radius and clearance margin remain predictor configuration and are
  also written to the model trace.

## Current Behaviour

The kinematic baseline records this evidence in `model_trace.scene_evidence`
and `model_trace.execution_assumptions`; replay metadata preserves the same
input for inspection.

The baseline does not yet claim that the observed speed changes its trajectory.
That requires the braking-envelope predictor in the next stage. A predictor can
set `require_base_motion=True`; if the state or its timestamp is unavailable,
the result is `RISK_UNKNOWN` with a zero-velocity hold.

## Failure Semantics

| Condition | Result |
| --- | --- |
| Missing pose/bounds or stale scene observation | `RISK_UNKNOWN` |
| A predictor requires base motion but it is missing/unstamped | `RISK_UNKNOWN` |
| Base motion is present | Evidence is recorded; the active predictor decides |

This is offline/interface evidence only. It does not demonstrate that the
Raspbot publishes odometry today, that the values are calibrated physical units,
or that the software is a safety-rated stopping system.

## Verification

The unit suite verifies that required base-motion evidence fails closed, that a
high-speed sample survives into replay output, and that ROS2-message conversion
does not leak ROS2 types into the core model.
