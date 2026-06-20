# Prediction Model

V1 uses a deterministic short-horizon kinematic predictor. It is designed to be
auditable rather than learned.

## Inputs

- Current 2D pose: `x`, `y`, `yaw`
- Candidate drive command: `vx`, `vy`, `wz`, `duration_s`
- Scene bounds
- Circular obstacles
- Observation freshness: `observation_age_s`

## Rollout

The predictor rolls the command forward with a fixed time step:

```text
x_next = x + dt * (vx * cos(yaw) - vy * sin(yaw))
y_next = y + dt * (vx * sin(yaw) + vy * cos(yaw))
yaw_next = yaw + dt * wz
```

Each predicted point becomes part of the replay trajectory.

## Risk Checks

For every predicted point, V1 checks:

- whether the robot centre remains inside configured scene bounds, after
  accounting for robot radius
- whether obstacle clearance remains above `clearance_margin`
- whether pose and observation evidence are present and fresh

Obstacle clearance is computed as:

```text
clearance = distance(predicted_point, obstacle_center) - obstacle_radius - robot_radius
```

If clearance falls below the configured margin, the command is rejected with:

```text
predicted obstacle collision or low clearance
```

## Trace Output

Replay episodes include a `model_trace` object that records the predictor name,
time step, robot radius, clearance margin, rollout equations, evidence checks,
input action, horizon, and risk trigger. The HTML replay report displays this
trace so the decision can be inspected without reading the source code.

## Limitations

- Command units are normalized V1 values, not calibrated physical velocities.
- Obstacles are circular approximations.
- The predictor is local and short-horizon; it does not plan around obstacles.
- A `RISK_UNKNOWN` decision means the model refused to score a command because
  required evidence was missing or stale.
