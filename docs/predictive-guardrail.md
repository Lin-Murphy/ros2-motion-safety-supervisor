# Predictive Guardrail

V1 uses a lightweight action-conditioned transition model:

```text
state_t + action_t -> predicted_state_t+1 / risk_t+1
```

This is world-model-inspired but deliberately small. It does not generate video,
model wheel slip, learn a policy, or claim physical safety certification.

## State

```text
pose: x, y, yaw
scene: rectangular boundary plus static circular obstacles
observation age
```

## Action

```text
drive(vx, vy, wz, duration_s)
stop(duration_s)
wait(duration_s)
```

`vx`, `vy`, and `wz` are normalized V1 command values. They are not calibrated
physical units.

## Transition

The predictor integrates planar body-frame velocity:

```text
x'   = x + dt * (vx * cos(yaw) - vy * sin(yaw))
y'   = y + dt * (vx * sin(yaw) + vy * cos(yaw))
yaw' = yaw + dt * wz
```

## Evaluation Idea

The key experiment is static-only versus predictive:

- static validator passes a legal command
- predictor rejects it because the trajectory hits an obstacle or boundary
- replay report records the decision and reason
