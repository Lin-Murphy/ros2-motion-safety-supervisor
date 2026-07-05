# Reference Execution Model

The reference execution model is an independent offline approximation of what
may happen after a command is issued. It is used to produce benchmark ground
truth; it is not another guardrail predictor and it does not approve commands.

## Model Effects

The first version includes:

- command delay before the requested motion becomes active;
- linear acceleration limits for `vx` and `vy`;
- angular acceleration limits for `wz`;
- velocity execution scale error;
- the same scene geometry checks for collision and boundary outcomes.

The model produces an actual execution trace and one of:

- `SAFE`;
- `COLLISION`;
- `BOUNDARY_VIOLATION`;
- `UNKNOWN` when the initial pose or bounds are unavailable.

## Why It Is Separate

The kinematic predictor assumes an ideal command-to-motion relationship. If the
reference result reused that same rollout, the evaluation would only verify the
predictor against itself. Keeping execution and prediction separate allows the
benchmark to expose cases where an ideal predictor approves an action but an
execution model with delay or motion limits reaches a dangerous state.

This is still an offline approximation, not physical validation. Its purpose is
to make the baseline's assumptions testable before introducing a learned model.
