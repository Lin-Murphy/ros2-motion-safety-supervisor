# Baseline Limitations

The `kinematic` predictor is intentionally simple. Its value is that its
assumptions, equations, and failure triggers can be inspected directly.

## Main Assumptions

- The current pose is sufficiently accurate.
- The command begins executing immediately.
- The commanded motion is close to the normalized planar model.
- The scene geometry is static and correctly observed.
- Circular obstacle approximations are adequate for the check.
- The short prediction horizon is sufficient for the decision.

## Expected Failure Modes

The baseline can be optimistic when the robot experiences command delay,
acceleration limits, wheel slip, velocity tracking error, pose error, stale
observations, or unobserved dynamic obstacles. These effects are not bugs in
the rollout equations; they are mismatches between the model and execution.

The independent reference execution model and parameterized benchmark will
make these mismatches explicit. The purpose is to measure when the baseline
approves an action whose reference outcome is dangerous.

## Interpretation

A baseline failure does not automatically justify a learned predictor. The
learned model must reduce dangerous false negatives on held-out scenarios
without creating an unacceptable increase in false rejects, uncertainty, or
runtime latency.
