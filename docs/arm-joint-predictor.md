# Arm Joint-Space Predictor

`ArmJointSpacePredictor` is the first runnable arm-motion predictor in this
repository. It is dependency-light and designed for deterministic replay, not
for direct robot execution.

## Contract

The predictor consumes an `ArmJointAction` and optional arm evidence attached
to `Scene`:

- current joint names, positions, optional measured velocities, and age;
- named position and velocity limits;
- serial planar link lengths;
- a planar end-effector workspace rectangle.

It returns the common `PredictionResult` used by the base path. The 2D base
trajectory remains empty for arm-only motion; `state_trace` records each
predicted joint state and end-effector point.

## Checks

- action, state, and model joint names must match in order;
- joint position limits;
- joint velocity limits;
- linear position-command interpolation or constant velocity-command rollout;
- end-effector workspace boundary;
- missing, stale, malformed, or non-finite joint-state evidence.

Known limit or workspace violations are `REJECTED`. Missing, stale, or
untrustworthy evidence is `RISK_UNKNOWN`, preserving the normal hold policy.

## Replay Inputs

```bash
python -m raspbot_guardrail replay \
  examples/plans/arm_joint_safe.json \
  examples/scenarios/simple_arm.json \
  --predictor arm_joint
```

`arm_joint_limit.json` exercises a predicted position-limit rejection using
the same scenario.

## Assumptions and Limits

- Each joint is revolute and all links lie in one plane.
- Link lengths and limits come from the replay scenario, not a URDF.
- The workspace check applies only to the end-effector point.
- There is no self-collision, link-environment collision, base-arm collision,
  trajectory execution adapter, MoveIt integration, or hardware validation.

The generated trace is `offline_simulated` engineering evidence. It is not a
claim about a calibrated physical arm or safety certification.
