# Braking-Envelope Predictor

## Goal

The kinematic baseline asks whether the *candidate command trajectory* is safe.
It does not account for a base that is already moving when the command arrives.

`BrakingEnvelopePredictor` adds one conservative question:

> Given the observed current velocity, command delay, and minimum credible
> deceleration, is there enough clear space to stop?

## Inputs and Output

Inputs come from `Scene` and the candidate action:

```text
pose + bounds + obstacles
+ BaseMotionState(vx, vy, wz, timestamp)
+ expected command delay
+ robot footprint + clearance margin
-> normal PredictionResult
```

It returns `APPROVED`, `REJECTED`, or `RISK_UNKNOWN` through the common
predictor interface. It does not publish `/cmd_vel` and cannot bypass static
policy or the decision engine.

## Calculation

The implementation uses a closed-form upper bound, rather than a time-stepped
simulation:

```text
reaction_distance = current_linear_speed * expected_command_delay_s
braking_distance  = current_linear_speed^2 / (2 * minimum_deceleration)
stopping_distance = reaction_distance + braking_distance
```

The stopping segment is projected along the observed base-velocity direction.
It is rejected if the footprint leaves the configured bounds or the segment
reaches the obstacle clearance margin. The ordinary kinematic predictor still
checks the requested command trajectory; either check can veto the action.

## Failure Semantics

| Evidence condition | Result |
| --- | --- |
| Missing/stale pose or scene | `RISK_UNKNOWN` |
| Missing/unstamped base velocity | `RISK_UNKNOWN` |
| Missing or invalid delay assumption | `RISK_UNKNOWN` |
| Nominal trajectory or stopping envelope violates a known constraint | `REJECTED` |
| Both checks are clear | `APPROVED` |

The trace records the observed state, assumptions, reaction distance, braking
distance, stopping point, and risk trigger.

## Run the Red-Team Example

```bash
python -m raspbot_guardrail replay examples/plans/stop.json examples/scenarios/braking_momentum_risk.json --predictor kinematic --episode "$env:TEMP\kinematic_stop.json" --html "$env:TEMP\kinematic_stop.html"
python -m raspbot_guardrail replay examples/plans/stop.json examples/scenarios/braking_momentum_risk.json --predictor braking_envelope --episode "$env:TEMP\braking_stop.json" --html "$env:TEMP\braking_stop.html"
```

The first replay is approved because a stop command has no future command path
to collide with. The second is rejected because the already-moving base needs
0.48 normalized distance units to stop, while the obstacle clearance is
smaller. This is a controlled model example, not evidence of physical Raspbot
braking performance.

## Independence Boundary

`ReferenceExecutionModel` remains a separately implemented discrete
acceleration-and-delay model for offline outcomes. This predictor never calls
it and does not share its integration loop or parameters. The next evaluation
stage must test the analytical predictor using held-out execution conditions.
