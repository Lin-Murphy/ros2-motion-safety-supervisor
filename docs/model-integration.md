# Model Integration

The project treats model integration as a predictor boundary, not as direct
robot control.

```text
candidate action
-> static policy
-> predictor interface
-> prediction result
-> guardrail decision
-> replay/evaluation evidence
```

## Current Predictor

V1 registers one predictor:

```text
kinematic -> KinematicRiskPredictor
```

It is deterministic and interpretable. It rolls out a short 2D trajectory and
checks bounds, obstacle clearance, and observation freshness.

## Predictor Interface

All predictors should expose:

```text
predict(scene, action) -> PredictionResult
```

The returned `PredictionResult` must include:

- `decision`: `APPROVED`, `REJECTED`, or `RISK_UNKNOWN`
- `reason`: human-readable decision reason
- `trajectory`: predicted motion trace when available
- `min_clearance`: minimum predicted obstacle clearance when available
- `model_trace`: model name, assumptions, equations or feature summary, and
  risk trigger

## Experimental Learned-Risk Predictor

A small learned predictor can start with replay/synthetic features:

```text
x, y, yaw, vx, vy, wz, duration_s, nearest_obstacle_distance
-> risk_score
```

It should be evaluated against the same manifest as the kinematic and
braking-envelope predictors. The useful comparison is not only accuracy, but
also dangerous false negatives, false rejects, and `RISK_UNKNOWN` behaviour.

This remains experimental. It is not a primary project deliverable until the
project has representative recorded execution data and an explicit held-out
split. It cannot become a second source of command authority.

## World-Model Boundary (Deferred)

An action-conditioned world model could sit behind the same interface:

```text
state_t + action_t -> predicted_state_t+1 / risk_t+1
```

The interface is documented so that such a model would have a safe integration
path:

```text
observation_t + action_t
-> predicted trajectory or predicted state
-> risk decision
-> replayable evidence
```

However, a world model is explicitly outside the current roadmap: synthetic
replay alone would not justify the data collection, model validation, and
distribution-shift claims it requires. Any later model would propose risk
estimates only; it could not bypass the guardrail or directly command
`/cmd_vel`.

## CLI Usage

Current commands accept an explicit predictor name:

```bash
python -m raspbot_guardrail replay examples/plans/collision_risk.json examples/scenarios/simple_room.json --predictor kinematic
python -m raspbot_guardrail evaluate examples/evaluation_cases.json --predictor kinematic
```

Future predictors should be added through the registry, then evaluated through
the same CLI and report pipeline.
