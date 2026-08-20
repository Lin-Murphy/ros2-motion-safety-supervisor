# Independent Braking Evaluation

## Question

Does the analytical braking envelope reduce dangerous approvals relative to the
kinematic baseline when actual execution differs from its assumptions?

## Protocol

Both predictors receive the same pose, obstacle, observed base velocity, and
candidate `stop` action. The braking predictor sees only the scene's expected
delay (`0.10`) and its configured minimum deceleration (`0.50`).

Separately, `ReferenceExecutionModel` starts from the observed motion, holds
that motion during an execution delay, then integrates a discrete deceleration
loop. Its delay, deceleration, and velocity-scale settings are not called by
or passed to the predictor.

The 12 deterministic cases cover nominal stopping conditions plus held-out
command-delay, deceleration, combined, and velocity-scale changes.

Run the comparison from PowerShell:

```powershell
python -m raspbot_guardrail braking-evaluate --json "$env:TEMP\braking_evaluation.json" --markdown "$env:TEMP\braking_evaluation.md"
```

## Current Controlled Result

On this fixed offline benchmark:

| Predictor | Dangerous cases | Dangerous false negatives | False rejects | Unknown |
| --- | ---: | ---: | ---: | ---: |
| `kinematic` | 6 | 6 | 0 | 0 |
| `braking_envelope` | 6 | 2 | 1 | 0 |

The braking envelope catches the nominal, delay-only, deceleration-only, and
one combined-shift stopping risks that the baseline approves. It is also more
conservative once: `nominal_conservative_reject` is safe in the reference
execution but rejected by the envelope.

Two dangerous cases still pass the braking check:

- `combined_escape`: actual delay and deceleration degrade together beyond the
  predictor's conservative configuration;
- `velocity_scale_collision`: actual velocity is scaled beyond the observed
  base-motion assumption.

## Interpretation

The result supports a narrow engineering claim: on this deterministic,
independently executed stop benchmark, the analytical envelope reduces
dangerous approvals compared with the current kinematic baseline, at a visible
false-reject cost. It does **not** establish calibrated physical units, real
Raspbot stopping performance, generalization to arbitrary scenes, or safety
certification.

The remaining false negatives define the next evidence gap rather than an
invitation to add a learned world model. Any later extension must improve these
cases on a separately designed protocol.
