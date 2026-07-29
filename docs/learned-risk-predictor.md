# Learned Risk Predictor

The first learned extension is a small structured risk predictor. It is not a
visual or generative world model and it does not publish commands.

## Contract

```text
observation + candidate action
        -> risk probability
        -> prediction decision
        -> replayable model trace
```

The predictor uses structured features including command magnitude, duration,
kinematic predicted clearance, nearest obstacle geometry, and observation age.
The visible trajectory remains the deterministic rollout used for explanation;
the learned component estimates risk rather than owning execution.

## Training Boundary

The default research model is fitted from the 40-case `train` split of the
expanded benchmark and uses outcomes produced by the independent reference
execution model. The `validation`, `test_parameter_shift`, `test_scene_shift`,
and `test_stress` cases are held out from fitting and expose distribution and
execution mismatch behaviour. This remains a controlled offline experiment,
not a statistically sufficient training study.

## Current Seed Result

On the current 12-case seed benchmark:

```text
kinematic:    dangerous false negatives = 2/3, false rejects = 1
learned_risk: dangerous false negatives = 0/3, false rejects = 5
```

The learned predictor is therefore more conservative on this seed. The result
does not prove general superiority. It demonstrates why the project reports
both safety errors and usability errors instead of optimizing accuracy alone.

## Safety Boundary

The learned predictor can recommend `REJECTED`, but it cannot publish
`/cmd_vel`. A future fusion policy will treat low confidence, timeout, and
disagreement as `RISK_UNKNOWN` with a zero-velocity hold.
