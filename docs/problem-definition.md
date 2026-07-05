# Problem Definition

## Research Question

This project studies a narrow question:

> Can a learned action-conditioned predictor reduce dangerous false negatives
> compared with a deterministic kinematic baseline under dynamics and
> observation uncertainty?

The question is about a command-level guardrail for a ROS2 mobile robot. It is
not a claim that a learned model should replace Nav2, a local controller, or a
formal safety mechanism.

## System Boundary

The guardrail receives a candidate action and the best available scene
evidence:

```text
candidate action + observation
        -> static policy
        -> predictive model(s)
        -> conservative decision
        -> approved command or zero-velocity hold
```

The current command contract is a generic `/cmd_vel`-style boundary. The core
package does not assume a specific Raspbot driver and does not publish live ROS2
messages in V1.

## Decision Semantics

- `APPROVED`: the available checks consider the action safe enough to pass.
- `REJECTED`: a known policy or predictive risk condition was triggered.
- `RISK_UNKNOWN`: required evidence is missing, stale, uncertain, conflicting,
  or the model cannot produce a trustworthy result.

`RISK_UNKNOWN` is a safety decision, not a model success or failure. The
default execution policy for it is a zero-velocity hold.

## Baseline and Ground Truth

The deterministic `kinematic` predictor is the transparent baseline. It is
expected to be fast and auditable, but it assumes that normalized commands are
executed according to an ideal planar motion model.

Future benchmark ground truth must come from an independent reference
execution model. That model will include selected execution effects such as
command delay, acceleration limits, motion error, and observation noise. A
predictor must not generate its own labels and then be evaluated against those
same labels.

## Safety Metrics

The primary safety error is a dangerous false negative:

```text
ground truth = dangerous
model decision = APPROVED
```

The benchmark will also report false rejects, `RISK_UNKNOWN` rate, trajectory
error, clearance error, risk calibration, and inference latency. Overall
accuracy alone is not sufficient because false negatives and false rejects have
different consequences in a guardrail.

## Scope Limits

The project does not currently claim:

- physical safety certification;
- calibrated real-world velocities;
- complete dynamic-obstacle prediction;
- replacement of Nav2 planning or local costmaps;
- a full visual or generative world model;
- hardware validation of the generic ROS2 adapter.

The first learned extension is a structured action-conditioned risk predictor.
An action-conditioned dynamics or latent world-model-style predictor is a
later extension, evaluated through the same interface and benchmark.
