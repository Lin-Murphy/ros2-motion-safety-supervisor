# Evaluation Benchmark

The evaluation manifest is a small deterministic benchmark for the V1 action
guardrail. It is designed to test whether the same command pipeline can handle
safe commands, static policy failures, predictive failures, missing evidence,
and multi-action sequences.

## Case Categories

| Category | What it tests | Expected guardrail behaviour |
| --- | --- | --- |
| `safe_path` | A valid command in a clear scene. | Approve the command. |
| `static_limit` | Command values that violate typed action limits. | Reject before prediction. |
| `predictive_collision` | A statically valid straight command that approaches an obstacle. | Reject after trajectory rollout. |
| `predictive_boundary` | A statically valid command that leaves configured bounds. | Reject after trajectory rollout. |
| `predictive_arc_collision` | A turning command whose arc approaches an obstacle. | Reject after trajectory rollout. |
| `multi_step_prediction` | A sequence where the second action is risky only after the first approved rollout advances the pose. | Reject at the later action. |
| `under_observed_scene` | Missing pose or bounds evidence. | Return `RISK_UNKNOWN`. |
| `stale_observation` | Scene evidence older than the configured freshness limit. | Return `RISK_UNKNOWN`. |

## Why This Matters

The benchmark separates static policy checks from predictive risk checks. A
useful red-team case is one where the action is well formed and within speed
limits, but the predicted trajectory still crosses a clearance or boundary
constraint. Those cases show why the predictive layer exists.

The benchmark also records which replay event produced the decision. This is
important for action sequences: the first command may be acceptable, while a
later command becomes risky after the replay pose has advanced.

## Regression Result

The generated report in `reports/evaluation.md` records the current V1 result:

```text
passed=10/10
predictor=kinematic
```

This is deterministic replay evidence. It validates decision consistency across
the configured regression cases; it does not claim physical calibration,
generalization, or learned-model improvement.

## Research Benchmark (Next Phase)

The next benchmark layer will generate cases from an independent reference
execution model. It varies command delay, acceleration limits, execution
error, and scene layout. The reference outcome provides ground truth for
collision, boundary violation, and minimum clearance.

The initial deterministic research cases are stored in
`benchmarks/research_cases.json` and are split into:

| Split | Purpose |
| --- | --- |
| `in_distribution` | Normal clear, turning, and direct-risk cases. |
| `parameter_shift` | Changed delay, acceleration, or velocity tracking. |
| `scene_shift` | Changed obstacle layout and turning geometry. |
| `stress_test` | Large execution mismatch intended to expose optimistic predictions. |

The cases are a seed benchmark, not a statistically sufficient dataset. The
expanded benchmark adds 100 deterministic generated cases with disjoint
training, validation, and held-out test families. It is still a controlled
offline experiment, not evidence of physical safety.

The expanded benchmark can be run with:

```bash
python -m raspbot_guardrail research-evaluate --benchmark expanded --predictor kinematic
python -m raspbot_guardrail research-evaluate --benchmark expanded --predictor learned_risk
```

The learned predictor is fitted only from the `train` split. The validation
and test splits are never used to fit its weights. `test_parameter_shift`,
`test_scene_shift`, and `test_stress` are held out to measure transfer across
execution and scene changes.

The current seed baseline report is `reports/research_evaluation.md`. It shows
that the kinematic baseline can be optimistic under execution mismatch: the
current seed contains two dangerous false negatives. This is a failure-mode
demonstration, not a claim about real hardware or final model performance.

The kinematic baseline and future learned predictors will be evaluated on the
same cases. The primary metric will be dangerous false negatives, defined as a
ground-truth dangerous outcome that the model approves. False rejects,
`RISK_UNKNOWN` rate, trajectory error, clearance error, calibration, and
inference latency will be reported alongside it.
