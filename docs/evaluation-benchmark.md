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

## Current Result

The generated report in `reports/evaluation.md` records the current V1 result:

```text
passed=10/10
predictor=kinematic
```

This is deterministic replay evidence. It validates decision consistency across
the configured cases; it does not claim physical calibration.
