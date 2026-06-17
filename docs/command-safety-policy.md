# Command Safety Policy

The guardrail is reject-first. A command is not safe merely because it is
syntactically valid.

## Static Validation

Static checks reject:

- unknown action types
- malformed command payloads
- negative durations
- over-long commands
- velocity commands outside configured normalized limits
- plans that exceed total duration budget

## Predictive Validation

Predictive checks reject:

- trajectory points outside the supplied boundary
- trajectory points with low obstacle clearance
- commands evaluated against missing pose or missing scene data

Missing or stale observations produce `RISK_UNKNOWN`, not `APPROVED`.

## Decisions

| Decision | Meaning |
| --- | --- |
| `APPROVED` | Static checks pass and the short-horizon predictor finds no scene risk. |
| `REJECTED` | The command is invalid, over budget, collision-predicted, or boundary-violating. |
| `RISK_UNKNOWN` | Required evidence is missing or stale. |
