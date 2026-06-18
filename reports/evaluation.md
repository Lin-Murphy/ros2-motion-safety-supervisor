# Guardrail Evaluation Report

- Cases: 5
- Passed: 5
- Pass rate: 100%

| Case | Category | Expected | Actual | Static | Predictive | Result | Min clearance | Reason |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| clear_drive_safe | safe_path | APPROVED | APPROVED | APPROVED | APPROVED | PASS | 0.250 | predicted path stays within scene constraints |
| collision_risk_predictive_reject | predictive_collision | REJECTED | REJECTED | APPROVED | REJECTED | PASS | 0.045 | predicted obstacle collision or low clearance |
| missing_pose_unknown | under_observed_scene | RISK_UNKNOWN | RISK_UNKNOWN | APPROVED | RISK_UNKNOWN | PASS |  | pose or bounds missing |
| stale_observation_unknown | stale_observation | RISK_UNKNOWN | RISK_UNKNOWN | APPROVED | RISK_UNKNOWN | PASS |  | scene observation stale or absent |
| overspeed_static_reject | static_limit | REJECTED | REJECTED | REJECTED | RISK_UNKNOWN | PASS |  | vx exceeds limit |

## Scope

This report validates deterministic V1 guardrail behaviour against explicit
offline replay cases. It checks decision consistency, not real-world
physical calibration.

The collision-risk case is especially useful because the command passes the
static policy but is rejected by the predictive layer.
