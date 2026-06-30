# Guardrail Evaluation Report

- Predictor: kinematic
- Cases: 10
- Passed: 10
- Pass rate: 100%

| Case | Category | Event | Expected | Actual | Static | Predictive | Trigger | Result | Min clearance | Reason |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| clear_drive_safe | safe_path | 0 | APPROVED | APPROVED | APPROVED | APPROVED |  | PASS | 0.250 | predicted path stays within scene constraints |
| collision_risk_predictive_reject | predictive_collision | 0 | REJECTED | REJECTED | APPROVED | REJECTED | clearance_below_margin | PASS | 0.045 | predicted obstacle collision or low clearance |
| missing_pose_unknown | under_observed_scene | 0 | RISK_UNKNOWN | RISK_UNKNOWN | APPROVED | RISK_UNKNOWN | missing_pose_or_bounds | PASS |  | pose or bounds missing |
| stale_observation_unknown | stale_observation | 0 | RISK_UNKNOWN | RISK_UNKNOWN | APPROVED | RISK_UNKNOWN | stale_or_absent_observation | PASS |  | scene observation stale or absent |
| overspeed_static_reject | static_limit | 0 | REJECTED | REJECTED | REJECTED | RISK_UNKNOWN |  | PASS |  | vx exceeds limit |
| boundary_violation_predictive_reject | predictive_boundary | 0 | REJECTED | REJECTED | APPROVED | REJECTED | boundary_violation | PASS |  | predicted boundary violation |
| turning_arc_predictive_reject | predictive_arc_collision | 0 | REJECTED | REJECTED | APPROVED | REJECTED | clearance_below_margin | PASS | 0.029 | predicted obstacle collision or low clearance |
| multi_action_second_step_predictive_reject | multi_step_prediction | 1 | REJECTED | REJECTED | APPROVED | REJECTED | clearance_below_margin | PASS | 0.040 | predicted obstacle collision or low clearance |
| missing_bounds_unknown | under_observed_scene | 0 | RISK_UNKNOWN | RISK_UNKNOWN | APPROVED | RISK_UNKNOWN | missing_pose_or_bounds | PASS |  | pose or bounds missing |
| negative_duration_static_reject | static_limit | 0 | REJECTED | REJECTED | REJECTED | RISK_UNKNOWN |  | PASS |  | drive duration must be positive |

## Scope

This report validates deterministic V1 guardrail behaviour against explicit
offline replay cases. It checks decision consistency, not real-world
physical calibration.

The collision-risk case is especially useful because the command passes the
static policy but is rejected by the predictive layer.
