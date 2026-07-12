# Research Benchmark Evaluation

- Predictor: fusion
- Cases: 12
- Dangerous cases: 3
- Dangerous false negatives: 0 (0.0%)
- False rejects: 5 (55.6%)
- Unknown cases: 0 (0.0%)

| Case | Split | Reference | Prediction | Dangerous FN | False reject | Unknown | Reference clearance | Predicted clearance |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| id_clear_01 | in_distribution | SAFE | APPROVED | NO | NO | NO |  |  |
| id_clear_02 | in_distribution | SAFE | APPROVED | NO | NO | NO |  |  |
| id_near_obstacle_01 | in_distribution | SAFE | APPROVED | NO | NO | NO | 1.3223 | 1.2989 |
| id_obstacle_01 | in_distribution | COLLISION | REJECTED | NO | NO | NO | -0.0120 | 0.0250 |
| shift_delay_01 | parameter_shift | SAFE | REJECTED | NO | YES | NO | 0.2895 | 0.1450 |
| shift_accel_01 | parameter_shift | SAFE | REJECTED | NO | YES | NO | 0.6025 | 0.3500 |
| shift_scale_01 | parameter_shift | COLLISION | REJECTED | NO | NO | NO | -0.0020 | 0.0600 |
| shift_turn_01 | parameter_shift | SAFE | APPROVED | NO | NO | NO | 0.3273 | 0.2404 |
| scene_offset_01 | scene_shift | SAFE | REJECTED | NO | YES | NO | 0.1669 | 0.1029 |
| scene_arc_01 | scene_shift | SAFE | REJECTED | NO | YES | NO | 0.1508 | 0.1148 |
| stress_scale_01 | stress_test | COLLISION | REJECTED | NO | NO | NO | -0.0060 | 0.1700 |
| stress_scale_02 | stress_test | SAFE | REJECTED | NO | YES | NO | 0.0104 | 0.0413 |

## Interpretation

Ground truth comes from the independent reference execution model.
This is an offline seed benchmark, not hardware validation or a
statistically sufficient generalization study.