# Evidence Status

| Claim | Evidence label | Status |
| --- | --- | --- |
| Raspbot has a `/cmd_vel` mobile-base command path. | `hardware_validated` | Recorded in `docs/learning-log.md`. |
| `/cmd_vel` uses `geometry_msgs/msg/Twist`. | `hardware_validated` | Recorded in `docs/learning-log.md`. |
| Camera frames can be published through `/image_raw`. | `hardware_validated` | Recorded in `docs/learning-log.md`. |
| Planner-to-executor action boundaries exist in local AI-agent examples. | `source_inspected` | Local source paths identified; public code is not copied. |
| The V1 predictor can reject collision-risk replay examples. | `offline_simulated` | Covered by examples and tests. |
| The V1 guardrail can be evaluated across safe, static-limit, predictive-risk, and under-observed cases. | `offline_simulated` | Covered by `examples/evaluation_cases.json` and `reports/evaluation.md`. |
| Replay episodes expose the predictor assumptions and risk trigger. | `offline_simulated` | Covered by `model_trace` in replay JSON and HTML reports. |
| The prediction layer is model-pluggable. | `offline_simulated` | Covered by the predictor interface, registry, and CLI `--predictor kinematic` option. |
| Approved actions can be converted into generic `/cmd_vel`-style dry-run commands. | `offline_simulated` | Covered by `GuardedCmdVelExecutor`, dry-run reports, and tests. |
| Guardrail decisions can be explained as a staged decision path. | `offline_simulated` | Covered by `decision_path`, HTML reports, `explain` CLI, and tests. |
| The ROS2 backend can drive a physical Raspbot. | `integration_pending` | Adapter boundary only in V1. |
| Normalized V1 command values are calibrated physical velocities. | `integration_pending` | Not claimed. |
