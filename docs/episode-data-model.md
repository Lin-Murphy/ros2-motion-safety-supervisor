# Episode Data Model

The project records every replay as an episode:

```text
observation + action + prediction + decision + timestamp/metadata
```

The Day 0 event schema is:

```json
{
  "index": 0,
  "action_type": "drive",
  "static_decision": "APPROVED",
  "predictive_decision": "REJECTED",
  "final_decision": "REJECTED",
  "reason": "predicted obstacle collision or low clearance",
  "decision_path": [
    {
      "stage": "static_policy",
      "result": "APPROVED",
      "reason": "static drive policy passed"
    },
    {
      "stage": "prediction",
      "result": "REJECTED",
      "reason": "predicted obstacle collision or low clearance"
    },
    {
      "stage": "final",
      "result": "REJECTED",
      "reason": "predicted obstacle collision or low clearance"
    }
  ]
}
```

This schema is intentionally close to future robot-learning data:

```text
observation_t + action_t + observation_t+1 + metadata
```

V1 records command-level decisions first. Later versions can add image features,
line error, obstacle distance, `/cmd_vel`, or ROS2 timestamps.
