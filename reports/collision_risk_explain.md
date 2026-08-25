# Guardrail Explanation: collision_risk

- Decision: REJECTED
- Reason: predicted obstacle collision or low clearance
- Predictor: kinematic
- Topic: /cmd_vel
- Backend-accepted commands: 1

## Decision Path

### Event 0: drive
1. static_policy: APPROVED - static drive policy passed
2. prediction: REJECTED - predicted obstacle collision or low clearance
3. final: REJECTED - predicted obstacle collision or low clearance

Risk details:
- min_clearance: 0.045
- risk_trigger: clearance_below_margin
- clearance_margin: 0.05
- horizon_s: 2.0

## Execution Evidence

The guardrail did not approve the action sequence, so the dry-run backend was asked to accept only a zero-velocity hold command.
- Backend status: accepted_by_backend
- Post-execution observation: not_collected
- Note: backend acceptance does not prove that the physical robot executed or stopped.

## Backend-Accepted Commands

0. topic=/cmd_vel, linear_x=0.000, linear_y=0.000, angular_z=0.000, duration_s=0.200
