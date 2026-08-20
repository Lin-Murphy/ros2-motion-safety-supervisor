# Decision, Dispatch, and Observation Evidence

## Goal

A zero-velocity hold request is not proof that the robot physically stopped.
Each guarded execution therefore records a deliberately separated evidence
chain:

```text
candidate actions
  -> supervisor decision
  -> requested safe or hold commands
  -> commands accepted by the backend adapter
  -> post-dispatch observed base motion, if separately collected
```

This makes it possible to inspect a failure without confusing a policy
decision, a software adapter result, and physical robot behaviour.

## `execution_evidence` Schema

Execution JSON includes an `execution_evidence` object with:

- `candidate_actions`: the supplied typed actions;
- `supervisor_decision`: `APPROVED`, `REJECTED`, or `RISK_UNKNOWN`;
- `requested_safe_commands`: commands the executor tried to send, including a
  requested zero-velocity hold on rejection or fault;
- `backend_accepted_commands`: commands the output adapter reported as
  accepted;
- `backend_status`: adapter fault and fallback-hold status;
- `post_execution_observation`: an optional later `BaseMotionState` supplied
  by the caller;
- `post_execution_observation_status`: `reported_by_caller` or
  `not_collected`.

The executor never invents the post-dispatch state from a successful publish.

## Meaning and Failure Semantics

| Field | Establishes | Does not establish |
| --- | --- | --- |
| Supervisor decision | The safety-core policy outcome for the recorded inputs | Physical robot behaviour |
| Requested command | What the executor attempted to send | Adapter receipt or motor response |
| Backend-accepted command | Adapter-level callback success | ROS delivery, driver execution, or stopping |
| Post-dispatch observation | A caller-reported later base-motion sample | A safety guarantee outside its timestamp and measurement quality |

If the backend is unavailable or raises an exception, the evidence still shows
the requested zero-velocity hold but leaves `backend_accepted_commands` empty.
If the fallback hold also fails, `backend_status.hold_failed` records that
separate fault.

## Runtime Boundary

The optional ROS2 node already writes the enriched episode returned by the
executor. It does not wait for a later odometry sample after every dispatch,
so `post_execution_observation_status` remains `not_collected` in the current
node path.

Collecting a later observation is a controlled runtime-integration task. This
contract is interface and replay evidence only; it is not live hardware
validation, emergency-stop evidence, or safety certification.
