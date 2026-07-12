# Conservative Predictor Fusion

The fusion predictor combines the deterministic kinematic baseline and the
experimental learned risk predictor behind the same predictor contract.

```text
kinematic predictor ─┐
                     ├─> conservative fusion ─> arbiter
learned risk model ──┘
```

The initial policy is intentionally conservative:

- any predictor rejection produces `REJECTED`;
- any unknown or model failure produces `RISK_UNKNOWN`;
- only unanimous approval produces `APPROVED`;
- the fusion layer never publishes `/cmd_vel`.

On the current 12-case seed benchmark, fusion has zero dangerous false
negatives and five false rejects. This is evidence of conservative behaviour,
not evidence that the learned model is generally superior. Larger held-out
benchmarks and runtime latency measurements are required before drawing a
stronger conclusion.
