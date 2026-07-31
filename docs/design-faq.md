# Design FAQ

## How is this different from Nav2 safety features?

Nav2 is the navigation stack. Its Collision Monitor is an independent runtime
safety node that filters `cmd_vel` using sensor-defined safety zones, while
Keepout Filters represent spatial restrictions in navigation costmaps.

This project is a model-pluggable command gateway around candidate robot
actions. It validates the action, predicts short-horizon consequences, combines
predictor and fault results, and records a replayable decision path.

The two systems can be composed:

```text
Nav2 planner/controller
        -> candidate command
        -> Motion Safety Supervisor
        -> Nav2 Collision Monitor or base controller
        -> mobile base
```

Nav2 mainly answers:

> Is this velocity command safe with respect to the current navigation safety
> zones?

This project asks:

> How should a robot system define an extensible safety boundary where static
> policy, interchangeable predictors, fault handling, and offline evaluation
> use one contract?

The project does not claim to replace Nav2 planning, costmaps, Collision
Monitor, a safety-rated controller, or formal safety certification. Its value
is the software boundary and evidence workflow around prediction models.

## Technical Summary

Nav2 provides navigation and runtime velocity filtering. This project provides
a model-pluggable command safety boundary: candidate actions are checked before
execution, predictor outputs and faults are combined conservatively, and every
decision can be replayed and evaluated. The two layers are complementary rather
than competing implementations.
