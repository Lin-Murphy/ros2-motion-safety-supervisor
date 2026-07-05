# Relationship to ROS2 Virtual Walls and Nav2

The project is complementary to Nav2 rather than a replacement for it.

## Different Abstraction Levels

Nav2 Keepout Filters represent spatial restrictions in costmaps. They answer a
map-level question:

```text
Which areas should planning or local navigation avoid?
```

This project answers a command-level question:

```text
If this candidate velocity command runs for this horizon,
what short-term state and risk can be predicted?
```

The V1 rectangular bounds and static obstacles are deliberately simple spatial
constraints, so they overlap with the basic idea of a virtual wall. The
project's distinct boundary is the model-pluggable, replayable check placed
between a candidate command and the ROS2 execution adapter.

## Intended Composition

```text
Nav2 planner / controller or teleoperation node
        -> candidate /cmd_vel
        -> command guardrail
        -> approved /cmd_vel or zero-velocity hold
```

Nav2 remains responsible for navigation and costmap-based planning. The
guardrail can provide an independent final command check, particularly for
malformed actions, short-horizon execution risk, stale evidence, or disagreement
between predictive models.

## Honest Boundary

The project should not claim that its V1 predictor performs better than Nav2's
local collision checking. That comparison requires a separately designed
benchmark and a real integration. The current research comparison is narrower:
deterministic kinematic baseline versus learned action-conditioned predictor
under controlled execution and observation uncertainty.
