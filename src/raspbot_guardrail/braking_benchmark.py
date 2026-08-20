"""Deterministic held-out stopping cases for braking-predictor evaluation."""

from __future__ import annotations

from dataclasses import dataclass

from .actions import StopAction
from .predictor import BaseMotionState, Bounds, CircleObstacle, Pose2D, Scene


@dataclass(frozen=True)
class BrakingBenchmarkCase:
    case_id: str
    split: str
    purpose: str
    scene: Scene
    action: StopAction
    reference_config: dict[str, float]


def generate_braking_benchmark() -> tuple[BrakingBenchmarkCase, ...]:
    """Return fixed stop cases with execution settings held out from prediction.

    The predictor sees only ``Scene.expected_command_delay_s`` and its own
    minimum-deceleration configuration. The reference configuration is used
    exclusively by the independently implemented execution model.
    """

    return (
        _case("nominal_clear", "nominal", "ample stopping space", 0.40, 1.20, 0.10, 0.08, 0.70, 1.00),
        _case("nominal_tight_safe", "nominal", "safe but near stopping boundary", 0.60, 0.80, 0.10, 0.08, 0.80, 1.00),
        _case("nominal_collision", "nominal", "known stopping collision", 0.60, 0.55, 0.10, 0.10, 0.70, 1.00),
        _case("nominal_conservative_reject", "nominal", "safe reference with conservative rejection", 0.60, 0.72, 0.10, 0.08, 0.80, 1.00),
        _case("delay_collision", "held_out_delay", "longer actual delay reaches obstacle", 0.60, 0.63, 0.10, 0.25, 0.80, 1.00),
        _case("delay_clear", "held_out_delay", "longer delay with sufficient clearance", 0.60, 0.95, 0.10, 0.25, 0.80, 1.00),
        _case("deceleration_collision", "held_out_deceleration", "weaker actual braking reaches obstacle", 0.60, 0.72, 0.10, 0.08, 0.35, 1.00),
        _case("deceleration_clear", "held_out_deceleration", "weaker actual braking with sufficient clearance", 0.60, 1.00, 0.10, 0.08, 0.35, 1.00),
        _case("combined_escape", "held_out_combined", "delay and braking shift exceed analytical envelope", 0.70, 1.00, 0.10, 0.25, 0.35, 1.00),
        _case("combined_reject", "held_out_combined", "delay and braking shift still trigger rejection", 0.70, 0.80, 0.10, 0.25, 0.35, 1.00),
        _case("combined_clear", "held_out_combined", "combined shift with ample clearance", 0.70, 1.40, 0.10, 0.25, 0.35, 1.00),
        _case("velocity_scale_collision", "held_out_velocity_scale", "unobserved velocity scale reaches obstacle", 0.55, 0.75, 0.10, 0.15, 0.50, 1.20),
    )


def _case(
    case_id: str,
    split: str,
    purpose: str,
    observed_linear_x: float,
    obstacle_x: float,
    expected_delay_s: float,
    reference_delay_s: float,
    reference_deceleration: float,
    reference_velocity_scale: float,
) -> BrakingBenchmarkCase:
    return BrakingBenchmarkCase(
        case_id=case_id,
        split=split,
        purpose=purpose,
        scene=Scene(
            pose=Pose2D(0.0, 0.0, 0.0),
            bounds=Bounds(-2.0, 2.0, -1.0, 1.0),
            obstacles=(CircleObstacle(obstacle_x, 0.0, 0.10),),
            observation_age_s=0.1,
            max_observation_age_s=1.0,
            base_motion=BaseMotionState(observed_linear_x, 0.0, 0.0, timestamp_s=10.0),
            expected_command_delay_s=expected_delay_s,
        ),
        action=StopAction(0.2),
        reference_config={
            "command_delay_s": reference_delay_s,
            "max_linear_deceleration": reference_deceleration,
            "velocity_scale": reference_velocity_scale,
        },
    )
