"""Deterministic parameterized benchmark cases for predictor research."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .actions import DriveAction
from .predictor import Bounds, CircleObstacle, Pose2D, Scene


@dataclass(frozen=True)
class ResearchBenchmarkCase:
    case_id: str
    split: str
    purpose: str
    action: DriveAction
    scene: Scene
    reference_config: dict[str, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.case_id,
            "split": self.split,
            "purpose": self.purpose,
            "action": {
                "type": "drive",
                "vx": self.action.vx,
                "vy": self.action.vy,
                "wz": self.action.wz,
                "duration_s": self.action.duration_s,
            },
            "scene": {
                "pose": None if self.scene.pose is None else asdict(self.scene.pose),
                "bounds": None if self.scene.bounds is None else asdict(self.scene.bounds),
                "obstacles": [asdict(obstacle) for obstacle in self.scene.obstacles],
                "observation_age_s": self.scene.observation_age_s,
                "max_observation_age_s": self.scene.max_observation_age_s,
            },
            "reference_config": self.reference_config,
        }


def generate_research_benchmark() -> tuple[ResearchBenchmarkCase, ...]:
    """Return a deterministic benchmark with held-out scenario families."""

    bounds = Bounds(-2.0, 2.0, -1.2, 1.2)
    cases = [
        _case("id_clear_01", "in_distribution", "clear straight motion", DriveAction(0.20, 0.0, 0.0, 1.0), Pose2D(-0.8, -0.5, 0.0), bounds, (), _config(0.05, 1.0, 1.5, 0.8)),
        _case("id_clear_02", "in_distribution", "clear turning motion", DriveAction(0.18, 0.0, 0.30, 1.2), Pose2D(-0.8, 0.4, 0.0), bounds, (), _config(0.05, 1.0, 1.5, 0.8)),
        _case("id_near_obstacle_01", "in_distribution", "near but clear obstacle", DriveAction(0.22, 0.0, 0.0, 1.0), Pose2D(-0.8, 0.0, 0.0), bounds, (CircleObstacle(0.9, 0.55, 0.10),), _config(0.05, 1.0, 1.5, 0.8)),
        _case("id_obstacle_01", "in_distribution", "direct obstacle collision", DriveAction(0.35, 0.0, 0.0, 1.5), Pose2D(-0.2, 0.0, 0.0), bounds, (CircleObstacle(0.35, 0.0, 0.10),), _config(0.05, 1.0, 1.5, 0.8)),
        _case("shift_delay_01", "parameter_shift", "long command delay", DriveAction(0.35, 0.0, 0.0, 1.5), Pose2D(-0.6, 0.0, 0.0), bounds, (CircleObstacle(0.35, 0.0, 0.10),), _config(0.25, 1.0, 1.5, 0.4)),
        _case("shift_accel_01", "parameter_shift", "slow acceleration", DriveAction(0.40, 0.0, 0.0, 1.2), Pose2D(-0.8, 0.0, 0.0), bounds, (CircleObstacle(0.25, 0.0, 0.08),), _config(0.10, 1.0, 1.5, 0.25)),
        _case("shift_scale_01", "parameter_shift", "high velocity tracking near clearance boundary", DriveAction(0.28, 0.0, 0.0, 1.5), Pose2D(-0.7, 0.0, 0.0), bounds, (CircleObstacle(0.06, 0.0, 0.10),), _config(0.05, 1.40, 1.5, 0.8)),
        _case("shift_turn_01", "parameter_shift", "turning with low angular acceleration", DriveAction(0.25, 0.0, 0.75, 1.5), Pose2D(-0.7, -0.3, 0.0), bounds, (CircleObstacle(0.0, 0.25, 0.12),), _config(0.10, 1.0, 0.35, 0.6)),
        _case("scene_offset_01", "scene_shift", "obstacle offset from training layout", DriveAction(0.34, 0.0, 0.0, 1.8), Pose2D(-0.7, 0.0, 0.0), bounds, (CircleObstacle(0.25, 0.18, 0.10),), _config(0.12, 1.0, 1.0, 0.6)),
        _case("scene_arc_01", "scene_shift", "arc approaching an offset obstacle", DriveAction(0.24, 0.0, 0.85, 1.6), Pose2D(-0.7, -0.45, 0.0), bounds, (CircleObstacle(-0.1, 0.0, 0.10),), _config(0.12, 1.0, 0.8, 0.6)),
        _case("stress_scale_01", "stress_test", "high execution speed crosses clearance boundary", DriveAction(0.30, 0.0, 0.0, 2.0), Pose2D(-0.7, 0.0, 0.0), bounds, (CircleObstacle(0.35, 0.0, 0.10),), _config(0.05, 1.80, 1.5, 0.8)),
        _case("stress_scale_02", "stress_test", "high speed turning", DriveAction(0.28, 0.0, 0.70, 1.8), Pose2D(-0.7, -0.2, 0.0), bounds, (CircleObstacle(0.0, 0.15, 0.10),), _config(0.10, 1.25, 0.7, 0.5)),
    ]
    return tuple(cases)


def generate_expanded_research_benchmark() -> tuple[ResearchBenchmarkCase, ...]:
    """Return a larger deterministic benchmark with held-out scenario families.

    Cases are generated from disjoint parameter ranges rather than randomly
    shuffled rows. This makes the held-out splits useful for testing whether a
    predictor transfers across execution and scene changes.
    """

    bounds = Bounds(-2.0, 2.0, -1.2, 1.2)
    cases: list[ResearchBenchmarkCase] = []
    split_sizes = {
        "train": 40,
        "validation": 16,
        "test_parameter_shift": 16,
        "test_scene_shift": 16,
        "test_stress": 12,
    }
    for split, size in split_sizes.items():
        for index in range(size):
            cases.append(_generated_case(split, index, bounds))
    return tuple(cases)


def write_benchmark_cases(path: Path, cases: tuple[ResearchBenchmarkCase, ...] | None = None) -> None:
    selected = cases or generate_research_benchmark()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"cases": [case.to_dict() for case in selected]}, indent=2), encoding="utf-8")


def _config(command_delay_s: float, velocity_scale: float, max_angular_accel: float, max_linear_accel: float) -> dict[str, float]:
    return {
        "command_delay_s": command_delay_s,
        "velocity_scale": velocity_scale,
        "max_angular_accel": max_angular_accel,
        "max_linear_accel": max_linear_accel,
    }


def _case(
    case_id: str,
    split: str,
    purpose: str,
    action: DriveAction,
    pose: Pose2D,
    bounds: Bounds,
    obstacles: tuple[CircleObstacle, ...],
    reference_config: dict[str, float],
) -> ResearchBenchmarkCase:
    return ResearchBenchmarkCase(
        case_id=case_id,
        split=split,
        purpose=purpose,
        action=action,
        scene=Scene(pose, bounds, obstacles, observation_age_s=0.1),
        reference_config=reference_config,
    )


def _generated_case(split: str, index: int, bounds: Bounds) -> ResearchBenchmarkCase:
    profiles = (
        (0.18, 0.00, 0.00, 0.9),
        (0.26, 0.00, 0.00, 1.2),
        (0.22, 0.00, 0.35, 1.3),
        (0.30, 0.00, 0.60, 1.4),
    )
    vx, vy, wz, duration_s = profiles[index % len(profiles)]
    if split in {"test_parameter_shift", "test_stress"} and index % 2 == 1:
        vx, vy, wz, duration_s = (0.35, 0.0, 0.0, 2.0)
    start_x = -1.1 + 0.16 * (index % 5)
    start_y = -0.45 + 0.18 * (index % 4)
    action = DriveAction(vx, vy, wz, duration_s)
    pose = Pose2D(start_x, start_y, 0.0)
    travel = max(0.18, abs(vx) * duration_s * 0.75)
    path_y = start_y + (0.35 if wz > 0.0 else 0.0)
    is_dangerous = index % 2 == 1

    if is_dangerous:
        obstacle_x = start_x + travel
        if split in {"test_parameter_shift", "test_stress"}:
            # Keep the obstacle outside the nominal rollout but inside the
            # delayed, speed-scaled reference execution.
            obstacle_x = start_x + abs(vx) * duration_s + 0.36
        obstacle = CircleObstacle(obstacle_x, path_y, 0.12)
    else:
        obstacle = CircleObstacle(start_x + travel, start_y + 0.75, 0.10)

    config = _config(0.05, 1.0, 1.5, 0.8)
    if split == "validation":
        config = _config(0.08, 1.05, 1.2, 0.7)
    elif split == "test_parameter_shift":
        config = _config(0.22, 1.80, 1.2, 0.8)
    elif split == "test_scene_shift":
        obstacle = CircleObstacle(start_x + travel * 0.85, path_y + (0.08 if is_dangerous else 0.55), 0.12)
        config = _config(0.12, 1.0, 1.0, 0.6)
    elif split == "test_stress":
        config = _config(0.28, 1.80, 1.5, 1.0)

    return _case(
        f"{split}_{index:03d}",
        split,
        f"generated {split} case {index}",
        action,
        pose,
        bounds,
        (obstacle,),
        config,
    )
