"""Scenario parser for deterministic replay examples."""

from __future__ import annotations

from typing import Any

from .predictor import Bounds, CircleObstacle, Pose2D, Scene


class ScenarioParseError(ValueError):
    """Raised when a scenario JSON file is malformed."""


def parse_scene(raw: dict[str, Any]) -> Scene:
    pose_raw = raw.get("pose")
    bounds_raw = raw.get("bounds")
    if pose_raw is None:
        pose = None
    elif isinstance(pose_raw, dict):
        pose = Pose2D(float(pose_raw["x"]), float(pose_raw["y"]), float(pose_raw["yaw"]))
    else:
        raise ScenarioParseError("pose must be an object or null")

    if bounds_raw is None:
        bounds = None
    elif isinstance(bounds_raw, dict):
        bounds = Bounds(
            min_x=float(bounds_raw["min_x"]),
            max_x=float(bounds_raw["max_x"]),
            min_y=float(bounds_raw["min_y"]),
            max_y=float(bounds_raw["max_y"]),
        )
    else:
        raise ScenarioParseError("bounds must be an object or null")

    obstacles = tuple(
        CircleObstacle(float(item["x"]), float(item["y"]), float(item["radius"]))
        for item in raw.get("obstacles", [])
    )
    return Scene(
        pose=pose,
        bounds=bounds,
        obstacles=obstacles,
        observation_age_s=raw.get("observation_age_s"),
        max_observation_age_s=float(raw.get("max_observation_age_s", 1.0)),
    )
