"""Scenario parser for deterministic replay examples."""

from __future__ import annotations

from typing import Any

from .arm_predictor import ArmJointState, JointLimit, PlanarArmModel, PlanarWorkspaceBounds
from .predictor import Bounds, CircleObstacle, Pose2D, Scene


class ScenarioParseError(ValueError):
    """Raised when a scenario JSON file is malformed."""


def _number_tuple(value: Any, field: str) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise ScenarioParseError(f"{field} must be a list")
    try:
        return tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ScenarioParseError(f"{field} must contain numbers") from exc


def _string_tuple(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ScenarioParseError(f"{field} must be a list of strings")
    return tuple(value)


def _parse_arm(raw: Any) -> tuple[ArmJointState | None, PlanarArmModel | None]:
    if raw is None:
        return None, None
    if not isinstance(raw, dict):
        raise ScenarioParseError("arm must be an object or null")
    state_raw = raw.get("state")
    model_raw = raw.get("model")
    if not isinstance(state_raw, dict) or not isinstance(model_raw, dict):
        raise ScenarioParseError("arm.state and arm.model must be objects")
    velocities_raw = state_raw.get("velocities")
    state = ArmJointState(
        joint_names=_string_tuple(state_raw.get("joint_names"), "arm.state.joint_names"),
        positions=_number_tuple(state_raw.get("positions"), "arm.state.positions"),
        velocities=None if velocities_raw is None else _number_tuple(velocities_raw, "arm.state.velocities"),
        observation_age_s=None if state_raw.get("observation_age_s") is None else float(state_raw["observation_age_s"]),
        max_observation_age_s=float(state_raw.get("max_observation_age_s", 1.0)),
    )
    limits_raw = model_raw.get("joint_limits")
    if not isinstance(limits_raw, list):
        raise ScenarioParseError("arm.model.joint_limits must be a list")
    try:
        limits = tuple(
            JointLimit(
                name=str(item["name"]),
                min_position=float(item["min_position"]),
                max_position=float(item["max_position"]),
                max_abs_velocity=float(item["max_abs_velocity"]),
            )
            for item in limits_raw
        )
        workspace_raw = model_raw["workspace"]
        if not isinstance(workspace_raw, dict):
            raise TypeError("workspace must be an object")
        model = PlanarArmModel(
            joint_limits=limits,
            link_lengths=_number_tuple(model_raw.get("link_lengths"), "arm.model.link_lengths"),
            workspace=PlanarWorkspaceBounds(
                min_x=float(workspace_raw["min_x"]),
                max_x=float(workspace_raw["max_x"]),
                min_y=float(workspace_raw["min_y"]),
                max_y=float(workspace_raw["max_y"]),
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ScenarioParseError("arm.model is malformed") from exc
    return state, model


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
    arm_state, arm_model = _parse_arm(raw.get("arm"))
    return Scene(
        pose=pose,
        bounds=bounds,
        obstacles=obstacles,
        observation_age_s=raw.get("observation_age_s"),
        max_observation_age_s=float(raw.get("max_observation_age_s", 1.0)),
        arm_state=arm_state,
        arm_model=arm_model,
    )
