"""Dependency-light arm joint-space safety predictor.

The model intentionally uses planar forward kinematics and simple joint limits
instead of URDF or MoveIt.  It is an offline replay boundary for the next
mobile-manipulator stages, not an execution or collision-planning system.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, isfinite, sin
from typing import Any

from .actions import ArmJointAction, TypedAction
from .policy import Decision
from .predictor import PredictionResult, Scene


@dataclass(frozen=True)
class ArmJointState:
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None = None
    observation_age_s: float | None = 0.0
    max_observation_age_s: float = 1.0


@dataclass(frozen=True)
class JointLimit:
    name: str
    min_position: float
    max_position: float
    max_abs_velocity: float


@dataclass(frozen=True)
class PlanarWorkspaceBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class PlanarArmModel:
    """Simple serial planar-arm model used only for reproducible replay."""

    joint_limits: tuple[JointLimit, ...]
    link_lengths: tuple[float, ...]
    workspace: PlanarWorkspaceBounds


class ArmJointSpacePredictor:
    """Roll out arm joint actions against limits and a simple workspace."""

    def __init__(self, dt_s: float = 0.1) -> None:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        self.dt_s = dt_s

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        if not isinstance(action, ArmJointAction):
            return self._unknown(action, scene, "unsupported_motion_domain")
        if scene.arm_state is None or scene.arm_model is None:
            return self._unknown(action, scene, "missing_arm_state_or_model")

        state = scene.arm_state
        model = scene.arm_model
        if state.observation_age_s is None or state.observation_age_s > state.max_observation_age_s:
            return self._unknown(action, scene, "stale_or_absent_joint_state")
        configuration_error = self._configuration_error(action, state, model)
        if configuration_error is not None:
            return self._unknown(action, scene, configuration_error)

        assert state.velocities is None or len(state.velocities) == len(state.joint_names)
        limits = {limit.name: limit for limit in model.joint_limits}
        initial_check = self._limits_check(state.joint_names, state.positions, tuple(0.0 for _ in state.positions), limits)
        if initial_check is not None:
            return self._rejected(action, scene, tuple(), initial_check)

        steps = max(1, ceil(action.duration_s / self.dt_s))
        velocities = self._command_velocities(action, state.positions)
        trace: list[dict[str, Any]] = []
        for step in range(1, steps + 1):
            t = min(step * self.dt_s, action.duration_s)
            positions = tuple(current + velocity * t for current, velocity in zip(state.positions, velocities))
            limit_error = self._limits_check(state.joint_names, positions, velocities, limits)
            endpoint = self._end_effector(model.link_lengths, positions)
            trace.append(self._trace_point(t, state.joint_names, positions, velocities, endpoint))
            if limit_error is not None:
                return self._rejected(action, scene, tuple(trace), limit_error, steps)
            if not self._inside_workspace(model.workspace, endpoint):
                return self._rejected(action, scene, tuple(trace), "workspace_boundary", steps)

        return PredictionResult(
            Decision.APPROVED,
            "predicted arm joint path stays within configured limits and workspace",
            tuple(),
            None,
            self._model_trace(action, scene, model, steps, None),
            tuple(trace),
        )

    def _command_velocities(self, action: ArmJointAction, current_positions: tuple[float, ...]) -> tuple[float, ...]:
        if action.mode == "velocity":
            return action.values
        return tuple((target - current) / action.duration_s for target, current in zip(action.values, current_positions))

    def _configuration_error(
        self,
        action: ArmJointAction,
        state: ArmJointState,
        model: PlanarArmModel,
    ) -> str | None:
        expected_names = tuple(limit.name for limit in model.joint_limits)
        if not expected_names or len(set(expected_names)) != len(expected_names):
            return "invalid_arm_model_joint_limits"
        if action.joint_names != expected_names or state.joint_names != expected_names:
            return "joint_names_do_not_match_arm_model"
        if len(state.positions) != len(expected_names):
            return "joint_state_positions_do_not_match_arm_model"
        if state.velocities is not None and len(state.velocities) != len(expected_names):
            return "joint_state_velocities_do_not_match_arm_model"
        if len(model.link_lengths) != len(expected_names) or any(not self._finite(length) or length <= 0.0 for length in model.link_lengths):
            return "invalid_arm_model_link_lengths"
        if not all(self._finite(value) for value in state.positions):
            return "joint_state_positions_not_finite"
        if state.velocities is not None and not all(self._finite(value) for value in state.velocities):
            return "joint_state_velocities_not_finite"
        if not self._finite(state.max_observation_age_s) or state.max_observation_age_s < 0.0:
            return "invalid_joint_state_freshness_limit"
        return None

    def _limits_check(
        self,
        names: tuple[str, ...],
        positions: tuple[float, ...],
        velocities: tuple[float, ...],
        limits: dict[str, JointLimit],
    ) -> str | None:
        for name, position, velocity in zip(names, positions, velocities):
            limit = limits[name]
            if not self._finite(position) or not self._finite(velocity):
                return "joint_rollout_not_finite"
            if position < limit.min_position or position > limit.max_position:
                return f"joint_position_limit:{name}"
            if abs(velocity) > limit.max_abs_velocity:
                return f"joint_velocity_limit:{name}"
        return None

    @staticmethod
    def _end_effector(link_lengths: tuple[float, ...], positions: tuple[float, ...]) -> tuple[float, float]:
        x = y = angle = 0.0
        for length, position in zip(link_lengths, positions):
            angle += position
            x += length * cos(angle)
            y += length * sin(angle)
        return x, y

    @staticmethod
    def _inside_workspace(workspace: PlanarWorkspaceBounds, endpoint: tuple[float, float]) -> bool:
        x, y = endpoint
        return workspace.min_x <= x <= workspace.max_x and workspace.min_y <= y <= workspace.max_y

    def _unknown(self, action: TypedAction, scene: Scene, trigger: str) -> PredictionResult:
        return PredictionResult(
            Decision.RISK_UNKNOWN,
            f"arm joint predictor cannot establish safe motion: {trigger}",
            tuple(),
            None,
            self._model_trace(action, scene, scene.arm_model, None, trigger),
        )

    def _rejected(
        self,
        action: ArmJointAction,
        scene: Scene,
        trace: tuple[dict[str, Any], ...],
        trigger: str,
        steps: int | None = None,
    ) -> PredictionResult:
        return PredictionResult(
            Decision.REJECTED,
            f"predicted arm motion violates {trigger}",
            tuple(),
            None,
            self._model_trace(action, scene, scene.arm_model, steps, trigger),
            trace,
        )

    def _model_trace(
        self,
        action: TypedAction,
        scene: Scene,
        model: PlanarArmModel | None,
        steps: int | None,
        risk_trigger: str | None,
    ) -> dict[str, Any]:
        limits = [] if model is None else [
            {
                "name": limit.name,
                "min_position": limit.min_position,
                "max_position": limit.max_position,
                "max_abs_velocity": limit.max_abs_velocity,
            }
            for limit in model.joint_limits
        ]
        workspace = None if model is None else {
            "min_x": model.workspace.min_x,
            "max_x": model.workspace.max_x,
            "min_y": model.workspace.min_y,
            "max_y": model.workspace.max_y,
        }
        state = scene.arm_state
        return {
            "model": "planar_arm_joint_space_v1",
            "motion_domain": action.motion_domain.value,
            "dt_s": self.dt_s,
            "steps": steps,
            "risk_trigger": risk_trigger,
            "assumptions": [
                "serial planar links with revolute joints",
                "linear joint interpolation for position commands",
                "constant commanded joint velocity for velocity commands",
                "workspace check applies to end-effector point only",
            ],
            "input_action": self._action_payload(action),
            "joint_state_evidence": None if state is None else {
                "joint_names": list(state.joint_names),
                "positions": list(state.positions),
                "velocities": None if state.velocities is None else list(state.velocities),
                "observation_age_s": state.observation_age_s,
                "max_observation_age_s": state.max_observation_age_s,
            },
            "joint_limits": limits,
            "link_lengths": [] if model is None else list(model.link_lengths),
            "workspace": workspace,
        }

    @staticmethod
    def _action_payload(action: TypedAction) -> dict[str, Any]:
        if isinstance(action, ArmJointAction):
            return {
                "type": action.action_type,
                "joint_names": list(action.joint_names),
                "mode": action.mode,
                "values": list(action.values),
                "duration_s": action.duration_s,
            }
        return {"type": action.action_type, "duration_s": getattr(action, "duration_s", None)}

    @staticmethod
    def _trace_point(
        t: float,
        names: tuple[str, ...],
        positions: tuple[float, ...],
        velocities: tuple[float, ...],
        endpoint: tuple[float, float],
    ) -> dict[str, Any]:
        return {
            "t": round(t, 6),
            "joint_positions": {name: round(value, 6) for name, value in zip(names, positions)},
            "joint_velocities": {name: round(value, 6) for name, value in zip(names, velocities)},
            "end_effector": {"x": round(endpoint[0], 6), "y": round(endpoint[1], 6)},
        }

    @staticmethod
    def _finite(value: object) -> bool:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and isfinite(float(value))
