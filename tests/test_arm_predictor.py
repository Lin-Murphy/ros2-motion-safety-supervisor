import unittest
from dataclasses import replace
from pathlib import Path

from raspbot_guardrail.actions import parse_plan
from raspbot_guardrail.arm_predictor import ArmJointState
from raspbot_guardrail.episode import read_json
from raspbot_guardrail.policy import Decision
from raspbot_guardrail.predictors.registry import build_predictor
from raspbot_guardrail.replay import ReplayEngine
from raspbot_guardrail.scenario import parse_scene


ROOT = Path(__file__).resolve().parents[1]


def _scene(*, observation_age_s: float | None = 0.05, workspace_max_x: float = 1.0):
    return parse_scene({
        "pose": None,
        "bounds": None,
        "obstacles": [],
        "arm": {
            "state": {
                "joint_names": ["shoulder", "elbow"],
                "positions": [0.0, 0.0],
                "velocities": [0.0, 0.0],
                "observation_age_s": observation_age_s,
                "max_observation_age_s": 0.5,
            },
            "model": {
                "joint_limits": [
                    {"name": "shoulder", "min_position": -1.57, "max_position": 1.57, "max_abs_velocity": 1.0},
                    {"name": "elbow", "min_position": -1.57, "max_position": 1.57, "max_abs_velocity": 1.0},
                ],
                "link_lengths": [0.4, 0.3],
                "workspace": {"min_x": -0.2, "max_x": workspace_max_x, "min_y": -1.0, "max_y": 1.0},
            },
        },
    })


def _arm_action(mode: str, values: list[float], duration_s: float = 1.0):
    return parse_plan({
        "actions": [
            {
                "type": "arm_joint",
                "command": {
                    "joint_names": ["shoulder", "elbow"],
                    "mode": mode,
                    "values": values,
                    "duration_s": duration_s,
                },
            }
        ]
    })[0]


class ArmJointSpacePredictorTests(unittest.TestCase):
    def test_safe_position_rollout_returns_state_trace(self) -> None:
        predictor = build_predictor("arm_joint")
        result = predictor.predict(_scene(), _arm_action("position", [0.3, -0.4]))

        self.assertEqual(result.decision, Decision.APPROVED)
        self.assertEqual(result.model_trace["model"], "planar_arm_joint_space_v1")
        self.assertEqual(len(result.state_trace), 10)
        self.assertEqual(result.state_trace[-1]["joint_positions"], {"shoulder": 0.3, "elbow": -0.4})

    def test_joint_position_and_velocity_limits_reject(self) -> None:
        predictor = build_predictor("arm_joint")
        position_result = predictor.predict(_scene(), _arm_action("position", [1.8, 0.0], duration_s=2.0))
        velocity_result = predictor.predict(_scene(), _arm_action("velocity", [1.1, 0.0]))

        self.assertEqual(position_result.decision, Decision.REJECTED)
        self.assertEqual(position_result.model_trace["risk_trigger"], "joint_position_limit:shoulder")
        self.assertEqual(velocity_result.decision, Decision.REJECTED)
        self.assertEqual(velocity_result.model_trace["risk_trigger"], "joint_velocity_limit:shoulder")

    def test_workspace_boundary_rejects(self) -> None:
        result = build_predictor("arm_joint").predict(_scene(workspace_max_x=0.5), _arm_action("position", [0.1, 0.0]))

        self.assertEqual(result.decision, Decision.REJECTED)
        self.assertEqual(result.model_trace["risk_trigger"], "workspace_boundary")

    def test_missing_or_stale_joint_state_is_unknown(self) -> None:
        predictor = build_predictor("arm_joint")
        missing = predictor.predict(parse_scene({"pose": None, "bounds": None, "obstacles": []}), _arm_action("position", [0.1, 0.0]))
        stale = predictor.predict(_scene(observation_age_s=0.6), _arm_action("position", [0.1, 0.0]))
        invalid_state = replace(_scene(), arm_state=ArmJointState(("shoulder",), (0.0,), observation_age_s=0.05))
        invalid = predictor.predict(invalid_state, _arm_action("position", [0.1, 0.0]))

        self.assertEqual(missing.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(missing.model_trace["risk_trigger"], "missing_arm_state_or_model")
        self.assertEqual(stale.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(stale.model_trace["risk_trigger"], "stale_or_absent_joint_state")
        self.assertEqual(invalid.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(invalid.model_trace["risk_trigger"], "joint_names_do_not_match_arm_model")

    def test_examples_replay_with_common_result_and_event_evidence(self) -> None:
        actions = parse_plan(read_json(ROOT / "examples" / "plans" / "arm_joint_safe.json"))
        scene = parse_scene(read_json(ROOT / "examples" / "scenarios" / "simple_arm.json"))
        result = ReplayEngine(predictor=build_predictor("arm_joint"), predictor_name="arm_joint").run("arm_example", actions, scene)

        self.assertEqual(result.final_decision, Decision.APPROVED)
        self.assertEqual(result.episode.events[0].motion_domain, "arm")
        self.assertEqual(len(result.episode.events[0].state_trace), 10)
        self.assertTrue(result.episode.metadata["scene"]["arm_state_present"])
        self.assertTrue(result.episode.metadata["scene"]["arm_model_present"])


if __name__ == "__main__":
    unittest.main()
