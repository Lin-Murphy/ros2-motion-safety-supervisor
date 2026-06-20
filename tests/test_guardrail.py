import unittest
from pathlib import Path

from raspbot_guardrail.actions import parse_plan
from raspbot_guardrail.evaluation import run_evaluation
from raspbot_guardrail.policy import Decision
from raspbot_guardrail.predictors.registry import available_predictors, build_predictor
from raspbot_guardrail.replay import ReplayEngine
from raspbot_guardrail.scenario import parse_scene


class GuardrailTests(unittest.TestCase):
    def test_clear_drive_is_approved(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [{"x": 1.0, "y": 0.7, "radius": 0.1}],
        })
        result = ReplayEngine().run("clear", actions, scene)
        self.assertEqual(result.final_decision, Decision.APPROVED)
        self.assertEqual(result.episode.metadata["predictor"], "kinematic")
        self.assertGreater(len(result.episode.events[0].trajectory), 0)
        self.assertEqual(result.episode.events[0].model_trace["model"], "kinematic_unicycle_v1")
        self.assertIn("equations", result.episode.events[0].model_trace)

    def test_collision_risk_is_rejected(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.45, "vy": 0.0, "wz": 0.0, "duration_s": 2.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [{"x": 0.75, "y": 0.0, "radius": 0.12}],
        })
        result = ReplayEngine().run("collision", actions, scene)
        self.assertEqual(result.final_decision, Decision.REJECTED)
        self.assertIsNotNone(result.episode.events[0].min_clearance)
        self.assertEqual(result.episode.events[0].model_trace["risk_trigger"], "clearance_below_margin")

    def test_missing_pose_is_unknown(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": None,
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })
        result = ReplayEngine().run("unknown", actions, scene)
        self.assertEqual(result.final_decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.episode.events[0].trajectory, [])

    def test_evaluation_manifest_passes(self) -> None:
        summary = run_evaluation(Path("examples/evaluation_cases.json"))
        self.assertEqual(summary.predictor, "kinematic")
        self.assertEqual(summary.total, 5)
        self.assertEqual(summary.passed, 5)

        collision_case = next(
            case for case in summary.cases
            if case.case_id == "collision_risk_predictive_reject"
        )
        self.assertEqual(collision_case.static_decision, Decision.APPROVED.value)
        self.assertEqual(collision_case.predictive_decision, Decision.REJECTED.value)

    def test_predictor_registry_builds_kinematic_predictor(self) -> None:
        self.assertIn("kinematic", available_predictors())
        predictor = build_predictor("kinematic")
        self.assertEqual(type(predictor).__name__, "KinematicRiskPredictor")


if __name__ == "__main__":
    unittest.main()
