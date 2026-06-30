import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from raspbot_guardrail.actions import parse_plan
from raspbot_guardrail.evaluation import run_evaluation
from raspbot_guardrail.execution import GuardedCmdVelExecutor
from raspbot_guardrail.explanation import format_explanation
from raspbot_guardrail.policy import Decision
from raspbot_guardrail.predictors.registry import available_predictors, build_predictor
from raspbot_guardrail.report import write_html_report
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
        self.assertEqual(result.episode.events[0].decision_path[-1]["result"], Decision.APPROVED.value)
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
        self.assertEqual(result.episode.events[0].decision_path[1]["stage"], "prediction")
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
        self.assertEqual(summary.total, 10)
        self.assertEqual(summary.passed, 10)

        categories = {case.category for case in summary.cases}
        self.assertIn("predictive_boundary", categories)
        self.assertIn("predictive_arc_collision", categories)
        self.assertIn("multi_step_prediction", categories)

        collision_case = next(
            case for case in summary.cases
            if case.case_id == "collision_risk_predictive_reject"
        )
        self.assertEqual(collision_case.static_decision, Decision.APPROVED.value)
        self.assertEqual(collision_case.predictive_decision, Decision.REJECTED.value)

        boundary_case = next(
            case for case in summary.cases
            if case.case_id == "boundary_violation_predictive_reject"
        )
        self.assertEqual(boundary_case.risk_trigger, "boundary_violation")

        multi_step_case = next(
            case for case in summary.cases
            if case.case_id == "multi_action_second_step_predictive_reject"
        )
        self.assertEqual(multi_step_case.decision_event_index, 1)
        self.assertEqual(multi_step_case.static_decision, Decision.APPROVED.value)
        self.assertEqual(multi_step_case.predictive_decision, Decision.REJECTED.value)
        self.assertEqual(multi_step_case.risk_trigger, "clearance_below_margin")

    def test_predictor_registry_builds_kinematic_predictor(self) -> None:
        self.assertIn("kinematic", available_predictors())
        predictor = build_predictor("kinematic")
        self.assertEqual(type(predictor).__name__, "KinematicRiskPredictor")

    def test_guarded_cmd_vel_dry_run_publishes_approved_drive_and_stop(self) -> None:
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
        result = GuardedCmdVelExecutor().execute("dry_run_clear", actions, scene)
        self.assertEqual(result.decision, Decision.APPROVED)
        self.assertEqual(len(result.published_commands), 2)
        self.assertEqual(result.published_commands[0].topic, "/cmd_vel")
        self.assertEqual(result.published_commands[0].linear_x, 0.2)
        self.assertEqual(result.published_commands[-1].linear_x, 0.0)

    def test_guarded_cmd_vel_dry_run_holds_rejected_action(self) -> None:
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
        result = GuardedCmdVelExecutor().execute("dry_run_collision", actions, scene)
        self.assertEqual(result.decision, Decision.REJECTED)
        self.assertEqual(len(result.published_commands), 1)
        self.assertEqual(result.published_commands[0].linear_x, 0.0)
        self.assertEqual(result.published_commands[0].angular_z, 0.0)

    def test_guarded_cmd_vel_dry_run_does_not_duplicate_existing_stop(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}},
                {"type": "stop", "duration_s": 0.2},
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [{"x": 1.0, "y": 0.7, "radius": 0.1}],
        })
        result = GuardedCmdVelExecutor().execute("dry_run_clear_with_stop", actions, scene)
        self.assertEqual(result.decision, Decision.APPROVED)
        self.assertEqual(len(result.published_commands), 2)
        self.assertEqual(result.reason, "all actions approved; commands emitted to dry-run cmd_vel backend")

    def test_replay_advances_pose_between_approved_actions(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.25, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}},
                {"type": "drive", "command": {"vx": 0.25, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}},
            ]
        })
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -1.5, "max_x": 1.5, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [{"x": 0.65, "y": 0.0, "radius": 0.08}],
        })
        result = ReplayEngine().run("multi_step_collision", actions, scene)

        self.assertEqual(result.final_decision, Decision.REJECTED)
        self.assertEqual(len(result.episode.events), 2)
        self.assertEqual(result.episode.events[0].final_decision, Decision.APPROVED.value)
        self.assertEqual(result.episode.events[1].index, 1)
        self.assertEqual(result.episode.events[1].final_decision, Decision.REJECTED.value)
        self.assertEqual(result.episode.events[1].model_trace["risk_trigger"], "clearance_below_margin")

    def test_explanation_includes_decision_path_and_command_policy(self) -> None:
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
        result = GuardedCmdVelExecutor().execute("explain_collision", actions, scene)
        explanation = format_explanation(result)
        self.assertIn("Decision: REJECTED", explanation)
        self.assertIn("prediction: REJECTED", explanation)
        self.assertIn("zero-velocity hold command", explanation)

    def test_html_report_includes_decision_summary(self) -> None:
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
        result = ReplayEngine().run("report_collision", actions, scene)

        with TemporaryDirectory() as directory:
            report_path = Path(directory) / "report.html"
            write_html_report(report_path, result.episode)
            html = report_path.read_text(encoding="utf-8")

        self.assertIn("Decision Summary", html)
        self.assertIn("Final decision", html)
        self.assertIn("REJECTED", html)
        self.assertIn("clearance_below_margin", html)
        self.assertIn("zero-velocity hold command", html)


if __name__ == "__main__":
    unittest.main()
