import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from raspbot_guardrail.actions import parse_plan
from raspbot_guardrail.safety_decision_engine import SafetyDecisionEngine
from raspbot_guardrail.backends.command import DryRunCommandBackend
from raspbot_guardrail.backends.ros2_cmd_vel import zero_twist
from raspbot_guardrail.backends.ros2_runtime import Ros2CmdVelBackend
from raspbot_guardrail.evaluation import run_evaluation
from raspbot_guardrail.execution import GuardedCmdVelExecutor
from raspbot_guardrail.explanation import format_explanation
from raspbot_guardrail.policy import Decision
from raspbot_guardrail.policy import PolicyResult
from raspbot_guardrail.predictor import KinematicRiskPredictor
from raspbot_guardrail.braking_predictor import BrakingEnvelopePredictor
from raspbot_guardrail.braking_benchmark import generate_braking_benchmark
from raspbot_guardrail.braking_evaluation import run_braking_evaluation
from raspbot_guardrail.predictors.registry import available_predictors, build_predictor
from raspbot_guardrail.report import write_html_report
from raspbot_guardrail.reference_execution import ReferenceExecutionModel, ReferenceOutcome
from raspbot_guardrail.replay import ReplayEngine
from raspbot_guardrail.ros2_node import odometry_to_base_motion, odometry_to_pose, quaternion_to_yaw, twist_to_action
from raspbot_guardrail.research_benchmark import generate_expanded_research_benchmark, generate_research_benchmark
from raspbot_guardrail.research_evaluation import run_research_evaluation
from raspbot_guardrail.gateway import MotionSafetySupervisor
from raspbot_guardrail.fusion import ConservativeFusionPredictor
from raspbot_guardrail.learned_risk import LearnedRiskPredictor, build_default_learned_predictor
from raspbot_guardrail.recorders import InMemoryEventRecorder
from raspbot_guardrail.sources import ReplayActionSource
from raspbot_guardrail.watchdog import OdomSample, RuntimeWatchdog, WatchdogState
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

    def test_required_base_motion_evidence_fails_closed_when_missing(self) -> None:
        action = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })

        result = KinematicRiskPredictor(require_base_motion=True).predict(scene, action)

        self.assertEqual(result.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.model_trace["risk_trigger"], "missing_or_unstamped_base_motion")

    def test_base_motion_evidence_is_replayable(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.0, "vy": 0.0, "wz": 0.0, "duration_s": 0.2}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "base_motion": {"linear_x": 0.8, "linear_y": 0.1, "angular_z": 0.3, "timestamp_s": 42.0},
            "expected_command_delay_s": 0.15,
            "obstacles": [],
        })

        result = ReplayEngine().run("high_speed_evidence", actions, scene)

        trace = result.episode.events[0].model_trace
        self.assertEqual(trace["scene_evidence"]["base_motion"]["linear_x"], 0.8)
        self.assertEqual(trace["execution_assumptions"]["expected_command_delay_s"], 0.15)
        self.assertEqual(result.episode.metadata["scene"]["base_motion"]["angular_z"], 0.3)

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
        self.assertIn("braking_envelope", available_predictors())
        self.assertIn("learned_risk", available_predictors())
        predictor = build_predictor("kinematic")
        self.assertEqual(type(predictor).__name__, "KinematicRiskPredictor")

    def test_braking_envelope_rejects_momentum_that_baseline_does_not_model(self) -> None:
        action = parse_plan({"actions": [{"type": "stop", "duration_s": 0.2}]})[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "base_motion": {"linear_x": 0.6, "linear_y": 0.0, "angular_z": 0.0, "timestamp_s": 10.0},
            "expected_command_delay_s": 0.2,
            "obstacles": [{"x": 0.65, "y": 0.0, "radius": 0.10}],
        })

        baseline = KinematicRiskPredictor().predict(scene, action)
        braking = BrakingEnvelopePredictor(minimum_deceleration=0.5).predict(scene, action)

        self.assertEqual(baseline.decision, Decision.APPROVED)
        self.assertEqual(braking.decision, Decision.REJECTED)
        self.assertEqual(braking.model_trace["risk_trigger"], "braking_envelope_clearance_below_margin")
        self.assertAlmostEqual(braking.model_trace["braking_envelope"]["stopping_distance"], 0.48)

    def test_braking_envelope_requires_motion_and_delay_evidence(self) -> None:
        action = parse_plan({"actions": [{"type": "stop", "duration_s": 0.2}]})[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })

        result = BrakingEnvelopePredictor().predict(scene, action)

        self.assertEqual(result.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.model_trace["risk_trigger"], "missing_or_unstamped_base_motion")

    def test_braking_envelope_approves_clear_stopping_space(self) -> None:
        action = parse_plan({"actions": [{"type": "stop", "duration_s": 0.2}]})[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "base_motion": {"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.0, "timestamp_s": 10.0},
            "expected_command_delay_s": 0.1,
            "obstacles": [{"x": 1.2, "y": 0.0, "radius": 0.10}],
        })

        result = BrakingEnvelopePredictor(minimum_deceleration=0.5).predict(scene, action)

        self.assertEqual(result.decision, Decision.APPROVED)
        self.assertAlmostEqual(result.model_trace["braking_envelope"]["stopping_distance"], 0.06)

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

    def test_reference_execution_exposes_delay_and_acceleration_limited_ground_truth(self) -> None:
        action = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.8, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })

        result = ReferenceExecutionModel(
            dt_s=0.05,
            command_delay_s=0.2,
            max_linear_accel=0.4,
            max_angular_accel=1.0,
        ).execute(scene, action)

        self.assertEqual(result.outcome, ReferenceOutcome.SAFE)
        self.assertGreater(result.trajectory[-1].t, action.duration_s)
        self.assertLess(result.trajectory[3].x, 0.01)
        self.assertEqual(result.model_trace["model"], "reference_acceleration_limited_v1")

    def test_reference_execution_has_independent_collision_outcome(self) -> None:
        action = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.5, "vy": 0.0, "wz": 0.0, "duration_s": 1.5}}
            ]
        })[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [{"x": 0.45, "y": 0.0, "radius": 0.08}],
        })

        result = ReferenceExecutionModel(
            dt_s=0.05,
            command_delay_s=0.0,
            max_linear_accel=2.0,
            max_angular_accel=2.0,
        ).execute(scene, action)

        self.assertEqual(result.outcome, ReferenceOutcome.COLLISION)
        self.assertIsNotNone(result.failure_time_s)
        self.assertLess(result.min_clearance, 0.0)

    def test_reference_execution_integrates_delayed_stop_from_observed_motion(self) -> None:
        action = parse_plan({"actions": [{"type": "stop", "duration_s": 0.2}]})[0]
        scene = parse_scene({
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "base_motion": {"linear_x": 0.6, "linear_y": 0.0, "angular_z": 0.0, "timestamp_s": 10.0},
            "expected_command_delay_s": 0.1,
            "obstacles": [{"x": 0.55, "y": 0.0, "radius": 0.10}],
        })

        result = ReferenceExecutionModel(
            dt_s=0.05,
            command_delay_s=0.2,
            max_linear_deceleration=0.5,
        ).execute(scene, action)

        self.assertEqual(result.outcome, ReferenceOutcome.COLLISION)
        self.assertGreater(result.trajectory[0].x, 0.0)
        self.assertEqual(result.model_trace["max_linear_deceleration"], 0.5)

    def test_braking_benchmark_compares_held_out_execution_conditions(self) -> None:
        first = generate_braking_benchmark()
        second = generate_braking_benchmark()
        summary = run_braking_evaluation(first)

        self.assertEqual(first, second)
        self.assertEqual(len(first), 12)
        self.assertEqual(
            {case.split for case in first},
            {"nominal", "held_out_delay", "held_out_deceleration", "held_out_combined", "held_out_velocity_scale"},
        )
        self.assertEqual(summary.total, 12)
        self.assertGreater(summary.metrics["kinematic"].dangerous_false_negatives, 0)
        self.assertLess(
            summary.metrics["braking_envelope"].dangerous_false_negatives,
            summary.metrics["kinematic"].dangerous_false_negatives,
        )

    def test_research_benchmark_has_deterministic_splits(self) -> None:
        first = generate_research_benchmark()
        second = generate_research_benchmark()

        self.assertEqual(first, second)
        self.assertEqual(len(first), 12)
        self.assertEqual(
            {case.split for case in first},
            {"in_distribution", "parameter_shift", "scene_shift", "stress_test"},
        )
        self.assertEqual(
            {case.split: sum(item.split == case.split for item in first) for case in first},
            {"in_distribution": 4, "parameter_shift": 4, "scene_shift": 2, "stress_test": 2},
        )
        self.assertTrue(all(case.scene.observation_age_s == 0.1 for case in first))
        self.assertTrue(all("command_delay_s" in case.reference_config for case in first))

    def test_expanded_research_benchmark_has_disjoint_evaluation_splits(self) -> None:
        first = generate_expanded_research_benchmark()
        second = generate_expanded_research_benchmark()

        self.assertEqual(first, second)
        self.assertEqual(len(first), 100)
        self.assertEqual(
            {case.split: sum(item.split == case.split for item in first) for case in first},
            {
                "train": 40,
                "validation": 16,
                "test_parameter_shift": 16,
                "test_scene_shift": 16,
                "test_stress": 12,
            },
        )
        self.assertEqual(len({case.case_id for case in first}), len(first))

        predictor = build_default_learned_predictor()
        self.assertEqual(predictor.state.training_cases, 40)

    def test_research_evaluation_reports_baseline_false_negatives(self) -> None:
        summary = run_research_evaluation()

        self.assertEqual(summary.predictor, "kinematic")
        self.assertEqual(summary.total, 12)
        self.assertGreater(summary.dangerous_cases, 0)
        self.assertGreater(summary.dangerous_false_negatives, 0)
        self.assertGreaterEqual(summary.false_rejects, 0)
        self.assertEqual(summary.unknown_cases, 0)

    def test_expanded_research_evaluation_reports_each_split(self) -> None:
        summary = run_research_evaluation(
            cases=generate_expanded_research_benchmark(),
            predictor=build_default_learned_predictor(),
            predictor_name="learned_risk",
        )

        self.assertEqual(summary.total, 100)
        self.assertEqual(summary.split_summary()["train"]["cases"], 40)
        self.assertEqual(summary.split_summary()["test_parameter_shift"]["cases"], 16)
        self.assertEqual(summary.split_summary()["test_scene_shift"]["cases"], 16)
        self.assertEqual(summary.split_summary()["test_stress"]["cases"], 12)

    def test_safety_decision_engine_never_approves_missing_prediction(self) -> None:
        result = SafetyDecisionEngine().decide(
            PolicyResult(Decision.APPROVED, "static policy passed"),
            prediction=None,
        )

        self.assertEqual(result.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.fallback, "zero_velocity_hold")
        self.assertEqual(result.decision_path[-1]["stage"], "final")

    def test_predictor_exception_becomes_unknown_fault_and_hold(self) -> None:
        class ExplodingPredictor:
            def predict(self, scene, action):
                raise RuntimeError("simulated predictor failure")

        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })

        result = ReplayEngine(predictor=ExplodingPredictor(), predictor_name="exploding").run(
            "predictor_failure", actions, scene
        )

        self.assertEqual(result.final_decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.episode.events[0].faults[0]["code"], "predictor_exception")
        self.assertEqual(result.episode.events[0].decision_path[-1]["result"], Decision.RISK_UNKNOWN.value)

    def test_supervisor_and_adapters_share_one_core_decision(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })
        source = ReplayActionSource(actions, scene)
        recorder = InMemoryEventRecorder()
        result = MotionSafetySupervisor(recorder=recorder).evaluate("adapter_clear", source.actions(), source.scene())
        backend = DryRunCommandBackend()
        backend.publish(zero_twist())

        self.assertEqual(result.decision, Decision.APPROVED)
        self.assertEqual(len(recorder.recorded_events), 1)
        self.assertEqual(backend.name, "dry_run_cmd_vel")

    def test_runtime_watchdog_holds_stale_command_and_mismatched_odom(self) -> None:
        command = zero_twist(duration_s=0.2, topic="/cmd_vel")
        stale = RuntimeWatchdog().check(
            type(command)(**{**command.to_dict(), "issued_at_s": 0.0}),
            OdomSample(0.9, 0.0, 0.0, 0.0),
            now_s=1.0,
        )
        self.assertEqual(stale.state, WatchdogState.HOLD)
        self.assertEqual(stale.faults[0].code, "command_stale")

        mismatch = RuntimeWatchdog().check(
            type(command)(**{**command.to_dict(), "issued_at_s": 1.0}),
            OdomSample(1.0, 0.8, 0.0, 0.0),
            now_s=1.1,
        )
        self.assertEqual(mismatch.state, WatchdogState.HOLD)
        self.assertEqual(mismatch.faults[0].code, "odom_command_mismatch")

    def test_backend_unavailable_becomes_unknown_fault(self) -> None:
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })
        backend = DryRunCommandBackend(available_state=False)
        result = GuardedCmdVelExecutor(backend=backend).execute("backend_failure", actions, scene)

        self.assertEqual(result.decision, Decision.RISK_UNKNOWN)
        self.assertEqual(result.faults[0]["code"], "backend_exception")

    def test_ros2_runtime_backend_keeps_core_message_agnostic(self) -> None:
        published = []
        backend = Ros2CmdVelBackend(published.append)
        backend.publish(zero_twist(topic="/safe_cmd_vel"))

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].topic, "/safe_cmd_vel")
        self.assertTrue(backend.available())

    def test_ros2_message_conversion_keeps_core_message_agnostic(self) -> None:
        class Vector:
            def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
                self.x, self.y, self.z, self.w = x, y, z, w

        class TwistLike:
            linear = Vector(0.2, -0.1, 0.0)
            angular = Vector(0.0, 0.0, 0.4)

        action = twist_to_action(TwistLike(), 0.2)
        self.assertEqual(action.vx, 0.2)
        self.assertEqual(action.vy, -0.1)
        self.assertEqual(action.wz, 0.4)
        self.assertEqual(action.duration_s, 0.2)
        self.assertAlmostEqual(quaternion_to_yaw(0.0, 0.0, 0.70710678, 0.70710678), 1.5708, places=3)

        class Pose:
            position = Vector(1.2, -0.4, 0.0)
            orientation = Vector(0.0, 0.0, 0.0, 1.0)

        class PoseWithCovariance:
            pose = Pose()

        class TwistWithCovariance:
            class twist:
                linear = Vector(0.6, -0.2, 0.0)
                angular = Vector(0.0, 0.0, 0.4)

        class Stamp:
            sec = 12
            nanosec = 500_000_000

        class Header:
            stamp = Stamp()

        class OdomLike:
            pose = PoseWithCovariance()
            twist = TwistWithCovariance()
            header = Header()

        pose = odometry_to_pose(OdomLike())
        self.assertEqual((pose.x, pose.y, pose.yaw), (1.2, -0.4, 0.0))
        motion = odometry_to_base_motion(OdomLike())
        self.assertEqual((motion.linear_x, motion.linear_y, motion.angular_z), (0.6, -0.2, 0.4))
        self.assertEqual(motion.timestamp_s, 12.5)

    def test_learned_risk_predictor_uses_common_contract(self) -> None:
        predictor = build_default_learned_predictor()
        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })
        result = predictor.predict(scene, actions[0])

        self.assertIn(result.decision, {Decision.APPROVED, Decision.REJECTED})
        self.assertEqual(result.model_trace["model"], "learned_risk_logistic_v1")
        self.assertIn("risk_probability", result.model_trace)

    def test_conservative_fusion_never_overrules_rejection(self) -> None:
        class ApprovePredictor:
            def predict(self, scene, action):
                return build_predictor("kinematic").predict(scene, action)

        class RejectPredictor:
            def predict(self, scene, action):
                result = build_predictor("kinematic").predict(scene, action)
                return type(result)(Decision.REJECTED, "synthetic rejection", result.trajectory, result.min_clearance, {"model": "reject"})

        actions = parse_plan({
            "actions": [
                {"type": "drive", "command": {"vx": 0.2, "vy": 0.0, "wz": 0.0, "duration_s": 1.0}}
            ]
        })
        scene = parse_scene({
            "pose": {"x": -0.8, "y": 0.0, "yaw": 0.0},
            "bounds": {"min_x": -2.0, "max_x": 2.0, "min_y": -1.0, "max_y": 1.0},
            "observation_age_s": 0.1,
            "obstacles": [],
        })
        result = ConservativeFusionPredictor((("approve", ApprovePredictor()), ("reject", RejectPredictor()))).predict(scene, actions[0])

        self.assertEqual(result.decision, Decision.REJECTED)
        self.assertEqual(result.model_trace["model"], "conservative_fusion_v1")


if __name__ == "__main__":
    unittest.main()
