"""Conservative fusion of multiple predictive model outputs."""

from __future__ import annotations

from typing import Any

from .actions import TypedAction
from .policy import Decision
from .predictor import PredictionResult, Scene
from .predictors.base import Predictor


class ConservativeFusionPredictor:
    """Combines predictors without allowing a model to overrule a rejection."""

    def __init__(self, predictors: tuple[tuple[str, Predictor], ...]) -> None:
        if not predictors:
            raise ValueError("at least one predictor is required")
        self.predictors = predictors

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        results: list[tuple[str, PredictionResult]] = []
        for name, predictor in self.predictors:
            try:
                results.append((name, predictor.predict(scene, action)))
            except Exception as exc:  # noqa: BLE001 - model failure is unknown evidence.
                return PredictionResult(
                    Decision.RISK_UNKNOWN,
                    f"predictor exception in {name}: {exc}",
                    tuple(),
                    None,
                    {"model": "conservative_fusion_v1", "risk_trigger": "predictor_exception", "failed_predictor": name},
                )

        if any(result.decision == Decision.REJECTED for _, result in results):
            decision = Decision.REJECTED
            reason = "at least one predictor rejected the action"
            trigger = "predictor_rejected"
        elif any(result.decision == Decision.RISK_UNKNOWN for _, result in results):
            decision = Decision.RISK_UNKNOWN
            reason = "predictor evidence is incomplete or uncertain"
            trigger = "predictor_unknown"
        else:
            decision = Decision.APPROVED
            reason = "all predictors approved the action"
            trigger = None

        primary = results[0][1]
        clearances = [result.min_clearance for _, result in results if result.min_clearance is not None]
        trace: dict[str, Any] = {
            "model": "conservative_fusion_v1",
            "risk_trigger": trigger,
            "predictors": [
                {
                    "name": name,
                    "decision": result.decision.value,
                    "reason": result.reason,
                    "risk_probability": result.model_trace.get("risk_probability"),
                }
                for name, result in results
            ],
        }
        return PredictionResult(decision, reason, primary.trajectory, min(clearances) if clearances else None, trace)
