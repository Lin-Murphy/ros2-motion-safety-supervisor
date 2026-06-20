"""Shared predictor interface."""

from __future__ import annotations

from typing import Protocol

from ..actions import TypedAction
from ..predictor import PredictionResult, Scene


class Predictor(Protocol):
    """Common interface for kinematic, learned, and world-model predictors."""

    def predict(self, scene: Scene, action: TypedAction) -> PredictionResult:
        """Predict action consequences and return a guardrail decision."""

