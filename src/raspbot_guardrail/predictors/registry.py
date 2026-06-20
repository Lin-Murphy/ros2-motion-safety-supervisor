"""Model registry for guardrail predictors."""

from __future__ import annotations

from collections.abc import Callable

from ..predictor import KinematicRiskPredictor
from .base import Predictor


PredictorFactory = Callable[[], Predictor]

_REGISTRY: dict[str, PredictorFactory] = {
    "kinematic": KinematicRiskPredictor,
}


def available_predictors() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def build_predictor(name: str) -> Predictor:
    try:
        return _REGISTRY[name]()
    except KeyError as exc:
        options = ", ".join(available_predictors())
        raise ValueError(f"unknown predictor {name!r}; available predictors: {options}") from exc


def register_predictor(name: str, factory: PredictorFactory) -> None:
    if not name:
        raise ValueError("predictor name must be non-empty")
    if name in _REGISTRY:
        raise ValueError(f"predictor {name!r} is already registered")
    _REGISTRY[name] = factory

