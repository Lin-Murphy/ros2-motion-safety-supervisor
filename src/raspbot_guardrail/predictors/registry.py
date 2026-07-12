"""Model registry for guardrail predictors."""

from __future__ import annotations

from collections.abc import Callable

from ..predictor import KinematicRiskPredictor
from .base import Predictor


PredictorFactory = Callable[[], Predictor]


def _build_learned_risk() -> Predictor:
    from ..learned_risk import build_default_learned_predictor

    return build_default_learned_predictor()


def _build_fusion() -> Predictor:
    from ..fusion import ConservativeFusionPredictor

    return ConservativeFusionPredictor((
        ("kinematic", KinematicRiskPredictor()),
        ("learned_risk", _build_learned_risk()),
    ))

_REGISTRY: dict[str, PredictorFactory] = {
    "kinematic": KinematicRiskPredictor,
    "learned_risk": lambda: _build_learned_risk(),
    "fusion": lambda: _build_fusion(),
}


def available_predictors() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def build_predictor(name: str) -> Predictor:
    if name == "learned_risk" and name not in _REGISTRY:
        _REGISTRY[name] = _build_learned_risk
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
