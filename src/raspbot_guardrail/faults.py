"""Structured fault values used by the motion safety boundary."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True)
class Fault:
    code: str
    component: str
    detail: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
