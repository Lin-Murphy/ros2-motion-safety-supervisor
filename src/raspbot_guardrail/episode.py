"""Episode data model and JSON I/O."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ReplayEvent:
    index: int
    action_type: str
    static_decision: str
    predictive_decision: str
    final_decision: str
    reason: str
    trajectory: list[dict[str, float]]
    min_clearance: float | None


@dataclass(frozen=True)
class Episode:
    name: str
    events: list[ReplayEvent]
    metadata: dict[str, Any]


def write_episode(path: Path, episode: Episode) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "name": episode.name,
        "metadata": episode.metadata,
        "events": [asdict(event) for event in episode.events],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))
