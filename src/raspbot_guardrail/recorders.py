"""Event recorder adapters."""

from __future__ import annotations

import json
from dataclasses import asdict, field
from pathlib import Path
from dataclasses import dataclass

from .episode import ReplayEvent
from .ports import EventRecorder


@dataclass
class InMemoryEventRecorder(EventRecorder):
    recorded_events: list[ReplayEvent] = field(default_factory=list)

    def record(self, event: ReplayEvent) -> None:
        self.recorded_events.append(event)


@dataclass
class JsonEventRecorder(EventRecorder):
    path: Path
    recorded_events: list[ReplayEvent] = field(default_factory=list)

    def record(self, event: ReplayEvent) -> None:
        self.recorded_events.append(event)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(
            json.dumps([asdict(item) for item in self.recorded_events], indent=2),
            encoding="utf-8",
        )
