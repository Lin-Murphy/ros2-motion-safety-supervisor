"""Mock backend used by deterministic tests and examples."""

from __future__ import annotations

from dataclasses import dataclass

from ..actions import TypedAction


@dataclass
class MockBackend:
    executed: list[str]

    def send(self, action: TypedAction) -> None:
        self.executed.append(action.action_type)
