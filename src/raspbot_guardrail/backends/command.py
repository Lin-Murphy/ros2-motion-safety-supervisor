"""Command backend adapters independent of the supervisor core."""

from __future__ import annotations

from dataclasses import dataclass, field

from .ros2_cmd_vel import DEFAULT_CMD_VEL_TOPIC, DryRunCmdVelPublisher, TwistCommand, zero_twist


@dataclass
class DryRunCommandBackend:
    topic: str = DEFAULT_CMD_VEL_TOPIC
    publisher: DryRunCmdVelPublisher | None = None
    name: str = "dry_run_cmd_vel"
    available_state: bool = True
    published_commands: list[TwistCommand] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.publisher is None:
            self.publisher = DryRunCmdVelPublisher(topic=self.topic)

    def publish(self, command: TwistCommand) -> None:
        if not self.available_state:
            raise RuntimeError("command backend unavailable")
        assert self.publisher is not None
        self.publisher.publish(command)
        self.published_commands.append(command)

    def hold(self, duration_s: float = 0.2) -> None:
        self.publish(zero_twist(duration_s=duration_s, topic=self.topic))

    def available(self) -> bool:
        return self.available_state
