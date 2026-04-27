from __future__ import annotations

from dataclasses import dataclass

from .config import MST602Config


@dataclass(slots=True)
class AxisState:
    axis: str
    channel: int
    position: float


class MST602Stage:
    """Coarse control wrapper for MST602 / DRV208 via pylablib."""

    def __init__(self, config: MST602Config):
        self.config = config
        self._dev = None

    def open(self) -> None:
        from pylablib.devices import Thorlabs

        self._dev = Thorlabs.KinesisMotor(
            self.config.serial,
            scale=self.config.scale,
            is_rack_system=self.config.is_rack_system,
        )

    def close(self) -> None:
        if self._dev is not None:
            self._dev.close()
            self._dev = None

    @staticmethod
    def list_devices():
        from pylablib.devices import Thorlabs

        return Thorlabs.list_kinesis_devices()

    def require_open(self) -> None:
        if self._dev is None:
            raise RuntimeError("MST602Stage is not open")

    def get_position(self, axis: str) -> float:
        self.require_open()
        channel = self.config.axis_channel(axis)
        return float(self._dev.get_position(channel=channel))

    def move_by(self, axis: str, steps: int) -> float:
        self.require_open()
        channel = self.config.axis_channel(axis)
        self._dev.move_by(int(steps), channel=channel)
        self._dev.wait_move(channel=channel)
        return float(self._dev.get_position(channel=channel))

    def move_to(self, axis: str, position: float) -> float:
        self.require_open()
        channel = self.config.axis_channel(axis)
        self._dev.move_to(position, channel=channel)
        self._dev.wait_move(channel=channel)
        return float(self._dev.get_position(channel=channel))

    def stop(self, axis: str) -> None:
        self.require_open()
        channel = self.config.axis_channel(axis)
        self._dev.stop(channel=channel)

    def snapshot(self) -> list[AxisState]:
        states: list[AxisState] = []
        for axis, channel in (self.config.channels or {}).items():
            pos = self._dev.get_position(channel=channel) if self._dev is not None else float("nan")
            states.append(AxisState(axis=axis, channel=channel, position=float(pos)))
        return states
