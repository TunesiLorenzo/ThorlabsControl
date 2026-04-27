from __future__ import annotations

from dataclasses import dataclass

from .coarse_stage import AxisState


class SimMST602Stage:
    def __init__(self, channels: dict[str, int] | None = None):
        self.channels = channels or {"x": 1, "y": 2, "z": 3}
        self.positions = {axis: 0.0 for axis in self.channels}
        self.opened = False

    @staticmethod
    def list_devices():
        return [("SIM_MST602", "Simulated MST602")]

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    def get_position(self, axis: str) -> float:
        return float(self.positions[axis])

    def move_by(self, axis: str, steps: int) -> float:
        self.positions[axis] += int(steps)
        return float(self.positions[axis])

    def move_to(self, axis: str, position: float) -> float:
        self.positions[axis] = float(position)
        return float(self.positions[axis])

    def stop(self, axis: str) -> None:
        _ = axis

    def snapshot(self) -> list[AxisState]:
        return [
            AxisState(axis=axis, channel=channel, position=float(self.positions[axis]))
            for axis, channel in self.channels.items()
        ]


@dataclass(slots=True)
class SimNanoTrakState:
    connected: bool
    mode: str
    signal: float
    circle_x: float
    circle_y: float


class SimNanoTrakController:
    def __init__(self, threshold: float = 0.2):
        self.threshold = threshold
        self.connected = False
        self.mode = "idle"
        self.signal = 0.05
        self.circle_x = 0.0
        self.circle_y = 0.0

    def open(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        if mode != "idle":
            self.signal = max(self.signal, self.threshold + 0.1)

    def start_tracking(self) -> None:
        self.set_mode("track")

    def stop_tracking(self) -> None:
        self.set_mode("idle")

    def get_signal(self) -> float:
        return float(self.signal)

    def get_circle_position(self) -> tuple[float, float]:
        return self.circle_x, self.circle_y

    def state(self) -> SimNanoTrakState:
        return SimNanoTrakState(
            connected=self.connected,
            mode=self.mode,
            signal=float(self.signal),
            circle_x=float(self.circle_x),
            circle_y=float(self.circle_y),
        )
