from __future__ import annotations

from dataclasses import asdict
from typing import Any

from .config import AppConfig
from .coarse_stage import MST602Stage
from .nanotrak import NanoTrakController
from .sim import SimMST602Stage, SimNanoTrakController


class NanoMaxSystem:
    def __init__(self, config: AppConfig):
        self.config = config
        if config.simulation.enabled:
            self.coarse = SimMST602Stage(config.mst602.channels)
            self.nanotrak = SimNanoTrakController(config.nanotrak.detector_threshold) if config.nanotrak.enabled else None
        else:
            self.coarse = MST602Stage(config.mst602)
            self.nanotrak = NanoTrakController(config.nanotrak) if config.nanotrak.enabled else None

    def open(self) -> None:
        self.coarse.open()
        if self.nanotrak is not None:
            self.nanotrak.open()

    def close(self) -> None:
        try:
            if self.nanotrak is not None:
                self.nanotrak.close()
        finally:
            self.coarse.close()

    def list_devices(self):
        return self.coarse.list_devices()

    def coarse_move_by(self, axis: str, steps: int) -> float:
        return self.coarse.move_by(axis, steps)

    def coarse_move_to(self, axis: str, position: float) -> float:
        return self.coarse.move_to(axis, position)

    def start_tracking(self) -> None:
        if self.nanotrak is None:
            raise RuntimeError("NanoTrak is disabled in config")
        self.nanotrak.start_tracking()

    def stop_tracking(self) -> None:
        if self.nanotrak is None:
            raise RuntimeError("NanoTrak is disabled in config")
        self.nanotrak.stop_tracking()

    def show_state(self) -> dict[str, Any]:
        coarse = [asdict(item) for item in self.coarse.snapshot()]
        nano = asdict(self.nanotrak.state()) if self.nanotrak is not None else None
        return {"coarse": coarse, "nanotrak": nano}

    def recover_coupling(self, axis: str, sweep_steps: int, count: int) -> dict[str, Any]:
        if self.nanotrak is None:
            raise RuntimeError("NanoTrak is disabled in config")

        threshold = self.config.nanotrak.detector_threshold
        samples: list[dict[str, Any]] = []
        pattern = []
        for idx in range(1, count + 1):
            pattern.extend([idx * sweep_steps, -2 * idx * sweep_steps])
        pattern.append(count * sweep_steps)

        found = False
        for move in pattern:
            pos = self.coarse_move_by(axis, move)
            signal = self.nanotrak.get_signal()
            samples.append({"axis": axis, "move": move, "position": pos, "signal": signal})
            if signal is not None and signal >= threshold:
                self.nanotrak.start_tracking()
                found = True
                break

        return {"found": found, "threshold": threshold, "samples": samples}
