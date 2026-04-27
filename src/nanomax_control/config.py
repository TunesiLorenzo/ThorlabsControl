from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class SimulationConfig:
    enabled: bool = False


@dataclass(slots=True)
class MST602Config:
    serial: str
    is_rack_system: bool = True
    scale: str = "step"
    channels: dict[str, int] | None = None

    def axis_channel(self, axis: str) -> int:
        if not self.channels or axis not in self.channels:
            raise KeyError(f"Axis '{axis}' is not configured in mst602.channels")
        return int(self.channels[axis])


@dataclass(slots=True)
class NanoTrakConfig:
    enabled: bool = True
    serial: str = ""
    poll_interval_ms: int = 200
    detector_threshold: float = 0.2
    tracking_mode: str = "track"
    kinesis_dll_dir: str | None = None
    assembly_candidates: list[str] | None = None


@dataclass(slots=True)
class AppConfig:
    simulation: SimulationConfig
    mst602: MST602Config
    nanotrak: NanoTrakConfig


def _require(mapping: dict[str, Any], key: str) -> Any:
    if key not in mapping:
        raise KeyError(f"Missing required config key: {key}")
    return mapping[key]


def load_config(path: str | Path) -> AppConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    sim = SimulationConfig(**(data.get("simulation") or {}))

    mst_raw = _require(data, "mst602")
    mst = MST602Config(
        serial=str(_require(mst_raw, "serial")),
        is_rack_system=bool(mst_raw.get("is_rack_system", True)),
        scale=str(mst_raw.get("scale", "step")),
        channels=dict(mst_raw.get("channels") or {"x": 1, "y": 2, "z": 1}),
    )

    nano_raw = data.get("nanotrak") or {}
    nano = NanoTrakConfig(
        enabled=bool(nano_raw.get("enabled", True)),
        serial=str(nano_raw.get("serial", "")),
        poll_interval_ms=int(nano_raw.get("poll_interval_ms", 200)),
        detector_threshold=float(nano_raw.get("detector_threshold", 0.2)),
        tracking_mode=str(nano_raw.get("tracking_mode", "track")),
        kinesis_dll_dir=nano_raw.get("kinesis_dll_dir"),
        assembly_candidates=list(nano_raw.get("assembly_candidates") or []),
    )

    return AppConfig(simulation=sim, mst602=mst, nanotrak=nano)
