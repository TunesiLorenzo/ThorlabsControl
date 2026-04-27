from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import NanoTrakConfig


@dataclass(slots=True)
class NanoTrakState:
    connected: bool
    mode: str
    signal: float | None
    circle_x: float | None
    circle_y: float | None


class NanoTrakController:
    """Wrapper for Thorlabs NanoTrak via pythonnet.

    This file is designed as a practical starter. Depending on your Kinesis
    installation, you may need to adjust assembly names or the final device
    creation call.
    """

    def __init__(self, config: NanoTrakConfig):
        self.config = config
        self._clr = None
        self._device_manager = None
        self._device = None
        self._mode = "idle"

    def _add_dll_dir(self) -> None:
        if self.config.kinesis_dll_dir:
            dll_dir = str(Path(self.config.kinesis_dll_dir))
            if dll_dir not in sys.path:
                sys.path.append(dll_dir)

    def _load_assemblies(self) -> None:
        self._add_dll_dir()
        import clr  # type: ignore

        self._clr = clr
        for assembly in self.config.assembly_candidates or []:
            try:
                clr.AddReference(assembly)
            except Exception:
                continue

    def open(self) -> None:
        self._load_assemblies()

        from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI  # type: ignore

        DeviceManagerCLI.BuildDeviceList()
        self._device_manager = DeviceManagerCLI

        # NOTE:
        # The exact class import below may need adapting for your installed
        # Kinesis package. We try a few common namespaces.
        factory_errors: list[str] = []
        candidates = [
            (
                "Thorlabs.MotionControl.GenericNanoTrakCLI",
                "GenericNanoTrak",
                "CreateNanoTrak",
            ),
            (
                "Thorlabs.MotionControl.Benchtop.NanoTrakCLI",
                "BenchtopNanoTrak",
                "CreateBenchtopNanoTrak",
            ),
            (
                "Thorlabs.MotionControl.ModularRack.NanoTrakCLI",
                "ModularRackNanoTrak",
                "CreateModularRackNanoTrak",
            ),
        ]

        serial = self.config.serial
        for module_name, class_name, factory_name in candidates:
            try:
                module = __import__(module_name, fromlist=[class_name])
                cls = getattr(module, class_name)
                factory = getattr(cls, factory_name)
                self._device = factory(serial)
                break
            except Exception as exc:
                factory_errors.append(f"{module_name}.{class_name}.{factory_name}: {exc}")

        if self._device is None:
            joined = "\n".join(factory_errors)
            raise RuntimeError(
                "Could not create NanoTrak device from installed Kinesis assemblies. "
                "Edit nanotrak.py to match your Kinesis version.\n"
                f"Tried:\n{joined}"
            )

        self._device.Connect(serial)
        time.sleep(0.25)

        try:
            self._device.StartPolling(int(self.config.poll_interval_ms))
        except Exception:
            pass

        try:
            self._device.EnableDevice()
        except Exception:
            pass

        self._mode = "idle"

    def close(self) -> None:
        if self._device is None:
            return
        try:
            self._device.StopPolling()
        except Exception:
            pass
        try:
            self._device.Disconnect(True)
        except Exception:
            pass
        self._device = None

    def require_open(self) -> Any:
        if self._device is None:
            raise RuntimeError("NanoTrakController is not open")
        return self._device

    def set_mode(self, mode: str) -> None:
        dev = self.require_open()
        mode_lower = mode.strip().lower()

        # Actual enum names differ between versions. This tries the common path.
        try:
            current_module = dev.GetType().Assembly
            enum_type = current_module.GetType("Thorlabs.MotionControl.GenericNanoTrakCLI.NanoTrakStatus+NanoTrakMode")
            if enum_type is not None:
                import System  # type: ignore
                enum_value = System.Enum.Parse(enum_type, mode_lower, True)
                dev.SetMode(enum_value)
            else:
                dev.SetMode(mode)
        except Exception:
            # Fallback for cases where a string or different enum is accepted.
            try:
                dev.SetMode(mode)
            except Exception as exc:
                raise RuntimeError(f"Failed to set NanoTrak mode '{mode}': {exc}") from exc

        self._mode = mode_lower

    def start_tracking(self) -> None:
        self.set_mode(self.config.tracking_mode)

    def stop_tracking(self) -> None:
        self.set_mode("idle")

    def get_signal(self) -> float | None:
        dev = self.require_open()
        for name in ["GetReading", "GetSignal", "GetDetectorReading"]:
            if hasattr(dev, name):
                try:
                    value = getattr(dev, name)()
                    return float(value)
                except Exception:
                    continue
        return None

    def get_circle_position(self) -> tuple[float | None, float | None]:
        dev = self.require_open()
        if hasattr(dev, "GetCirclePosition"):
            try:
                value = dev.GetCirclePosition()
                if hasattr(value, "X") and hasattr(value, "Y"):
                    return float(value.X), float(value.Y)
                if isinstance(value, tuple) and len(value) >= 2:
                    return float(value[0]), float(value[1])
            except Exception:
                pass
        return None, None

    def state(self) -> NanoTrakState:
        signal = self.get_signal()
        cx, cy = self.get_circle_position()
        return NanoTrakState(
            connected=self._device is not None,
            mode=self._mode,
            signal=signal,
            circle_x=cx,
            circle_y=cy,
        )
