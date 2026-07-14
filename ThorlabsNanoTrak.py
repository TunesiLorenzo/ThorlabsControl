"""Minimal Kinesis wrapper for a NanoTrak module in a modular rack."""

import os
import time
from ctypes import (
    POINTER,
    Structure,
    WinDLL,
    byref,
    c_bool,
    c_char_p,
    c_float,
    c_int,
    c_short,
    c_ushort,
)

from ThorlabsStepper import KINESIS_DIR, ThorlabsError, check_zero


class NT_HVComponent(Structure):
    _fields_ = [
        ("horizontalComponent", c_ushort),
        ("verticalComponent", c_ushort),
    ]


class NT_TIAReading(Structure):
    _fields_ = [
        ("absoluteReading", c_float),
        ("relativeReading", c_ushort),
        ("selectedRange", c_ushort),
        ("underOrOverRead", c_ushort),
    ]


class ThorlabsModularNanoTrak:
    """Control the NanoTrak card and its two piezo outputs."""

    MODE_PIEZO = 0x01
    MODE_LATCH = 0x02
    MODE_TRACKING = 0x03
    MODE_HORIZONTAL_TRACKING = 0x04
    MODE_VERTICAL_TRACKING = 0x05

    MODE_NAMES = {
        MODE_PIEZO: "Piezo",
        MODE_LATCH: "Latch",
        MODE_TRACKING: "Tracking",
        MODE_HORIZONTAL_TRACKING: "Horizontal tracking",
        MODE_VERTICAL_TRACKING: "Vertical tracking",
    }

    DEVICE_MAX = 65535

    def __init__(self, serial: str, kinesis_dir: str = KINESIS_DIR, poll_ms: int = 100):
        self.serial = str(serial).encode("ascii")
        self.kinesis_dir = self._resolve_kinesis_dir(kinesis_dir)
        self.poll_ms = int(poll_ms)
        self.dll = None
        self._opened = False
        self._polling = False
        self._load_dlls()

    @staticmethod
    def _resolve_kinesis_dir(configured_dir):
        if os.path.isdir(configured_dir):
            return configured_dir
        bundled = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dll")
        if os.path.isdir(bundled):
            return bundled
        return configured_dir

    def _load_dlls(self):
        os.add_dll_directory(self.kinesis_dir)
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.DeviceManager.dll"))
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.Benchtop.NanoTrak.dll"))
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.Benchtop.Piezo.dll"))
        dll = WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.ModularRack.dll"))

        dll.TLI_BuildDeviceList.restype = c_short
        dll.TLI_BuildDeviceList.argtypes = []
        dll.NT_Open.restype = c_short
        dll.NT_Open.argtypes = [c_char_p]
        dll.NT_Close.restype = None
        dll.NT_Close.argtypes = [c_char_p]
        dll.NT_StartPolling.restype = c_bool
        dll.NT_StartPolling.argtypes = [c_char_p, c_int]
        dll.NT_StopPolling.restype = None
        dll.NT_StopPolling.argtypes = [c_char_p]
        dll.NT_RequestSettings.restype = c_short
        dll.NT_RequestSettings.argtypes = [c_char_p]
        dll.NT_RequestCirclePosition.restype = c_short
        dll.NT_RequestCirclePosition.argtypes = [c_char_p]
        dll.NT_GetCirclePosition.restype = c_short
        dll.NT_GetCirclePosition.argtypes = [c_char_p, POINTER(NT_HVComponent)]
        dll.NT_SetCircleHomePosition.restype = c_short
        dll.NT_SetCircleHomePosition.argtypes = [c_char_p, POINTER(NT_HVComponent)]
        dll.NT_HomeCircle.restype = c_short
        dll.NT_HomeCircle.argtypes = [c_char_p]
        dll.NT_RequestMode.restype = c_short
        dll.NT_RequestMode.argtypes = [c_char_p]
        dll.NT_GetMode.restype = c_ushort
        dll.NT_GetMode.argtypes = [c_char_p]
        dll.NT_SetMode.restype = c_short
        dll.NT_SetMode.argtypes = [c_char_p, c_ushort]
        dll.NT_RequestSignalState.restype = c_short
        dll.NT_RequestSignalState.argtypes = [c_char_p]
        dll.NT_GetSignalState.restype = c_int
        dll.NT_GetSignalState.argtypes = [c_char_p]
        dll.NT_RequestReading.restype = c_short
        dll.NT_RequestReading.argtypes = [c_char_p]
        dll.NT_GetReading.restype = c_short
        dll.NT_GetReading.argtypes = [c_char_p, POINTER(NT_TIAReading)]
        dll.NT_ChannelEnable.restype = c_short
        dll.NT_ChannelEnable.argtypes = [c_char_p, c_ushort, c_bool]
        self.dll = dll

    def connect(self):
        check_zero(self.dll.TLI_BuildDeviceList(), "TLI_BuildDeviceList")
        check_zero(self.dll.NT_Open(self.serial), "NT_Open")
        self._opened = True
        for channel in (1, 2):
            check_zero(
                self.dll.NT_ChannelEnable(self.serial, channel, True),
                f"NT_ChannelEnable(channel={channel})",
            )
        check_zero(self.dll.NT_RequestSettings(self.serial), "NT_RequestSettings")
        if not self.dll.NT_StartPolling(self.serial, self.poll_ms):
            raise ThorlabsError("NT_StartPolling failed")
        self._polling = True
        time.sleep(max(0.1, self.poll_ms / 1000.0))

    def disconnect(self):
        if self._polling:
            self.dll.NT_StopPolling(self.serial)
            self._polling = False
        if self._opened:
            self.dll.NT_Close(self.serial)
            self._opened = False

    def safe_shutdown(self):
        try:
            self.disconnect()
        except Exception:
            pass

    @classmethod
    def _to_percent(cls, value):
        return 100.0 * int(value) / cls.DEVICE_MAX

    @classmethod
    def _to_device(cls, percent):
        percent = max(0.0, min(100.0, float(percent)))
        return int(round(percent * cls.DEVICE_MAX / 100.0))

    def get_position_percent(self):
        check_zero(self.dll.NT_RequestCirclePosition(self.serial), "NT_RequestCirclePosition")
        time.sleep(max(0.02, self.poll_ms / 1000.0))
        value = NT_HVComponent()
        check_zero(
            self.dll.NT_GetCirclePosition(self.serial, byref(value)),
            "NT_GetCirclePosition",
        )
        return self._to_percent(value.horizontalComponent), self._to_percent(
            value.verticalComponent
        )

    def set_position_percent(self, horizontal, vertical):
        self.set_mode(self.MODE_PIEZO)
        value = NT_HVComponent(
            self._to_device(horizontal), self._to_device(vertical)
        )
        check_zero(
            self.dll.NT_SetCircleHomePosition(self.serial, byref(value)),
            "NT_SetCircleHomePosition",
        )
        check_zero(self.dll.NT_HomeCircle(self.serial), "NT_HomeCircle")

    def get_mode(self):
        check_zero(self.dll.NT_RequestMode(self.serial), "NT_RequestMode")
        time.sleep(max(0.02, self.poll_ms / 1000.0))
        return int(self.dll.NT_GetMode(self.serial))

    def set_mode(self, mode):
        mode = int(mode)
        if mode not in self.MODE_NAMES:
            raise ValueError(f"Unsupported NanoTrak mode: {mode}")
        check_zero(self.dll.NT_SetMode(self.serial, mode), "NT_SetMode")

    def get_signal(self):
        check_zero(self.dll.NT_RequestSignalState(self.serial), "NT_RequestSignalState")
        check_zero(self.dll.NT_RequestReading(self.serial), "NT_RequestReading")
        time.sleep(max(0.02, self.poll_ms / 1000.0))
        reading = NT_TIAReading()
        check_zero(self.dll.NT_GetReading(self.serial, byref(reading)), "NT_GetReading")
        return bool(self.dll.NT_GetSignalState(self.serial)), float(reading.absoluteReading)
