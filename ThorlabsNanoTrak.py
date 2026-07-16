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

from ThorlabsStepper import KINESIS_DIR, ThorlabsError, check_zero, ensure_device_list


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


class NT_CircleParameters(Structure):
    _pack_ = 1
    _fields_ = [
        ("mode", c_ushort),
        ("diameter", c_ushort),
        ("samplesPerRevolution", c_ushort),
        ("minDiameter", c_ushort),
        ("maxDiameter", c_ushort),
        ("algorithmAdjustment", c_ushort),
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

    FEEDBACK_TIA = 0x01
    FEEDBACK_BNC_1V = 0x02
    FEEDBACK_BNC_2V = 0x03
    FEEDBACK_BNC_5V = 0x04
    FEEDBACK_BNC_10V = 0x05

    FEEDBACK_NAMES = {
        FEEDBACK_TIA: "PIN / Optical input (TIA)",
        FEEDBACK_BNC_1V: "External BNC (0-1V)",
        FEEDBACK_BNC_2V: "External BNC (0-2V)",
        FEEDBACK_BNC_5V: "External BNC (0-5V)",
        FEEDBACK_BNC_10V: "External BNC (0-10V)",
    }

    DEVICE_MAX = 65535
    CIRCLE_DIAMETER_MAX_NT = 10.0
    TRACK_RADIUS_MAX_NT = CIRCLE_DIAMETER_MAX_NT / 2.0
    CIRCLE_MODE_FIXED = 0x01
    CIRCLE_SAMPLE_RATE_HZ = 7000.0
    TRACK_FREQUENCY_MIN_HZ = 17.5
    TRACK_FREQUENCY_MAX_HZ = 87.5
    MIN_FREQUENCY_SAMPLES_PER_REVOLUTION = 400
    MAX_FREQUENCY_SAMPLES_PER_REVOLUTION = 80

    def __init__(self, serial: str, kinesis_dir: str = KINESIS_DIR, poll_ms: int = 100):
        self.serial = str(serial).encode("ascii")
        self.kinesis_dir = self._resolve_kinesis_dir(kinesis_dir)
        self.poll_ms = int(poll_ms)
        self.dll = None
        self._opened = False
        self._rack_opened = False
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
        dll.MMR_Open.restype = c_short
        dll.MMR_Open.argtypes = [c_char_p]
        dll.MMR_Close.restype = None
        dll.MMR_Close.argtypes = [c_char_p]
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
        dll.NT_GetCircleDiameter.restype = c_ushort
        dll.NT_GetCircleDiameter.argtypes = [c_char_p]
        dll.NT_SetCircleDiameter.restype = c_short
        dll.NT_SetCircleDiameter.argtypes = [c_char_p, c_ushort]
        dll.NT_GetCircleParams.restype = c_short
        dll.NT_GetCircleParams.argtypes = [c_char_p, POINTER(NT_CircleParameters)]
        dll.NT_SetCircleParams.restype = c_short
        dll.NT_SetCircleParams.argtypes = [c_char_p, POINTER(NT_CircleParameters)]
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
        dll.NT_RequestFeedbackSource.restype = c_short
        dll.NT_RequestFeedbackSource.argtypes = [c_char_p]
        dll.NT_GetFeedbackSource.restype = c_ushort
        dll.NT_GetFeedbackSource.argtypes = [c_char_p]
        dll.NT_SetFeedbackSource.restype = c_short
        dll.NT_SetFeedbackSource.argtypes = [c_char_p, c_ushort]
        dll.NT_RequestNTChannels.restype = c_short
        dll.NT_RequestNTChannels.argtypes = [c_char_p]
        dll.NT_GetNTChannels.restype = c_short
        dll.NT_GetNTChannels.argtypes = [c_char_p, POINTER(c_short), POINTER(c_short)]
        dll.NT_SetNTChannels.restype = c_short
        dll.NT_SetNTChannels.argtypes = [c_char_p, c_short, c_short]
        dll.NT_RequestPhaseCompensationParams.restype = c_short
        dll.NT_RequestPhaseCompensationParams.argtypes = [c_char_p]
        dll.NT_GetPhaseCompensationParams.restype = c_short
        dll.NT_GetPhaseCompensationParams.argtypes = [c_char_p, POINTER(NT_HVComponent)]
        dll.NT_SetPhaseCompensationParams.restype = c_short
        dll.NT_SetPhaseCompensationParams.argtypes = [c_char_p, POINTER(NT_HVComponent)]
        self.dll = dll

    def connect(self):
        ensure_device_list(self.dll)
        check_zero(self.dll.MMR_Open(self.serial), "MMR_Open")
        self._rack_opened = True
        check_zero(self.dll.NT_Open(self.serial), "NT_Open")
        self._opened = True
        for channel in (1, 2):
            check_zero(
                self.dll.NT_ChannelEnable(self.serial, channel, True),
                f"NT_ChannelEnable(channel={channel})",
            )
        if not self.dll.NT_StartPolling(self.serial, self.poll_ms):
            raise ThorlabsError("NT_StartPolling failed")
        self._polling = True
        # NT_RequestSettings is deliberately not called here: on this
        # modular-rack-hosted NanoTrak card (model MNA601) it always fails
        # with FT_InvalidHandle even though every other call -- including
        # NT_GetHardwareInfo, NT_ChannelEnable, and live position/mode
        # requests -- works fine, both before and after polling starts.
        # It isn't needed anyway: NT_RequestCirclePosition/NT_RequestMode
        # (used by get_position_percent/get_mode below) fetch live values
        # from the device directly without it.
        time.sleep(max(0.1, self.poll_ms / 1000.0))

    def disconnect(self):
        if self._polling:
            self.dll.NT_StopPolling(self.serial)
            self._polling = False
        if self._opened:
            self.dll.NT_Close(self.serial)
            self._opened = False
        if self._rack_opened:
            self.dll.MMR_Close(self.serial)
            self._rack_opened = False

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
        # Polling keeps the DLL cache current.  Sending a synchronous request
        # before every getter serializes GUI commands behind several device
        # round trips and can make mode/setting changes appear to hang.
        value = NT_HVComponent()
        check_zero(
            self.dll.NT_GetCirclePosition(self.serial, byref(value)),
            "NT_GetCirclePosition",
        )
        return self._to_percent(value.horizontalComponent), self._to_percent(
            value.verticalComponent
        )

    def set_position_percent(self, horizontal, vertical):
        horizontal = float(horizontal)
        vertical = float(vertical)
        # MODE_PIEZO changes the hardware personality to the separate Piezo
        # controller API and may require a reboot.  Manual positioning within
        # NanoTrak operation is instead done with tracking latched, followed
        # by moving the scan-circle centre.
        self.set_mode(self.MODE_LATCH)
        value = NT_HVComponent(
            self._to_device(horizontal), self._to_device(vertical)
        )
        check_zero(
            self.dll.NT_SetCircleHomePosition(self.serial, byref(value)),
            "NT_SetCircleHomePosition",
        )
        check_zero(self.dll.NT_HomeCircle(self.serial), "NT_HomeCircle")
        check_zero(
            self.dll.NT_RequestCirclePosition(self.serial),
            "NT_RequestCirclePosition",
        )
        self._wait_for_position(horizontal, vertical)

    def _wait_for_position(self, horizontal, vertical, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        tolerance_percent = 0.02
        while time.monotonic() < deadline:
            time.sleep(max(0.02, self.poll_ms / 1000.0))
            actual_h, actual_v = self.get_position_percent()
            if (
                abs(actual_h - horizontal) <= tolerance_percent
                and abs(actual_v - vertical) <= tolerance_percent
            ):
                return actual_h, actual_v
        raise ThorlabsError(
            "NanoTrak did not confirm the requested manual position "
            f"H={horizontal:g}%, V={vertical:g}% within {timeout_s:g}s."
        )

    def get_mode(self):
        return int(self.dll.NT_GetMode(self.serial))

    def set_mode(self, mode):
        mode = int(mode)
        if mode not in self.MODE_NAMES:
            raise ValueError(f"Unsupported NanoTrak mode: {mode}")
        check_zero(self.dll.NT_SetMode(self.serial, mode), "NT_SetMode")
        check_zero(self.dll.NT_RequestMode(self.serial), "NT_RequestMode")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            time.sleep(max(0.02, self.poll_ms / 1000.0))
            if self.get_mode() == mode:
                return
        raise ThorlabsError(
            f"NanoTrak did not confirm mode {self.MODE_NAMES[mode]} within 2s."
        )

    def get_signal(self):
        reading = NT_TIAReading()
        check_zero(self.dll.NT_GetReading(self.serial, byref(reading)), "NT_GetReading")
        return bool(self.dll.NT_GetSignalState(self.serial)), float(reading.absoluteReading)

    def get_track_radius_nt(self):
        """Return the scan-circle radius in NanoTrak units (0 to 5)."""
        diameter_device = int(self.dll.NT_GetCircleDiameter(self.serial))
        diameter_nt = (
            diameter_device * self.CIRCLE_DIAMETER_MAX_NT / self.DEVICE_MAX
        )
        return diameter_nt / 2.0

    def get_track_frequency_hz(self):
        params = NT_CircleParameters()
        check_zero(
            self.dll.NT_GetCircleParams(self.serial, byref(params)),
            "NT_GetCircleParams",
        )
        samples = int(params.samplesPerRevolution)
        if samples <= 0:
            raise ThorlabsError(
                f"NanoTrak returned an invalid samples-per-revolution value: {samples}."
            )
        return self.CIRCLE_SAMPLE_RATE_HZ / samples

    def set_track_frequency_hz(self, frequency_hz):
        """Set frequency, quantized to the device's four-sample resolution."""
        frequency_hz = float(frequency_hz)
        if not self.TRACK_FREQUENCY_MIN_HZ <= frequency_hz <= self.TRACK_FREQUENCY_MAX_HZ:
            raise ValueError(
                "NanoTrak frequency must be between "
                f"{self.TRACK_FREQUENCY_MIN_HZ:g} and "
                f"{self.TRACK_FREQUENCY_MAX_HZ:g} Hz."
            )
        samples = 4 * round(self.CIRCLE_SAMPLE_RATE_HZ / frequency_hz / 4)
        samples = max(
            self.MAX_FREQUENCY_SAMPLES_PER_REVOLUTION,
            min(self.MIN_FREQUENCY_SAMPLES_PER_REVOLUTION, samples),
        )
        params = NT_CircleParameters()
        check_zero(
            self.dll.NT_GetCircleParams(self.serial, byref(params)),
            "NT_GetCircleParams",
        )
        params.samplesPerRevolution = samples
        check_zero(
            self.dll.NT_SetCircleParams(self.serial, byref(params)),
            "NT_SetCircleParams",
        )
        return self.CIRCLE_SAMPLE_RATE_HZ / samples

    def set_track_radius_nt(self, radius):
        """Set the scan-circle radius in NanoTrak units (0 to 5)."""
        radius = float(radius)
        if not 0.0 <= radius <= self.TRACK_RADIUS_MAX_NT:
            raise ValueError(
                f"NanoTrak radius must be between 0 and {self.TRACK_RADIUS_MAX_NT:g} NT units."
            )
        diameter_nt = 2.0 * radius
        diameter_device = int(
            round(diameter_nt * self.DEVICE_MAX / self.CIRCLE_DIAMETER_MAX_NT)
        )
        params = NT_CircleParameters()
        check_zero(
            self.dll.NT_GetCircleParams(self.serial, byref(params)),
            "NT_GetCircleParams",
        )
        # Fixed mode prevents the absolute-power or LUT algorithms from
        # replacing the radius entered in the GUI.
        params.mode = self.CIRCLE_MODE_FIXED
        params.diameter = diameter_device
        check_zero(
            self.dll.NT_SetCircleParams(self.serial, byref(params)),
            "NT_SetCircleParams",
        )
        return radius

    def get_feedback_source(self):
        return int(self.dll.NT_GetFeedbackSource(self.serial))

    def set_feedback_source(self, source):
        source = int(source)
        if source not in self.FEEDBACK_NAMES:
            raise ValueError(f"Unsupported NanoTrak feedback source: {source}")
        check_zero(
            self.dll.NT_SetFeedbackSource(self.serial, source), "NT_SetFeedbackSource"
        )
        return source

    def refresh_settings_cache(self):
        """Settings getters read the cache maintained by active polling."""

    def get_nt_channels(self):
        chan_a = c_short()
        chan_b = c_short()
        check_zero(
            self.dll.NT_GetNTChannels(self.serial, byref(chan_a), byref(chan_b)),
            "NT_GetNTChannels",
        )
        return int(chan_a.value), int(chan_b.value)

    def set_nt_channels(self, chan_a, chan_b):
        chan_a = int(chan_a)
        chan_b = int(chan_b)
        check_zero(
            self.dll.NT_SetNTChannels(self.serial, chan_a, chan_b),
            "NT_SetNTChannels",
        )
        return chan_a, chan_b

    def get_phase_compensation(self):
        value = NT_HVComponent()
        check_zero(
            self.dll.NT_GetPhaseCompensationParams(self.serial, byref(value)),
            "NT_GetPhaseCompensationParams",
        )
        return int(value.horizontalComponent), int(value.verticalComponent)

    def set_phase_compensation(self, horizontal, vertical):
        horizontal = int(horizontal)
        vertical = int(vertical)
        value = NT_HVComponent(horizontal, vertical)
        check_zero(
            self.dll.NT_SetPhaseCompensationParams(self.serial, byref(value)),
            "NT_SetPhaseCompensationParams",
        )
        return horizontal, vertical
