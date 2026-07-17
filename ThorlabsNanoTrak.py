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

    # ---------------------------------------------------------------- modes
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

    # ---------------------------------------------------------- feedback in
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

    # ------------------------------------------------- scan circle / device
    DEVICE_MAX = 65535
    CIRCLE_DIAMETER_MAX_NT = 10.0
    TRACK_RADIUS_MAX_NT = CIRCLE_DIAMETER_MAX_NT / 2.0
    CIRCLE_MODE_FIXED = 0x01  # NT_ParameterCircleMode: use the fixed diameter, not an auto-adjusting algorithm.
    CIRCLE_SAMPLE_RATE_HZ = 7000.0
    TRACK_FREQUENCY_MIN_HZ = 17.5
    TRACK_FREQUENCY_MAX_HZ = 87.5
    MIN_FREQUENCY_SAMPLES_PER_REVOLUTION = 400
    MAX_FREQUENCY_SAMPLES_PER_REVOLUTION = 80

    # Kinesis function signatures: (name, restype, argtypes). Declaring them
    # as data instead of 70-odd repetitive `dll.X.restype = ...` lines makes
    # it obvious at a glance which calls exist and keeps _load_dlls short.
    _FUNCTIONS = [
        ("TLI_BuildDeviceList", c_short, []),
        ("MMR_Open", c_short, [c_char_p]),
        ("MMR_Close", None, [c_char_p]),
        ("NT_Open", c_short, [c_char_p]),
        ("NT_Close", None, [c_char_p]),
        ("NT_StartPolling", c_bool, [c_char_p, c_int]),
        ("NT_StopPolling", None, [c_char_p]),
        ("NT_RequestCirclePosition", c_short, [c_char_p]),
        ("NT_GetCirclePosition", c_short, [c_char_p, POINTER(NT_HVComponent)]),
        ("NT_SetCircleHomePosition", c_short, [c_char_p, POINTER(NT_HVComponent)]),
        ("NT_HomeCircle", c_short, [c_char_p]),
        ("NT_RequestCircleParams", c_short, [c_char_p]),
        ("NT_GetCircleParams", c_short, [c_char_p, POINTER(NT_CircleParameters)]),
        ("NT_SetCircleParams", c_short, [c_char_p, POINTER(NT_CircleParameters)]),
        ("NT_RequestMode", c_short, [c_char_p]),
        ("NT_GetMode", c_ushort, [c_char_p]),
        ("NT_SetMode", c_short, [c_char_p, c_ushort]),
        ("NT_GetSignalState", c_int, [c_char_p]),
        ("NT_GetReading", c_short, [c_char_p, POINTER(NT_TIAReading)]),
        ("NT_ChannelEnable", c_short, [c_char_p, c_ushort, c_bool]),
        ("NT_RequestFeedbackSource", c_short, [c_char_p]),
        ("NT_GetFeedbackSource", c_ushort, [c_char_p]),
        ("NT_SetFeedbackSource", c_short, [c_char_p, c_ushort]),
        ("NT_RequestNTChannels", c_short, [c_char_p]),
        ("NT_GetNTChannels", c_short, [c_char_p, POINTER(c_short), POINTER(c_short)]),
        ("NT_SetNTChannels", c_short, [c_char_p, c_short, c_short]),
        ("NT_RequestPhaseCompensationParams", c_short, [c_char_p]),
        ("NT_GetPhaseCompensationParams", c_short, [c_char_p, POINTER(NT_HVComponent)]),
        ("NT_SetPhaseCompensationParams", c_short, [c_char_p, POINTER(NT_HVComponent)]),
    ]

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
        for name, restype, argtypes in self._FUNCTIONS:
            func = getattr(dll, name)
            func.restype = restype
            func.argtypes = argtypes
        self.dll = dll

    # ---------------------------------------------------- connection lifecycle

    def connect(self):
        ensure_device_list(self.dll)
        check_zero(self.dll.MMR_Open(self.serial), "MMR_Open")
        self._rack_opened = True
        check_zero(self.dll.NT_Open(self.serial), "NT_Open")
        self._opened = True
        if not self.dll.NT_StartPolling(self.serial, self.poll_ms):
            raise ThorlabsError("NT_StartPolling failed")
        self._polling = True
        time.sleep(self._settle_delay())
        # Enable whichever piezo channels the device is actually configured
        # to drive (NT_SetNTChannels/chanA,chanB) -- not a hardcoded 1/2,
        # which would silently leave the real output channels disabled if
        # the device is wired/configured to use different channel numbers.
        # A channel number of 0 means "unused" (e.g. a single-axis setup);
        # NT_ChannelEnable rejects it with FT_InvalidParameter, so skip it.
        for channel in self.get_nt_channels():
            if channel == 0:
                continue
            check_zero(
                self.dll.NT_ChannelEnable(self.serial, channel, True),
                f"NT_ChannelEnable(channel={channel})",
            )
        # Default to Latch so the outputs hold still and tracking does not
        # start dithering as soon as the device connects.
        self.set_mode(self.MODE_LATCH)
        self._ensure_fixed_circle_mode()
        # NT_RequestSettings is deliberately not called here: on this
        # modular-rack-hosted NanoTrak card (model MNA601) it always fails
        # with FT_InvalidHandle even though every other call -- including
        # NT_GetHardwareInfo, NT_ChannelEnable, and live position/mode
        # requests -- works fine, both before and after polling starts.
        # It isn't needed anyway: each getter below requests exactly the
        # data it needs.

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

    # ------------------------------------------------------------ unit helpers

    @classmethod
    def _to_percent(cls, value):
        return 100.0 * int(value) / cls.DEVICE_MAX

    @classmethod
    def _to_device(cls, percent):
        percent = max(0.0, min(100.0, float(percent)))
        return int(round(percent * cls.DEVICE_MAX / 100.0))

    def _settle_delay(self):
        return max(0.1, self.poll_ms / 1000.0)

    def _request(self, dll_func, name):
        """Ask the device to refresh one setting, then give it time to reply.

        Several NanoTrak settings (circle params, feedback source, NT
        channels, phase compensation) are NOT kept fresh by NT_StartPolling
        the way position/mode are -- their Get calls read a device-side
        cache that only updates after the matching Request call. Skipping
        this returns stale/uninitialised data.
        """
        check_zero(dll_func(self.serial), name)
        time.sleep(self._settle_delay())

    # --------------------------------------------------------------- position

    def get_position_percent(self):
        # Polling keeps the DLL cache current, so no NT_RequestCirclePosition
        # is needed here; requesting before every getter would serialize GUI
        # commands behind extra device round trips.
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
        self._ensure_fixed_circle_mode()
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

    # ------------------------------------------------------------------- mode

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

    # --------------------------------------------------------------- signal

    def get_signal(self):
        reading = NT_TIAReading()
        check_zero(self.dll.NT_GetReading(self.serial, byref(reading)), "NT_GetReading")
        return bool(self.dll.NT_GetSignalState(self.serial)), float(reading.absoluteReading)

    def get_feedback_source(self):
        self._request(self.dll.NT_RequestFeedbackSource, "NT_RequestFeedbackSource")
        return int(self.dll.NT_GetFeedbackSource(self.serial))

    def set_feedback_source(self, source):
        source = int(source)
        if source not in self.FEEDBACK_NAMES:
            raise ValueError(f"Unsupported NanoTrak feedback source: {source}")
        check_zero(
            self.dll.NT_SetFeedbackSource(self.serial, source), "NT_SetFeedbackSource"
        )
        return source

    # ----------------------------------------------------------- scan circle

    def _get_circle_params(self):
        self._request(self.dll.NT_RequestCircleParams, "NT_RequestCircleParams")
        params = NT_CircleParameters()
        check_zero(
            self.dll.NT_GetCircleParams(self.serial, byref(params)),
            "NT_GetCircleParams",
        )
        return params

    def _ensure_fixed_circle_mode(self):
        # The absolute-power/LUT circle algorithms continuously recentre the
        # scan circle from the live signal, silently overriding any home
        # position or radius set elsewhere. Fixed mode is required for those
        # to stick, so every call to set_position_percent forces it here.
        params = self._get_circle_params()
        if params.mode == self.CIRCLE_MODE_FIXED:
            return
        params.mode = self.CIRCLE_MODE_FIXED
        check_zero(
            self.dll.NT_SetCircleParams(self.serial, byref(params)),
            "NT_SetCircleParams",
        )
        # Confirm rather than trust: the settle delay in _get_circle_params
        # is a best-effort guess at the device's response time, and reading
        # back too early would silently report the pre-change (non-Fixed)
        # mode as if nothing were wrong -- exactly the kind of race that
        # would make a manual position apply "sometimes" work.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._get_circle_params().mode == self.CIRCLE_MODE_FIXED:
                return
        raise ThorlabsError("NanoTrak did not confirm Fixed circle mode within 2s.")

    def get_track_radius_nt(self):
        """Return the scan-circle radius in NanoTrak units (0 to 5)."""
        params = self._get_circle_params()
        diameter_nt = params.diameter * self.CIRCLE_DIAMETER_MAX_NT / self.DEVICE_MAX
        return diameter_nt / 2.0

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
        params = self._get_circle_params()
        params.mode = self.CIRCLE_MODE_FIXED
        params.diameter = diameter_device
        check_zero(
            self.dll.NT_SetCircleParams(self.serial, byref(params)),
            "NT_SetCircleParams",
        )
        return radius

    def get_track_frequency_samples(self):
        params = self._get_circle_params()
        samples = int(params.samplesPerRevolution)
        if samples <= 0:
            raise ThorlabsError(
                f"NanoTrak returned an invalid samples-per-revolution value: {samples}."
            )
        return samples

    def get_track_frequency_hz(self):
        return self.CIRCLE_SAMPLE_RATE_HZ / self.get_track_frequency_samples()

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
        params = self._get_circle_params()
        params.samplesPerRevolution = samples
        check_zero(
            self.dll.NT_SetCircleParams(self.serial, byref(params)),
            "NT_SetCircleParams",
        )
        return self.CIRCLE_SAMPLE_RATE_HZ / samples

    # -------------------------------------------------------- NT channel map

    def get_nt_channels(self):
        """Return (chanA, chanB): the piezo output channels used for H/V."""
        self._request(self.dll.NT_RequestNTChannels, "NT_RequestNTChannels")
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

    # ---------------------------------------------------- phase compensation
    #
    # Per the Kinesis NanoTrak header (NT_SetPhaseCompensationParams):
    #   raw value (0-65535) = phase(degrees) * samplesPerRevolution / 360
    # i.e. the raw counts are a sample-offset within the scan circle, so
    # they are only meaningful relative to the *current* track frequency
    # (which sets samplesPerRevolution). Converting through degrees here
    # keeps a requested phase (e.g. Kinesis-calibrated -31.5 degrees)
    # correct even if the track frequency changes later.

    @staticmethod
    def _phase_degrees_to_raw(degrees, samples):
        degrees_mod = float(degrees) % 360.0
        return int(round(degrees_mod * samples / 360.0)) % (ThorlabsModularNanoTrak.DEVICE_MAX + 1)

    @staticmethod
    def _phase_raw_to_degrees(raw, samples):
        degrees = (int(raw) * 360.0 / samples) % 360.0
        if degrees > 180.0:
            degrees -= 360.0
        return degrees

    def get_phase_compensation(self):
        """Return (horizontal, vertical) phase compensation in degrees (-180 to 180)."""
        self._request(
            self.dll.NT_RequestPhaseCompensationParams,
            "NT_RequestPhaseCompensationParams",
        )
        value = NT_HVComponent()
        check_zero(
            self.dll.NT_GetPhaseCompensationParams(self.serial, byref(value)),
            "NT_GetPhaseCompensationParams",
        )
        samples = self.get_track_frequency_samples()
        return (
            self._phase_raw_to_degrees(value.horizontalComponent, samples),
            self._phase_raw_to_degrees(value.verticalComponent, samples),
        )

    def set_phase_compensation(self, horizontal, vertical):
        """Set (horizontal, vertical) phase compensation in degrees."""
        samples = self.get_track_frequency_samples()
        h_raw = self._phase_degrees_to_raw(horizontal, samples)
        v_raw = self._phase_degrees_to_raw(vertical, samples)
        value = NT_HVComponent(h_raw, v_raw)
        check_zero(
            self.dll.NT_SetPhaseCompensationParams(self.serial, byref(value)),
            "NT_SetPhaseCompensationParams",
        )
        return (
            self._phase_raw_to_degrees(h_raw, samples),
            self._phase_raw_to_degrees(v_raw, samples),
        )
