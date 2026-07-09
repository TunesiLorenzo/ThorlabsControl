import os
import time
from ctypes import (
    WinDLL,
    byref,
    POINTER,
    c_char_p,
    c_short,
    c_int,
    c_bool,
    c_uint32,
)
import matplotlib.pyplot as plt
from ArduinoSampler import close_arduino, open_arduino, burst_read_binary
import math

KINESIS_DIR = r"C:\Program Files\Thorlabs\Kinesis"


class ThorlabsError(RuntimeError):
    pass


def check_zero(result, name):
    if result != 0:
        raise ThorlabsError(f"{name} failed with error code {result}")


def decode_status(status: int):
    flags = []
    if status & 0x00000001:
        flags.append("CW hardware limit")
    if status & 0x00000002:
        flags.append("CCW hardware limit")
    if status & 0x00000004:
        flags.append("CW software limit")
    if status & 0x00000008:
        flags.append("CCW software limit")
    if status & 0x00000010:
        flags.append("moving CW")
    if status & 0x00000020:
        flags.append("moving CCW")
    if status & 0x00000040:
        flags.append("jogging CW")
    if status & 0x00000080:
        flags.append("jogging CCW")
    if status & 0x00000100:
        flags.append("motor connected")
    if status & 0x00000200:
        flags.append("homing")
    if status & 0x00000400:
        flags.append("homed")
    if status & 0x20000000:
        flags.append("active")
    if status & 0x80000000:
        flags.append("channel enabled")
    return flags


class ThorlabsModularStepperController:

    """
    Wrapper for a Thorlabs Modular Rack stepper motor channel.

    Notes:
    - Positions, velocities, accelerations, jog steps are in DEVICE UNITS.
    - Channel should normally be 1 or 2.
    - This class keeps the DLL loading style from your earlier script.
    """

    # Travel directions from the header
    DIRECTION_FORWARD = 0x01
    DIRECTION_BACKWARD = 0x02

    # Jog modes from header
    JOG_CONTINUOUS = 0x01
    JOG_SINGLE_STEP = 0x02

    # Stop modes from header
    STOP_IMMEDIATE = 0x01
    STOP_PROFILED = 0x02

    def __init__(
        self,
        serial: str,
        channel: int,
        kinesis_dir: str = KINESIS_DIR,
        poll_ms: int = 400,
    ):
        self.serial_str = str(serial)
        self.serial = self.serial_str.encode("ascii")
        self.channel = int(channel)
        self.kinesis_dir = kinesis_dir
        self.poll_ms = int(poll_ms)

        if self.channel not in (1, 2):
            raise ValueError("channel must be 1 or 2")

        self.dll = None
        self._opened = False
        self._enabled = False
        self._polling = False
        self._actual_poll_ms = None

        self._load_dlls()

    # ------------------------------------------------------------------
    # DLL bootstrap
    # ------------------------------------------------------------------
    def _load_dlls(self):
        os.add_dll_directory(self.kinesis_dir)

        # Keep same style as your previous script
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.DeviceManager.dll"))
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.Benchtop.StepperMotor.dll"))
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.Benchtop.NanoTrak.dll"))
        WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.Benchtop.Piezo.dll"))
        dll = WinDLL(os.path.join(self.kinesis_dir, "Thorlabs.MotionControl.ModularRack.dll"))

        # Rack functions
        dll.TLI_BuildDeviceList.restype = c_short
        dll.TLI_BuildDeviceList.argtypes = []

        dll.MMR_Open.restype = c_short
        dll.MMR_Open.argtypes = [c_char_p]

        dll.MMR_Close.restype = None
        dll.MMR_Close.argtypes = [c_char_p]

        dll.MMR_IsChannelValid.restype = c_bool
        dll.MMR_IsChannelValid.argtypes = [c_char_p, c_short]

        # Channel enable / disable
        dll.SBC_EnableChannel.restype = c_short
        dll.SBC_EnableChannel.argtypes = [c_char_p, c_short]

        dll.SBC_DisableChannel.restype = c_short
        dll.SBC_DisableChannel.argtypes = [c_char_p, c_short]

        # Polling
        dll.SBC_StartPolling.restype = c_bool
        dll.SBC_StartPolling.argtypes = [c_char_p, c_short, c_int]

        dll.SBC_StopPolling.restype = None
        dll.SBC_StopPolling.argtypes = [c_char_p, c_short]

        dll.SBC_PollingDuration.restype = c_int
        dll.SBC_PollingDuration.argtypes = [c_char_p, c_short]

        # Status / position
        dll.SBC_RequestSettings.restype = c_short
        dll.SBC_RequestSettings.argtypes = [c_char_p, c_short]

        dll.SBC_RequestStatusBits.restype = c_short
        dll.SBC_RequestStatusBits.argtypes = [c_char_p, c_short]

        dll.SBC_RequestPosition.restype = c_short
        dll.SBC_RequestPosition.argtypes = [c_char_p, c_short]

        dll.SBC_GetStatusBits.restype = c_uint32
        dll.SBC_GetStatusBits.argtypes = [c_char_p, c_short]

        dll.SBC_GetPosition.restype = c_int
        dll.SBC_GetPosition.argtypes = [c_char_p, c_short]

        # Home
        dll.SBC_CanHome.restype = c_bool
        dll.SBC_CanHome.argtypes = [c_char_p, c_short]

        dll.SBC_Home.restype = c_short
        dll.SBC_Home.argtypes = [c_char_p, c_short]

        # Velocity
        dll.SBC_SetVelParams.restype = c_short
        dll.SBC_SetVelParams.argtypes = [c_char_p, c_short, c_int, c_int]

        dll.SBC_GetVelParams.restype = c_short
        dll.SBC_GetVelParams.argtypes = [c_char_p, c_short, POINTER(c_int), POINTER(c_int)]

        # Relative move
        dll.SBC_MoveRelative.restype = c_short
        dll.SBC_MoveRelative.argtypes = [c_char_p, c_short, c_int]

        # Absolute move
        dll.SBC_SetMoveAbsolutePosition.restype = c_short
        dll.SBC_SetMoveAbsolutePosition.argtypes = [c_char_p, c_short, c_int]

        dll.SBC_GetMoveAbsolutePosition.restype = c_int
        dll.SBC_GetMoveAbsolutePosition.argtypes = [c_char_p, c_short]

        dll.SBC_MoveAbsolute.restype = c_short
        dll.SBC_MoveAbsolute.argtypes = [c_char_p, c_short]

        # Jog
        dll.SBC_SetJogMode.restype = c_short
        dll.SBC_SetJogMode.argtypes = [c_char_p, c_short, c_short, c_short]

        dll.SBC_GetJogStepSize.restype = c_int
        dll.SBC_GetJogStepSize.argtypes = [c_char_p, c_short]

        dll.SBC_SetJogStepSize.restype = c_short
        dll.SBC_SetJogStepSize.argtypes = [c_char_p, c_short, c_int]

        dll.SBC_SetJogVelParams.restype = c_short
        dll.SBC_SetJogVelParams.argtypes = [c_char_p, c_short, c_int, c_int]

        dll.SBC_GetJogVelParams.restype = c_short
        dll.SBC_GetJogVelParams.argtypes = [c_char_p, c_short, POINTER(c_int), POINTER(c_int)]

        dll.SBC_MoveJog.restype = c_short
        dll.SBC_MoveJog.argtypes = [c_char_p, c_short, c_short]

        # Stop
        dll.SBC_StopImmediate.restype = c_short
        dll.SBC_StopImmediate.argtypes = [c_char_p, c_short]

        dll.SBC_StopProfiled.restype = c_short
        dll.SBC_StopProfiled.argtypes = [c_char_p, c_short]

        self.dll = dll

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self):
        check_zero(self.dll.TLI_BuildDeviceList(), "TLI_BuildDeviceList")
        check_zero(self.dll.MMR_Open(self.serial), "MMR_Open")
        self._opened = True

        if not self.dll.MMR_IsChannelValid(self.serial, self.channel):
            raise ThorlabsError(
                f"Invalid channel {self.channel} for serial {self.serial_str}"
            )

        check_zero(
            self.dll.SBC_EnableChannel(self.serial, self.channel),
            "SBC_EnableChannel",
        )
        self._enabled = True

        ok = self.dll.SBC_StartPolling(self.serial, self.channel, self.poll_ms)
        if not ok:
            raise ThorlabsError("SBC_StartPolling failed")
        self._polling = True
        self._actual_poll_ms = self.get_polling_duration()

        time.sleep(1.0)

        check_zero(
            self.dll.SBC_RequestSettings(self.serial, self.channel),
            "SBC_RequestSettings",
        )
        check_zero(
            self.dll.SBC_RequestStatusBits(self.serial, self.channel),
            "SBC_RequestStatusBits",
        )
        check_zero(
            self.dll.SBC_RequestPosition(self.serial, self.channel),
            "SBC_RequestPosition",
        )
        time.sleep(0.5)

    def disconnect(self):
        try:
            if self._polling:
                self.dll.SBC_StopPolling(self.serial, self.channel)
        finally:
            self._polling = False

        try:
            if self._enabled:
                self.dll.SBC_DisableChannel(self.serial, self.channel)
        finally:
            self._enabled = False

        try:
            if self._opened:
                self.dll.MMR_Close(self.serial)
        finally:
            self._opened = False

    def safe_shutdown(self):
        """
        Best-effort safety stop.
        """
        try:
            self.dll.SBC_StopProfiled(self.serial, self.channel)
        except Exception:
            pass

        time.sleep(0.1)

        try:
            self.dll.SBC_StopImmediate(self.serial, self.channel)
        except Exception:
            pass

        self.disconnect()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc is not None:
            self.safe_shutdown()
        else:
            self.disconnect()
        return False

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------
    def request_update(self):
        check_zero(
            self.dll.SBC_RequestStatusBits(self.serial, self.channel),
            "SBC_RequestStatusBits",
        )
        check_zero(
            self.dll.SBC_RequestPosition(self.serial, self.channel),
            "SBC_RequestPosition",
        )
        time.sleep(0.1)

    def get_status_bits(self) -> int:
        return int(self.dll.SBC_GetStatusBits(self.serial, self.channel))

    def get_status_flags(self):
        return decode_status(self.get_status_bits())

    def get_position(self, real_unit: bool = False) -> int:
        if real_unit:
            return self.unit_device2real(value=self.dll.SBC_GetPosition(self.serial, self.channel),type=0)
        else:
            return int(self.dll.SBC_GetPosition(self.serial, self.channel))

    def get_status_poll_interval_s(self) -> float:
        poll_ms = self._actual_poll_ms or self.poll_ms
        return max(0.01, poll_ms / 1000.0)

    def is_moving(self, settle_delay_s: float | None = None) -> bool:
        """
        Return True if the controller reports motion/homing.

        Notes:
        - SBC_GetStatusBits() returns the latest cached status received by the DLL.
        - So we request a fresh update first, then wait at least one polling interval
        before reading the bits.
        """
        check_zero(
            self.dll.SBC_RequestStatusBits(self.serial, self.channel),
            "SBC_RequestStatusBits",
        )

        if settle_delay_s is None:
            settle_delay_s = self.get_status_poll_interval_s()

        time.sleep(settle_delay_s)

        status = self.get_status_bits()

        moving_mask = (
            0x00000010 |  # moving CW
            0x00000020 |  # moving CCW
            0x00000040 |  # jogging CW
            0x00000080 |  # jogging CCW
            0x00000200    # homing
        )
        return (status & moving_mask) != 0
    
    def get_velocity_params(self, real_unit: bool = False):
        acceleration = c_int()
        max_velocity = c_int()
        check_zero(
            self.dll.SBC_GetVelParams(
                self.serial,
                self.channel,
                byref(acceleration),
                byref(max_velocity),
            ),
            "SBC_GetVelParams",
        )
        if real_unit:
            return {
                "acceleration": self.unit_device2real(value=int(acceleration.value),type=2),
                "max_velocity": self.unit_device2real(value=int(max_velocity.value),type=1),
            }
        else:
            return {
                "acceleration": int(acceleration.value),
                "max_velocity": int(max_velocity.value),
            }

    def wait_until_stopped(
        self,
        timeout_s: float = 30.0,
        poll_interval_s: float | None = None,
        require_motion_seen: bool = True,
        motion_start_timeout_s: float = 1.0,
    ):
        """
        Wait until motion has actually finished.

        Why this works better:
        - A move command can be issued and the first status read may still say
        'not moving' because the DLL status is cached.
        - So we optionally wait until motion is seen once, then wait for it to clear.
        """
        if poll_interval_s is None:
            poll_interval_s = self.get_status_poll_interval_s()

        start = time.time()
        deadline = start + timeout_s
        motion_start_deadline = start + motion_start_timeout_s
        saw_motion = False
        while time.time() < deadline:
            moving = self.is_moving(settle_delay_s=poll_interval_s)

            if moving:
                saw_motion = True
            else:
                if (
                    (not require_motion_seen)
                    or saw_motion
                    or time.time() >= motion_start_deadline
                ):
                    return

        raise TimeoutError(
            f"Timed out waiting for motion to stop on channel {self.channel}"
        )

    def wait_until_homed(self, timeout_s: float = 120.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.request_update()
            if self.get_status_bits() & 0x00000400:
                return
            time.sleep(0.3)
        raise TimeoutError("Timed out waiting for homing to complete")

    def set_velocity_params(self, acceleration: int, max_velocity: int, real_unit: bool = False):
        if real_unit:            
            check_zero(
                self.dll.SBC_SetVelParams(
                    self.serial,
                    self.channel,
                    int(self.unit_real2device(value=acceleration,type=2)),
                    int(self.unit_real2device(value=max_velocity,type=1)),
                ),
                "SBC_SetVelParams",
            )

        else:
            check_zero(
                self.dll.SBC_SetVelParams(
                    self.serial,
                    self.channel,
                    int(acceleration),
                    int(max_velocity),
                ),
                "SBC_SetVelParams",
            )

    def get_acceleration(self, real_unit: bool = False) -> int:
        if real_unit:
            return self.unit_device2real(value=self.get_velocity_params()["acceleration"],type=2)
        else:
            return self.get_velocity_params()["acceleration"]

    def get_max_velocity(self, real_unit: bool = False) -> int:
        if real_unit:
            return self.unit_device2real(value=self.get_velocity_params()["max_velocity"],type=1)
        else:
            return self.get_velocity_params()["max_velocity"]

    def set_acceleration(self, acceleration: int, real_unit: bool = False):
        params = self.get_velocity_params()
        if real_unit:
            acceleration = self.unit_real2device(value=acceleration,type=2)
        self.set_velocity_params(acceleration=acceleration, max_velocity=params["max_velocity"])

    def set_max_velocity(self, max_velocity: int, real_unit: bool = False):
        params = self.get_velocity_params()
        if real_unit:
            max_velocity = self.unit_real2device(value=max_velocity,type=1)
        self.set_velocity_params(acceleration=params["acceleration"], max_velocity=max_velocity)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def home(self, wait: bool = True, timeout_s: float = 20.0):
        if not self.dll.SBC_CanHome(self.serial, self.channel):
            raise ThorlabsError("This channel cannot home")

        check_zero(
            self.dll.SBC_Home(self.serial, self.channel),
            "SBC_Home",
        )

        if wait:
            self.wait_until_homed(timeout_s=timeout_s)

    def move_relative(self, displacement: int, wait: bool = True, timeout_s: float = 30.0, real_unit: bool = False):
        if real_unit:
            displacement = self.unit_real2device(value=displacement,type=0)


        check_zero(
            self.dll.SBC_MoveRelative(self.serial, self.channel, int(displacement)),
            "SBC_MoveRelative",
        )

        if wait:
            # sleep_time =  1.6 * abs(self.unit_device2real(value=displacement,type=0)) / self.get_max_velocity(real_unit=True)
            # time.sleep(sleep_time)
            self.wait_until_stopped(timeout_s=timeout_s)

    def set_absolute_target(self, position: int):
        check_zero(
            self.dll.SBC_SetMoveAbsolutePosition(self.serial, self.channel, int(position)),
            "SBC_SetMoveAbsolutePosition",
        )

    def get_absolute_target(self) -> int:
        return int(self.dll.SBC_GetMoveAbsolutePosition(self.serial, self.channel))

    def move_absolute(self, position: int, wait: bool = True, timeout_s: float = 30.0, real_unit: bool = False):
        if real_unit:
            self.set_absolute_target(self.unit_real2device(value=position,type=0))
        else:
            self.set_absolute_target(position)

        check_zero(
            self.dll.SBC_MoveAbsolute(self.serial, self.channel),
            "SBC_MoveAbsolute",
        )

        if wait:
            self.wait_until_stopped(timeout_s=timeout_s)

    # ------------------------------------------------------------------
    # Jog
    # ------------------------------------------------------------------
    def set_jog_mode(self, continuous: bool = False, profiled_stop: bool = True):
        jog_mode = self.JOG_CONTINUOUS if continuous else self.JOG_SINGLE_STEP
        stop_mode = self.STOP_PROFILED if profiled_stop else self.STOP_IMMEDIATE

        check_zero(
            self.dll.SBC_SetJogMode(
                self.serial,
                self.channel,
                jog_mode,
                stop_mode,
            ),
            "SBC_SetJogMode",
        )

    def set_jog_step_size(self, step_size: int):
        check_zero(
            self.dll.SBC_SetJogStepSize(self.serial, self.channel, int(step_size)),
            "SBC_SetJogStepSize",
        )

    def get_jog_step_size(self) -> int:
        return int(self.dll.SBC_GetJogStepSize(self.serial, self.channel))

    def set_jog_velocity_params(self, acceleration: int, max_velocity: int):
        check_zero(
            self.dll.SBC_SetJogVelParams(
                self.serial,
                self.channel,
                int(acceleration),
                int(max_velocity),
            ),
            "SBC_SetJogVelParams",
        )

    def get_jog_velocity_params(self):
        acceleration = c_int()
        max_velocity = c_int()
        check_zero(
            self.dll.SBC_GetJogVelParams(
                self.serial,
                self.channel,
                byref(acceleration),
                byref(max_velocity),
            ),
            "SBC_GetJogVelParams",
        )
        return {
            "acceleration": int(acceleration.value),
            "max_velocity": int(max_velocity.value),
        }

    def jog_forward(self, wait: bool = True, timeout_s: float = 30.0):
        check_zero(
            self.dll.SBC_MoveJog(self.serial, self.channel, self.DIRECTION_FORWARD),
            "SBC_MoveJog(forward)",
        )
        if wait:
            self.wait_until_stopped(timeout_s=timeout_s)

    def jog_backward(self, wait: bool = True, timeout_s: float = 30.0):
        check_zero(
            self.dll.SBC_MoveJog(self.serial, self.channel, self.DIRECTION_BACKWARD),
            "SBC_MoveJog(backward)",
        )
        if wait:
            self.wait_until_stopped(timeout_s=timeout_s)

    # ------------------------------------------------------------------
    # Stop
    # ------------------------------------------------------------------
    def stop_profiled(self):
        check_zero(
            self.dll.SBC_StopProfiled(self.serial, self.channel),
            "SBC_StopProfiled",
        )

    def stop_immediate(self):
        check_zero(
            self.dll.SBC_StopImmediate(self.serial, self.channel),
            "SBC_StopImmediate",
        )

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------
    def print_state(self, label: str = "STATE", real_unit : bool = False):

        status = self.get_status_bits()
        pos = self.get_position(real_unit=real_unit)
        print(f"{label} status: 0x{status:08X}")
        print(f"{label} flags : {decode_status(status)}")
        print(f"{label} pos   : {pos}")
        print(f"{label} vel   : {self.get_velocity_params(real_unit=real_unit)}")

    def unit_device2real(self, value: int, type: int):
        if type==0: # distance
            conv_value = float(value) / 819200
        elif type==1: # velocity
            conv_value = float(value) / 43980465           
        else: # acceleration
            conv_value = float(value) / 9012

        return conv_value   
        
    def unit_real2device(self, value: float, type: int):
        if type==0: # distance
            conv_value = value * 819200
        elif type==1: # velocity
            conv_value = value * 43980465           
        else: # acceleration
            conv_value = value * 9012

        return int(conv_value)    
    
    def get_polling_duration(self) -> int:
        return int(self.dll.SBC_PollingDuration(self.serial, self.channel))

@staticmethod
def x_move(x: float):
    with ThorlabsModularStepperController(serial=SERIAL, channel=1) as motor:
        motor.print_state("INITIAL")
        motor.print_state("INITIAL",real_unit=True)

        motor.set_velocity_params(acceleration=36048, max_velocity=2000000)
        print("Velocity:", motor.get_velocity_params(real_unit=True))
        print("Velocity:", motor.get_velocity_params())

        motor.set_velocity_params(acceleration=5, max_velocity=1.5,real_unit=True)
        print("Velocity:", motor.get_velocity_params(real_unit=True))
        print("Velocity:", motor.get_velocity_params())

        motor.move_relative(x,wait=True,real_unit=True)
        print("Position after relative:", motor.get_position(real_unit=True))

@staticmethod
def y_move(y: float):
    with ThorlabsModularStepperController(serial=SERIAL, channel=2) as motor:
        motor.print_state("INITIAL")
        motor.print_state("INITIAL",real_unit=True)

        motor.set_velocity_params(acceleration=36048, max_velocity=2000000)
        print("Velocity:", motor.get_velocity_params(real_unit=True))
        print("Velocity:", motor.get_velocity_params())

        motor.set_velocity_params(acceleration=5, max_velocity=1.5,real_unit=True)
        print("Velocity:", motor.get_velocity_params(real_unit=True))
        print("Velocity:", motor.get_velocity_params())

        motor.move_relative(y,wait=True,real_unit=True)
        print("Position after relative:", motor.get_position(real_unit=True))

@staticmethod
def homing_cycle():
    # Homing cycle for both axes
    print("=" * 60)
    print("HOMING CYCLE - Channel 1 (X axis)")
    print("=" * 60)
    with ThorlabsModularStepperController(serial=SERIAL, channel=1) as motor:
        motor.print_state("BEFORE HOMING")
        print("\nStarting homing procedure...")
        motor.home(wait=True, timeout_s=30.0)
        print("Homing complete!")
        motor.print_state("AFTER HOMING")
    
    print("\n" + "=" * 60)
    print("HOMING CYCLE - Channel 2 (Y axis)")
    print("=" * 60)
    with ThorlabsModularStepperController(serial=SERIAL, channel=2) as motor:
        motor.print_state("BEFORE HOMING")
        print("\nStarting homing procedure...")
        motor.home(wait=True, timeout_s=30.0)
        print("Homing complete!")
        motor.print_state("AFTER HOMING")
    
    print("\n" + "=" * 60)
    print("HOMING CYCLE COMPLETE")
    print("=" * 60)


def build_axis_points(start: float, span: float, spacing: float):
    if spacing <= 0:
        raise ValueError("spacing must be positive")

    start = float(start)
    span = float(span)
    total = abs(span)
    if total == 0:
        return [start]

    direction = 1.0 if span >= 0 else -1.0
    steps = int(math.floor(total / spacing + 1e-9))
    points = [start + direction * i * spacing for i in range(steps + 1)]
    end = start + span

    if abs(points[-1] - end) > 1e-9:
        points.append(end)

    return points


def motion_profile(distance: float, max_velocity: float, acceleration: float):
    distance = abs(float(distance))
    max_velocity = abs(float(max_velocity))
    acceleration = abs(float(acceleration))

    if distance == 0:
        return {
            "distance": 0.0,
            "ramp_time": 0.0,
            "cruise_time": 0.0,
            "ramp_distance": 0.0,
            "cruise_distance": 0.0,
            "peak_velocity": 0.0,
            "total_time": 0.0,
        }

    if max_velocity <= 0:
        raise ValueError("max_velocity must be positive")
    if acceleration <= 0:
        raise ValueError("acceleration must be positive")

    ramp_time = max_velocity / acceleration
    ramp_distance = 0.5 * acceleration * ramp_time**2

    if distance >= 2 * ramp_distance:
        cruise_distance = distance - 2 * ramp_distance
        cruise_time = cruise_distance / max_velocity
        peak_velocity = max_velocity
    else:
        ramp_time = math.sqrt(distance / acceleration)
        ramp_distance = distance / 2
        cruise_distance = 0.0
        cruise_time = 0.0
        peak_velocity = acceleration * ramp_time

    return {
        "distance": distance,
        "ramp_time": ramp_time,
        "cruise_time": cruise_time,
        "ramp_distance": ramp_distance,
        "cruise_distance": cruise_distance,
        "peak_velocity": peak_velocity,
        "total_time": 2 * ramp_time + cruise_time,
    }


def expected_move_time(distance: float, max_velocity: float, acceleration: float):
    return motion_profile(distance, max_velocity, acceleration)["total_time"]


def sampling_duration_for_move(
    distance: float,
    max_velocity: float,
    acceleration: float,
    safety_factor: float = 1.10,
    overhead_s: float = 0.05,
):
    move_time = expected_move_time(distance, max_velocity, acceleration)
    return move_time * safety_factor + overhead_s


def position_after_elapsed(profile, elapsed_s: float):
    distance = profile["distance"]
    if distance == 0:
        return 0.0

    elapsed_s = max(0.0, float(elapsed_s))
    if elapsed_s >= profile["total_time"]:
        return distance

    ramp_time = profile["ramp_time"]
    cruise_time = profile["cruise_time"]
    acceleration = profile["peak_velocity"] / ramp_time

    if elapsed_s <= ramp_time:
        return 0.5 * acceleration * elapsed_s**2

    cruise_start_s = ramp_time
    cruise_end_s = cruise_start_s + cruise_time
    if elapsed_s <= cruise_end_s:
        return profile["ramp_distance"] + profile["peak_velocity"] * (
            elapsed_s - cruise_start_s
        )

    decel_time = elapsed_s - cruise_end_s
    return (
        profile["ramp_distance"]
        + profile["cruise_distance"]
        + profile["peak_velocity"] * decel_time
        - 0.5 * acceleration * decel_time**2
    )


def sample_positions_for_motion(
    x_start: float,
    displacement: float,
    sample_count: int,
    read_duration_s: float,
    max_velocity: float,
    acceleration: float,
    motion_start_offset_s: float = 0.0,
):
    if sample_count <= 0:
        return []

    profile = motion_profile(displacement, max_velocity, acceleration)
    direction = 1.0 if displacement >= 0 else -1.0

    if sample_count == 1:
        elapsed_times = [read_duration_s / 2]
    else:
        elapsed_times = [
            read_duration_s * i / (sample_count - 1) for i in range(sample_count)
        ]

    return [
        x_start
        + direction
        * position_after_elapsed(profile, elapsed_s - motion_start_offset_s)
        for elapsed_s in elapsed_times
    ]


def plot_scan(scan_rows):
    plotted_rows = 0
    fig, ax = plt.subplots()
    color_map = plt.get_cmap("viridis")
    color_count = max(1, len(scan_rows) - 1)

    for row_index, row in enumerate(scan_rows):
        samples = row["samples"]
        if not samples:
            continue

        row_xs = sample_positions_for_motion(
            x_start=row["x_start"],
            displacement=row["x_displacement"],
            sample_count=len(samples),
            read_duration_s=row.get("sample_span_s", row["read_duration"]),
            max_velocity=row["max_velocity"],
            acceleration=row["acceleration"],
            motion_start_offset_s=row.get("motion_start_offset_s", 0.0),
        )

        ax.plot(
            row_xs,
            samples,
            linewidth=1.0,
            color=color_map(row_index / color_count),
            label=f"y={row['y']:.3f}",
        )
        plotted_rows += 1

    if plotted_rows == 0:
        print("No samples collected; skipping scan plot.")
        return

    ax.set_xlabel("X position")
    ax.set_ylabel("ADC count")
    ax.set_title("Scanned Lines")
    ax.grid(True)

    if plotted_rows <= 20:
        ax.legend(title="Row", fontsize="small", ncols=2)

    fig.tight_layout()
    plt.show()


def check_connection_and_home(motorx, motory, home_timeout_s: float = 30.0):
    print("Checking motor connections...")
    for label, motor in (("X", motorx), ("Y", motory)):
        motor.request_update()
        status = motor.get_status_bits()
        position = motor.get_position(real_unit=True)
        print(
            f"{label} connected: "
            f"status=0x{status:08X}, "
            f"position={position:.6f}, "
            f"poll={motor.get_polling_duration()} ms"
        )

    print("Homing motors...")
    for label, motor in (("X", motorx), ("Y", motory)):
        print(f"Homing {label} axis...")
        motor.home(wait=True, timeout_s=home_timeout_s)
        motor.request_update()

        status = motor.get_status_bits()
        if not status & 0x00000400:
            raise ThorlabsError(f"{label} axis did not report homed")

        print(f"{label} homed at position {motor.get_position(real_unit=True):.6f}")

    print("Connection and homing check complete.")


def print_scan_timing_stats(scan_rows):
    if not scan_rows:
        return

    move_to_read_delays = [
        row["move_to_read_start_s"]
        for row in scan_rows
        if "move_to_read_start_s" in row
    ]
    command_call_times = [
        row["move_command_call_s"]
        for row in scan_rows
        if "move_command_call_s" in row
    ]
    return_to_read_delays = [
        row["move_return_to_read_start_s"]
        for row in scan_rows
        if "move_return_to_read_start_s" in row
    ]
    sample_spans = [
        row["sample_span_s"]
        for row in scan_rows
        if "sample_span_s" in row
    ]
    motion_start_offsets = [
        row["motion_start_offset_s"]
        for row in scan_rows
        if "motion_start_offset_s" in row
    ]

    def print_stats(label, values):
        if not values:
            return

        avg = sum(values) / len(values)
        print(
            f"{label}: "
            f"avg={avg * 1000:.2f} ms, "
            f"min={min(values) * 1000:.2f} ms, "
            f"max={max(values) * 1000:.2f} ms"
        )

    print("Timing summary:")
    print_stats("Move command issue -> burst read loop start", move_to_read_delays)
    print_stats("Move command return -> burst read loop start", return_to_read_delays)
    print_stats("move_relative() call duration", command_call_times)
    print_stats("Plot compensation offset", motion_start_offsets)
    print_stats("Buffered sample span used for plotting", sample_spans)




if __name__ == "__main__":
    SERIAL = "50865380"
    ARDUINO_PORT = "COM3"
    ARDUINO_BAUD = 230400

    x0 = 1.0
    y0 = 1.0
    x_span = 1.0
    y_span = 0
    line_spacing = 0.1
    default_acceleration = 4.0
    default_max_velocity = 4.0
    row_settle_s = 2
    read_safety_factor = 1.10
    read_overhead_s = 0.05
    home_timeout_s = 30.0
    skip_homing_check = False
    motion_start_reference = "command_return"

    x_start = x0 - x_span / 2
    y_start = y0 - y_span / 2
    y_points = build_axis_points(y_start, y_span, line_spacing)
    scan_rows = []

    motorx = None
    motory = None
    ser = None
    scan_failed = True

    try:
        motorx = ThorlabsModularStepperController(
            serial=SERIAL,
            channel=1,
            poll_ms=1,
        )
        motory = ThorlabsModularStepperController(
            serial=SERIAL,
            channel=2,
            poll_ms=1,
        )
        motorx.connect()
        motory.connect()
        if skip_homing_check:
            print("Skipping motor connection/homing check.")
        else:
            check_connection_and_home(
                motorx=motorx,
                motory=motory,
                home_timeout_s=home_timeout_s,
            )

        motorx.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )
        motory.set_velocity_params(
            acceleration=default_acceleration,
            max_velocity=default_max_velocity,
            real_unit=True,
        )

        motorx.move_absolute(x_start, wait=True, real_unit=True)
        motory.move_absolute(y_start, wait=True, real_unit=True)

        ser = open_arduino(port=ARDUINO_PORT, baud=ARDUINO_BAUD)

        move_time = expected_move_time(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
        )
        read_duration = sampling_duration_for_move(
            distance=x_span,
            max_velocity=default_max_velocity,
            acceleration=default_acceleration,
            safety_factor=read_safety_factor,
            overhead_s=read_overhead_s,
        )

        print(
            f"Scanning {len(y_points)} rows; "
            f"expected X move time {move_time:.3f}s, "
            f"read duration {read_duration:.3f}s per row."
        )

        for row_index, y_target in enumerate(y_points):
            if row_index > 0:
                motory.move_absolute(y_target, wait=True, real_unit=True)
                time.sleep(row_settle_s)

            direction = 1.0 if row_index % 2 == 0 else -1.0
            x_displacement = direction * x_span
            actual_x_start = motorx.get_position(real_unit=True)
            actual_y = motory.get_position(real_unit=True)

            reset_done_s = time.perf_counter()
            ser.reset_input_buffer()
            sample_window_start_s = time.perf_counter()
            move_command_issue_s = time.perf_counter()
            motorx.move_relative(x_displacement, wait=False, real_unit=True)
            move_command_return_s = time.perf_counter()
            samples, read_timing = burst_read_binary(
                ser=ser,
                duration=read_duration,
                reset_buffer=False,
                return_timing=True,
            )
            sample_window_end_s = read_timing["end_time"]

            if motion_start_reference == "command_issue":
                motion_start_s = move_command_issue_s
            elif motion_start_reference == "command_return":
                motion_start_s = move_command_return_s
            else:
                raise ValueError(
                    "motion_start_reference must be 'command_issue' "
                    "or 'command_return'"
                )

            motorx.wait_until_stopped(
                timeout_s=max(read_duration + 2.0, move_time * 2.0 + 2.0),
                require_motion_seen=False,
            )
            actual_x_end = motorx.get_position(real_unit=True)

            scan_rows.append(
                {
                    "row": row_index,
                    "y": actual_y,
                    "x_start": actual_x_start,
                    "x_end": actual_x_end,
                    "x_displacement": x_displacement,
                    "motion_time": move_time,
                    "read_duration": read_duration,
                    "sample_span_s": sample_window_end_s - sample_window_start_s,
                    "motion_start_offset_s": motion_start_s - sample_window_start_s,
                    "move_to_read_start_s": (
                        read_timing["start_time"] - move_command_issue_s
                    ),
                    "move_return_to_read_start_s": (
                        read_timing["start_time"] - move_command_return_s
                    ),
                    "move_command_call_s": (
                        move_command_return_s - move_command_issue_s
                    ),
                    "serial_reset_s": sample_window_start_s - reset_done_s,
                    "acceleration": default_acceleration,
                    "max_velocity": default_max_velocity,
                    "samples": samples,
                }
            )
            print(
                f"Row {row_index + 1}/{len(y_points)}: "
                f"y={actual_y:.6f}, "
                f"x={actual_x_start:.6f}->{actual_x_end:.6f}, "
                f"samples={len(samples)}, "
                f"cmd->read={scan_rows[-1]['move_to_read_start_s'] * 1000:.2f} ms"
            )

        print_scan_timing_stats(scan_rows)
        scan_failed = False

    finally:
        if ser is not None:
            close_arduino(ser)

        if scan_failed:
            if motorx is not None:
                motorx.safe_shutdown()
            if motory is not None:
                motory.safe_shutdown()
        else:
            if motorx is not None:
                motorx.disconnect()
            if motory is not None:
                motory.disconnect()

    plot_scan(scan_rows)
