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

    def is_moving(self) -> bool:
        status = self.get_status_bits()
        moving_mask = 0x00000010 | 0x00000020 | 0x00000040 | 0x00000080 | 0x00000200
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

    def wait_until_stopped(self, timeout_s: float = 30.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if not self.is_moving():
                return
            time.sleep(0.3)
        raise TimeoutError("Timed out waiting for motion to stop")

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
            sleep_time =  1.6 * abs(self.unit_device2real(value=displacement,type=0)) / self.get_max_velocity(real_unit=True)

            time.sleep(sleep_time)

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

if __name__ == "__main__":
    SERIAL = "50865380"
    
    # Uncomment to also run movement tests after homing
    i = 0
    for i in range(10):
        x_move(0.5)
        y_move(0.1)
        x_move(-0.5)
        i += 1
    print(f"Completed iteration {i}")
    # with ThorlabsModularStepperController(serial=SERIAL, channel=1) as motor:
    #     motor.print_state("INITIAL")
    #     motor.print_state("INITIAL",real_unit=True)

    #     motor.set_velocity_params(acceleration=36048, max_velocity=2000000)
    #     print("Velocity:", motor.get_velocity_params(real_unit=True))
    #     print("Velocity:", motor.get_velocity_params())

    #     motor.set_velocity_params(acceleration=5, max_velocity=1.5,real_unit=True)
    #     print("Velocity:", motor.get_velocity_params(real_unit=True))
    #     print("Velocity:", motor.get_velocity_params())

    #     x = 0.5

    #     motor.move_relative(x,wait=True,real_unit=True)
    #     print("Position after relative:", motor.get_position(real_unit=True))

    #     motor.move_relative(-x,wait=True,real_unit=True)
    #     print("Position after relative:", motor.get_position(real_unit=True))

    #     motor.move_relative(x,wait=True,real_unit=True)
    #     print("Position after relative:", motor.get_position(real_unit=True))

    #     motor.move_relative(-x,wait=True,real_unit=True)
    #     print("Position after relative:", motor.get_position(real_unit=True))
    '''
        motor.move_absolute(0,wait=False)
        print("Position after absolute:", motor.get_position())
        
        motor.set_jog_mode(continuous=False, profiled_stop=True)
        motor.set_jog_step_size(1000000)
        motor.set_jog_velocity_params(acceleration=36048, max_velocity=120000000)

        motor.jog_forward(wait=False)
        print("Position after jog forward:", motor.get_position())

        motor.set_jog_velocity_params(acceleration=36048, max_velocity=2000000)
        motor.jog_forward(wait=False)
        print("Position after jog backward:", motor.get_position())
        '''
    
      #  motor.print_state("FINAL")