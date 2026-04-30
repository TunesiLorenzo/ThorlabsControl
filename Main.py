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
import serial 
import numpy as np

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
        ser: serial = None,
        val: list = [],
    ):
        self.serial_str = str(serial)
        self.serial = self.serial_str.encode("ascii")
        self.channel = int(channel)
        self.kinesis_dir = kinesis_dir
        self.poll_ms = int(poll_ms)
        self.ser = None
        self.val = []

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
        check_zero(
            self.dll.SBC_RequestPosition(self.serial, self.channel),
            "SBC_RequestPosition",
        )

        if settle_delay_s is None:
            settle_delay_s = max(0.01, self.poll_ms / 1000.0)

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
    ):
        """
        Wait until motion has actually finished.

        Why this works better:
        - A move command can be issued and the first status read may still say
        'not moving' because the DLL status is cached.
        - So we optionally wait until motion is seen once, then wait for it to clear.
        """
        if poll_interval_s is None:
            poll_interval_s = max(0.01, self.poll_ms / 1000.0)

        deadline = time.time() + timeout_s
        saw_motion = False
        while time.time() < deadline:
            moving = self.is_moving(settle_delay_s=poll_interval_s)

            if moving:
                saw_motion = True
            else:
                if (not require_motion_seen) or saw_motion:
                    return
            self.val.append(get_reading(ser))
            time.sleep(poll_interval_s)

        # raise TimeoutError(
        #     f"Timed out waiting for motion to stop on channel {self.channel}"
        # )

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


def get_reading(ser=None):
    """Read one line and return a float, or None if invalid/empty."""
    close_after = False
    if ser is None:
        ser = serial.Serial(SERIAL_PORT, SERIAL_RATE, timeout=1)
        close_after = True

    try:
        raw = ser.readline()
        if not raw:
            return None

        text = raw.decode("utf-8", errors="ignore").strip()
        if text == "":
            return None

        return float(text)

    except ValueError:
        return None

    finally:
        if close_after:
            ser.close()


if __name__ == "__main__":
    SERIAL = "50865380"
    SERIAL_PORT = 'COM4'    
    SERIAL_RATE = 115200
    
    # Uncomment to also run movement tests after homing 
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)
    
    motorx = ThorlabsModularStepperController(serial=SERIAL, channel=1, poll_ms=30, ser=ser)
    
    motory = ThorlabsModularStepperController(serial=SERIAL, channel=2, poll_ms=30, ser=ser)
    
    try:
        motorx.connect()
        motory.connect()

        motorx.set_velocity_params(acceleration=2, max_velocity=0.1,real_unit=True)
        motory.set_velocity_params(acceleration=2, max_velocity=0.1,real_unit=True)
        

        # motorx.home()
        # motory.home()

        motorx.move_absolute(0,wait=False,real_unit=True)
        motory.move_absolute(0,wait=False,real_unit=True)
        time.sleep(3)

        Result = []
        xpos=[0]
        ypos=[0]


        motorx.move_relative(1,wait=True,real_unit=True)
        Result.append(motorx.val.copy())
        xpos.append(motorx.get_position(real_unit=True))
        ypos.append(motory.get_position(real_unit=True))
        motorx.val.clear()

        
        motory.move_relative(1,wait=True,real_unit=True)
        Result.append(motory.val.copy())
        xpos.append(motorx.get_position(real_unit=True))
        ypos.append(motory.get_position(real_unit=True))
        motory.val.clear()

        motorx.move_relative(-0.9,wait=True,real_unit=True)
        Result.append(motorx.val.copy())
        xpos.append(motorx.get_position(real_unit=True))
        ypos.append(motory.get_position(real_unit=True))
        motorx.val.clear()

        
        # motory.move_relative(-0.9,wait=True,real_unit=True)
        # xpos.append(motorx.get_position(real_unit=True))
        # ypos.append(motory.get_position(real_unit=True))
        # Result.append(motory.val.copy())
        # motory.val=[]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        for i in range(len(Result)):
            xvec = np.linspace(xpos[i], xpos[i+1], len(Result[i]))
            yvec = np.linspace(ypos[i], ypos[i+1], len(Result[i]))
            zvec = np.asarray(Result[i], dtype=float)
            ax.plot([xpos[i], xpos[i+1]], [ypos[i], ypos[i+1]], [0, 0], alpha=0.3)
            ax.scatter(xvec, yvec, zvec, s=8)

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("signal")
        plt.show()

    finally:
        try:
            motorx.stop_profiled()
        except Exception:
            pass
        try:
            motory.stop_profiled()
        except Exception:
            pass
        try:
            motorx.disconnect()
        except Exception:
            pass
        try:
            motory.disconnect()
        except Exception:
            pass



    # motory.move_absolute(1,real_unit=True)
    # t1=time.time()
    # motorx.get_position(real_unit=True)
    # t2=time.time()
    # print("Time to get_position ", t2-t1)

    # starting_range=1
    # line_spacing=0.015
    # time_delay = 0
    # distance=starting_range
    # xpos=[]
    # ypos=[]
    # while distance>0.02:

    #     motorx.set_velocity_params(acceleration=2, max_velocity=distance*2,real_unit=True)
    #     motory.set_velocity_params(acceleration=2, max_velocity=distance*2,real_unit=True)


    #     motorx.move_relative(distance,wait=True,real_unit=True)
    #     #sample logic
    #     time.sleep(time_delay)
    #     xpos.append(motorx.get_position(real_unit=True))
    #     ypos.append(motory.get_position(real_unit=True))

    #     motory.move_relative(distance,wait=True,real_unit=True)
    #     #sample logic
    #     time.sleep(time_delay)
    #     xpos.append(motorx.get_position(real_unit=True))
    #     ypos.append(motory.get_position(real_unit=True))

    #     distance = distance - line_spacing

    #     motorx.move_relative(-distance,wait=True,real_unit=True)
    #     #sample logic
    #     time.sleep(time_delay)
    #     xpos.append(motorx.get_position(real_unit=True))
    #     ypos.append(motory.get_position(real_unit=True))

    #     motory.move_relative(-distance,wait=True,real_unit=True)
    #     #sample logic
    #     time.sleep(time_delay)
    #     xpos.append(motorx.get_position(real_unit=True))
    #     ypos.append(motory.get_position(real_unit=True))

    #     distance = distance - line_spacing



    # plt.figure()
    # plt.plot(xpos, ypos, "o-", label="points")  # o = markers, - = connecting line
    # plt.xlabel("x position")
    # plt.ylabel("y position")
    # plt.title("X vs Y positions")
    # plt.grid(True)
    # plt.legend()
    # plt.show()

