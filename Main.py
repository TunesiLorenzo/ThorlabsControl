import math
import time

import matplotlib.pyplot as plt
<<<<<<< HEAD
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
=======
import numpy as np

from ArduinoSampler import close_arduino, open_arduino, burst_read_binary
from motion_timing import (
    MoveStartLagMonitor,
    expected_move_time,
    sample_positions_for_motion,
    sampling_duration_for_move,
)
from ThorlabsStepper import ThorlabsError, ThorlabsModularStepperController


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


def load_scan(path):
    """Reload scan_rows saved by the __main__ scan loop's np.savez() dump."""
    data = np.load(path, allow_pickle=True)
    return data["scan_rows"].tolist()


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
>>>>>>> NewStart
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

<<<<<<< HEAD
            if moving:
                saw_motion = True
            else:
                if (not require_motion_seen) or saw_motion:
                    return
            self.val.append(get_reading(ser))
            time.sleep(poll_interval_s)
=======
    fig.tight_layout()
    plt.show()

>>>>>>> NewStart

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
    measured_lag_lower = [
        row["measured_lag_lower_s"]
        for row in scan_rows
        if row.get("measured_lag_lower_s") is not None
    ]
    measured_lag_upper = [
        row["measured_lag_upper_s"]
        for row in scan_rows
        if row.get("measured_lag_upper_s") is not None
    ]
    measured_lag_midpoint = [
        row["measured_lag_midpoint_s"]
        for row in scan_rows
        if row.get("measured_lag_midpoint_s") is not None
    ]
    measured_moving_bit_lag = [
        row["measured_moving_bit_lag_s"]
        for row in scan_rows
        if row.get("measured_moving_bit_lag_s") is not None
    ]
    n_lag_missed = sum(
        1 for row in scan_rows if row.get("measured_lag_midpoint_s") is None
    )

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
    print_stats("Plot compensation offset (measured lag midpoint, used for X projection)", motion_start_offsets)
    print_stats("Buffered sample span used for plotting", sample_spans)
    print_stats("Measured move-start lag, lower bound", measured_lag_lower)
    print_stats("Measured move-start lag, upper bound", measured_lag_upper)
    print_stats("Measured move-start lag, midpoint", measured_lag_midpoint)
    print_stats("Measured move-start lag, moving-bit", measured_moving_bit_lag)
    if n_lag_missed:
        print(
            f"Move-start lag was not detected on {n_lag_missed}/{len(scan_rows)} "
            "row(s); those rows fell back to command-return timing."
        )

<<<<<<< HEAD
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

=======

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
    skip_homing_check = True
    # Position-based move-start-lag detection (see motion_timing.MoveStartLagMonitor).
    move_start_lag_threshold_mm = 0.0000005  # 0.5 nm; must exceed quantization noise
    lag_monitor_timeout_s = 2.0

    # Dump every row's raw samples + timing/lag metadata (including whatever
    # tail was sampled after the motor actually stopped) so they can be
    # re-plotted/re-analyzed without re-running the hardware.
    SAVE_FILE = "scan_last.npz"

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

            # Watches get_position()/status bits in the background while this
            # thread issues the move and then blocks on the Arduino read, so
            # we get the real per-row motion-start lag instead of assuming
            # command-issue/command-return timing.
            lag_monitor = MoveStartLagMonitor(
                motorx,
                pos0=actual_x_start,
                position_threshold_mm=move_start_lag_threshold_mm,
                timeout_s=lag_monitor_timeout_s,
            ).start()

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

            lag_monitor.join(timeout=lag_monitor_timeout_s)
            lag_result = lag_monitor.result(move_command_issue_s)
            if lag_result is not None:
                motion_start_s = move_command_issue_s + lag_result["lag_midpoint_s"]
            else:
                print(
                    f"Row {row_index}: move-start lag not detected within "
                    f"{lag_monitor_timeout_s:.1f}s; falling back to "
                    "command-return timing for the X projection."
                )
                motion_start_s = move_command_return_s
                lag_result = {}

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
                    "measured_lag_lower_s": lag_result.get("lag_lower_bound_s"),
                    "measured_lag_upper_s": lag_result.get("lag_upper_bound_s"),
                    "measured_lag_midpoint_s": lag_result.get("lag_midpoint_s"),
                    "measured_moving_bit_lag_s": lag_result.get("moving_bit_lag_s"),
                    "acceleration": default_acceleration,
                    "max_velocity": default_max_velocity,
                    "samples": samples,
                }
            )
            lag_mid_s = scan_rows[-1]["measured_lag_midpoint_s"]
            lag_mid_str = "n/a" if lag_mid_s is None else f"{lag_mid_s * 1000:.2f} ms"
            print(
                f"Row {row_index + 1}/{len(y_points)}: "
                f"y={actual_y:.6f}, "
                f"x={actual_x_start:.6f}->{actual_x_end:.6f}, "
                f"samples={len(samples)}, "
                f"cmd->read={scan_rows[-1]['move_to_read_start_s'] * 1000:.2f} ms, "
                f"measured lag mid={lag_mid_str}"
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

    if SAVE_FILE and scan_rows:
        np.savez(SAVE_FILE, scan_rows=np.array(scan_rows, dtype=object))
        print(f"Saved {len(scan_rows)} row(s) (raw samples + timing/lag metadata) to {SAVE_FILE}")

    plot_scan(scan_rows)
>>>>>>> NewStart
