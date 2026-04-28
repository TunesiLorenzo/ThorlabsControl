import os
import sys
import time
from ctypes import (
    WinDLL, byref, POINTER,
    c_char_p, c_short, c_int, c_bool, c_uint32, c_ushort, c_double
)


KINESIS_DIR = r"C:\Program Files\Thorlabs\Kinesis"
SERIAL = b"50865380"   # MST602 module serial
CHANNEL = 1            # use 1 or 2
MOVE_TYPE = 0 # 0 for relative 1 for absolute
REL_MOVE = 50000      # small safe test move in device units
POLL_MS = 400
DO_HOME = True    # set True if you want to test homing first


class ThorlabsError(RuntimeError):
    pass


def check_zero(result, name):
    if result != 0:
        raise ThorlabsError(f"{name} failed with error code {result}")


def decode_status(status):
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


def wait_until_not_moving(dll, serial, channel, timeout_s=30.0):
    moving_mask = 0x00000010 | 0x00000020 | 0x00000040 | 0x00000080
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = dll.SBC_GetStatusBits(serial, channel)
        if (status & moving_mask) == 0:
            return
        time.sleep(0.1)
    raise TimeoutError("Timed out waiting for motion to stop")


def wait_until_homed(dll, serial, channel, timeout_s=120.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = dll.SBC_GetStatusBits(serial, channel)
        homing = bool(status & 0x00000200)
        homed = bool(status & 0x00000400)
        if homed and not homing:
            return
        time.sleep(0.2)
    raise TimeoutError("Timed out waiting for homing to complete")


def main():
    os.add_dll_directory(KINESIS_DIR)

    WinDLL(os.path.join(KINESIS_DIR, "Thorlabs.MotionControl.DeviceManager.dll"))
    WinDLL(os.path.join(KINESIS_DIR, "Thorlabs.MotionControl.Benchtop.StepperMotor.dll"))
    WinDLL(os.path.join(KINESIS_DIR, "Thorlabs.MotionControl.Benchtop.NanoTrak.dll"))
    WinDLL(os.path.join(KINESIS_DIR, "Thorlabs.MotionControl.Benchtop.Piezo.dll"))
    dll = WinDLL(os.path.join(KINESIS_DIR, "Thorlabs.MotionControl.ModularRack.dll"))

    dll.TLI_BuildDeviceList.restype = c_short
    dll.TLI_BuildDeviceList.argtypes = []

    dll.MMR_Open.restype = c_short
    dll.MMR_Open.argtypes = [c_char_p]

    dll.MMR_Close.restype = None
    dll.MMR_Close.argtypes = [c_char_p]

    dll.MMR_IsChannelValid.restype = c_bool
    dll.MMR_IsChannelValid.argtypes = [c_char_p, c_short]

    dll.SBC_EnableChannel.restype = c_short
    dll.SBC_EnableChannel.argtypes = [c_char_p, c_short]

    dll.SBC_DisableChannel.restype = c_short
    dll.SBC_DisableChannel.argtypes = [c_char_p, c_short]

    dll.SBC_StartPolling.restype = c_bool
    dll.SBC_StartPolling.argtypes = [c_char_p, c_short, c_int]

    dll.SBC_StopPolling.restype = None
    dll.SBC_StopPolling.argtypes = [c_char_p, c_short]

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

    dll.SBC_CanHome.restype = c_bool
    dll.SBC_CanHome.argtypes = [c_char_p, c_short]

    dll.SBC_Home.restype = c_short
    dll.SBC_Home.argtypes = [c_char_p, c_short]

    dll.SBC_MoveRelative.restype = c_short
    dll.SBC_MoveRelative.argtypes = [c_char_p, c_short, c_int]

    check_zero(dll.TLI_BuildDeviceList(), "TLI_BuildDeviceList")
    check_zero(dll.MMR_Open(SERIAL), "MMR_Open")

    try:
        if not dll.MMR_IsChannelValid(SERIAL, CHANNEL):
            raise ThorlabsError(f"Invalid channel {CHANNEL} for serial {SERIAL.decode()}")

        check_zero(dll.SBC_EnableChannel(SERIAL, CHANNEL), "SBC_EnableChannel")

        ok = dll.SBC_StartPolling(SERIAL, CHANNEL, POLL_MS)
        if not ok:
            raise ThorlabsError("SBC_StartPolling failed")

        time.sleep(1.0)

        check_zero(dll.SBC_RequestSettings(SERIAL, CHANNEL), "SBC_RequestSettings")
        check_zero(dll.SBC_RequestStatusBits(SERIAL, CHANNEL), "SBC_RequestStatusBits")
        check_zero(dll.SBC_RequestPosition(SERIAL, CHANNEL), "SBC_RequestPosition")
        time.sleep(0.5)

        status = dll.SBC_GetStatusBits(SERIAL, CHANNEL)
        pos = dll.SBC_GetPosition(SERIAL, CHANNEL)

        print(f"Initial status: 0x{status:08X}")
        print("Initial flags :", decode_status(status))
        print("Initial pos   :", pos)
        print("Can home      :", bool(dll.SBC_CanHome(SERIAL, CHANNEL)))

        if DO_HOME:
            print("Starting home...")
            check_zero(dll.SBC_Home(SERIAL, CHANNEL), "SBC_Home")
            wait_until_homed(dll, SERIAL, CHANNEL)
            print("Home complete")

        if MOVE_TYPE:
            print(f"Moving by {REL_MOVE} device units (ABSOLUTE)...")
            check_zero(dll.SBC_MoveAbsolute(SERIAL, CHANNEL, REL_MOVE), "SBC_MoveAbsolute")
            wait_until_not_moving(dll, SERIAL, CHANNEL)
        else:
            print(f"Moving by {REL_MOVE} device units...")
            check_zero(dll.SBC_MoveToPosition(SERIAL, CHANNEL, REL_MOVE), "SBC_MoveToPosition")
            wait_until_not_moving(dll, SERIAL, CHANNEL)

        check_zero(dll.SBC_RequestPosition(SERIAL, CHANNEL), "SBC_RequestPosition")
        check_zero(dll.SBC_RequestStatusBits(SERIAL, CHANNEL), "SBC_RequestStatusBits")
        time.sleep(1)

        final_status = dll.SBC_GetStatusBits(SERIAL, CHANNEL)
        final_pos = dll.SBC_GetPosition(SERIAL, CHANNEL)

        print(f"Final status  : 0x{final_status:08X}")
        print("Final flags   :", decode_status(final_status))
        print("Final pos     :", final_pos)

    finally:
        try:
            dll.SBC_StopPolling(SERIAL, CHANNEL)
        except Exception:
            pass
        try:
            dll.SBC_DisableChannel(SERIAL, CHANNEL)
        except Exception:
            pass
        try:
            dll.MMR_Close(SERIAL)
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc)
        sys.exit(1)