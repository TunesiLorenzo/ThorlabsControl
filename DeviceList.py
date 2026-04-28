import clr

clr.AddReference(r"C:\Program Files\Thorlabs\Kinesis\Thorlabs.MotionControl.DeviceManagerCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI

DeviceManagerCLI.BuildDeviceList()

try:
    serials = DeviceManagerCLI.GetDeviceList()
    print("Device list:", list(serials))
except Exception as e:
    print("GetDeviceList failed:", e)