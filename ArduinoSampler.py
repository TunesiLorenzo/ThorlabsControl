import serial
import time
import matplotlib.pyplot as plt

def open_arduino(port="COM3", baud=230400):
    ser = serial.Serial(port, baud, timeout=0)
    time.sleep(2.0)  # Arduino reset after opening serial
    return ser

def close_arduino(ser):
    ser.close()


def burst_read_binary(
    ser,
    duration=1.0,
    reset_buffer=True,
    return_timing=False,
    idle_sleep_s=0.0005,
):
    if reset_buffer:
        ser.reset_input_buffer()

    raw = bytearray()
    start = time.perf_counter()

    while time.perf_counter() - start < duration:
        n = ser.in_waiting
        if n:
            raw.extend(ser.read(n))
        else:
            raw.extend(ser.read(1))
            if idle_sleep_s:
                time.sleep(idle_sleep_s)
    end = time.perf_counter()

    # ArduinoCode.ino sends one 8-bit ADC sample per byte.
    samples = list(raw)
    if return_timing:
        return samples, {
            "start_time": start,
            "end_time": end,
            "duration": end - start,
        }
    return samples

def plot(samples):
    plt.figure()
    plt.plot(samples)
    plt.title("ADC Binary Burst")
    plt.xlabel("Sample index")
    plt.ylabel("ADC count")
    plt.grid()


if __name__ == "__main__":
    ser = open_arduino(port="COM3", baud=230400)
    samples = burst_read_binary(ser=ser, duration=0.1)
    plot(samples)
    print(len(samples), "samples")

    # samples = burst_read_binary(ser=ser, duration=2)
    # plot(samples)
    # print(len(samples), "samples")

    close_arduino(ser)

    plt.show()
