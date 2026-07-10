import time

import serial


def open_arduino(port="COM3", baud=230400, reset_wait_s=2.0):
    """Open the Arduino serial port and wait for the board reset."""
    ser = serial.Serial(port, baud, timeout=0)
    if reset_wait_s:
        time.sleep(reset_wait_s)
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
    """Read one timed burst from a continuously streaming Arduino.

    reset_buffer=True discards bytes already waiting in the PC-side serial
    buffer immediately before timing starts. The Arduino firmware still
    free-runs unless it is changed to a command-triggered protocol.
    """
    if reset_buffer:
        ser.reset_input_buffer()

    raw = bytearray()
    start = time.perf_counter()
    deadline = start + float(duration)

    while time.perf_counter() < deadline:
        waiting = ser.in_waiting
        if waiting:
            raw.extend(ser.read(waiting))
        elif idle_sleep_s:
            time.sleep(idle_sleep_s)

    end = time.perf_counter()
    samples = list(raw)

    if not return_timing:
        return samples

    return samples, {
        "start_time": start,
        "end_time": end,
        "duration": end - start,
    }


def plot(samples):
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(samples)
    plt.title("ADC Binary Burst")
    plt.xlabel("Sample index")
    plt.ylabel("ADC count")
    plt.grid()


if __name__ == "__main__":
    ser = open_arduino(port="COM3", baud=230400)
    try:
        samples = burst_read_binary(ser=ser, duration=0.1)
        print(len(samples), "samples")
        plot(samples)

        import matplotlib.pyplot as plt

        plt.show()
    finally:
        close_arduino(ser)
