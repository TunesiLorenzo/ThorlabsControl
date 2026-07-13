import time

import serial


def open_arduino(port="COM3", baud=230400, reset_wait_s=2.0, rx_buffer_size=65536):
    """Open the Arduino serial port and wait for the board reset.

    rx_buffer_size raises the OS-level receive buffer past pyserial's
    default (4096 bytes on Windows, ~178 ms of headroom at 230400 baud), so
    a transient stall in whatever thread is draining the port -- e.g. a GUI
    main thread busy redrawing -- doesn't silently drop bytes instead of
    just delaying them. No-op on platforms without set_buffer_size (e.g.
    pyserial on Linux/macOS).
    """
    ser = serial.Serial(port, baud, timeout=0)
    if hasattr(ser, "set_buffer_size"):
        ser.set_buffer_size(rx_size=rx_buffer_size)
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
    stop_condition=None,
    hard_timeout=None,
    stop_poll_interval_s=0.005,
):
    """Read one timed burst from a continuously streaming Arduino.

    reset_buffer=True discards bytes already waiting in the PC-side serial
    buffer immediately before timing starts. The Arduino firmware still
    free-runs unless it is changed to a command-triggered protocol.

    By default the burst ends the instant `duration` elapses. If
    `stop_condition` is given, `duration` instead becomes a *minimum*: once
    it elapses, the loop keeps draining the serial port and re-checks
    stop_condition() every `stop_poll_interval_s` until it returns True (or
    `hard_timeout`, default 2 * duration, is reached) -- e.g. so a scan row
    isn't cut short if the stage started moving later than the modeled
    profile assumed and is still mid-move when `duration` elapses.
    """
    if reset_buffer:
        ser.reset_input_buffer()

    raw = bytearray()
    start = time.perf_counter()
    min_deadline = start + float(duration)
    hard_deadline = start + float(hard_timeout if hard_timeout is not None else duration * 2.0)
    next_stop_check = min_deadline

    while True:
        now = time.perf_counter()
        if now >= min_deadline:
            if stop_condition is None or now >= hard_deadline:
                break
            if now >= next_stop_check:
                next_stop_check = now + stop_poll_interval_s
                if stop_condition():
                    break

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
