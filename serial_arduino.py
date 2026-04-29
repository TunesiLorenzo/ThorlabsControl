import time
from math import ceil

import serial


# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = '/dev/ttyUSB0'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def x_axis_sample_count(
    x_displacement_mm: float,
    y_step_mm: float,
    include_endpoints: bool = True,
) -> int:
    if y_step_mm == 0:
        raise ValueError("y_step_mm must not be zero")

    intervals = ceil(abs(float(x_displacement_mm)) / abs(float(y_step_mm)))
    return intervals + 1 if include_endpoints else intervals


def read_serial_float(
    port: str = SERIAL_PORT,
    rate: int = SERIAL_RATE,
    x_displacement_mm: float = 1.0,
    y_step_mm: float = 1.0,
    sample_delay: float = 0.0,
    include_endpoints: bool = True,
):
    sample_count = x_axis_sample_count(
        x_displacement_mm=x_displacement_mm,
        y_step_mm=y_step_mm,
        include_endpoints=include_endpoints,
    )

    if sample_count < 1:
        raise ValueError("sample_count must be at least 1")

    values = []
    with serial.Serial(port, rate) as ser:
        for index in range(sample_count):
            raw_reading = ser.readline().decode('utf-8').strip()
            if not raw_reading:
                continue

            try:
                values.append(float(raw_reading))
            except ValueError as exc:
                raise ValueError(f"Could not convert serial reading to float: {raw_reading!r}") from exc

            if sample_delay > 0.0 and index < sample_count - 1:
                time.sleep(sample_delay)

    if not values:
        raise ValueError("No valid serial readings were received")

    return sum(values) / len(values)


def main():
    while True:
        x_displacement_mm = 10.0
        y_step_mm = 0.5
        sample_count = x_axis_sample_count(x_displacement_mm, y_step_mm)
        reading = read_serial_float(
            x_displacement_mm=x_displacement_mm,
            y_step_mm=y_step_mm,
            sample_delay=0.05,
        )
        print(f"samples={sample_count}, reading={reading}")


if __name__ == "__main__":
    main()