import time
from math import ceil

import serial


# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = '/dev/ttyUSB0'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def read_serial_float(
    port: str = SERIAL_PORT,
    rate: int = SERIAL_RATE,
    sample_count: int = 1000,
    sample_delay: float = 0.0,
):
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
        reading = read_serial_float(
            sample_count=1000,
            sample_delay=0.003,
        )
        print(f"samples=1000, reading={reading}")


if __name__ == "__main__":
    main()