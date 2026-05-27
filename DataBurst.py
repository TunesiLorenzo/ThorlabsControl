import serial
import struct
import time
import matplotlib.pyplot as plt


def capture_stream(port='COM3', baud=230400, duration=1.2):
    ser = serial.Serial(port, baud, timeout=0)
    time.sleep(2)  # allow Arduino reset

    ser.reset_input_buffer()
    ser.write(b's')  # trigger capture

    start = time.time()
    raw = bytearray()

    # Read as fast as possible for fixed duration
    while time.time() - start < duration:
        raw.extend(ser.read(ser.in_waiting or 1))

    ser.close()

    # Ensure even number of bytes (uint16)
    raw = raw[:len(raw) - (len(raw) % 2)]

    # Convert to uint16 samples
    samples = struct.unpack('<' + 'H' * (len(raw) // 2), raw)

    return samples


def plot(samples):
    plt.figure()
    plt.plot(samples)
    plt.title("ADC Stream (1 second capture)")
    plt.xlabel("Sample index")
    plt.ylabel("ADC value")
    plt.grid()
    plt.show()


# --- RUN ---
samples = capture_stream(port='COM5', baud=460800, duration=1)

print("Samples received:", len(samples))
plot(samples)