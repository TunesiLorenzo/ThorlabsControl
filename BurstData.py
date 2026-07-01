import serial
import struct
import time
import matplotlib.pyplot as plt


def capture_stream(port='COM3', baud=128000, duration=1.0):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(2)  # allow Arduino reset

    ser.reset_input_buffer()

    start = time.time()
    samples = []

    while time.time() - start < duration:
        line = ser.readline().decode(errors='ignore').strip()

        if line:
            try:
                samples.append(float(line))
            except ValueError:
                pass  # ignore incomplete/corrupt lines

    ser.close()
    return samples


def plot(samples):
    plt.figure()
    plt.plot(samples)
    plt.title("ADC Stream")
    plt.xlabel("Sample index")
    plt.ylabel("Voltage / ADC value")
    plt.grid()
    plt.show()


samples = capture_stream(port='COM3', baud=128000, duration=2)

print("Samples received:", len(samples))
print("First samples:", samples[:10])

plot(samples)