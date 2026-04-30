import serial


# this port address is for the serial tx/rx pins on the GPIO header
SERIAL_PORT = '/dev/ttyUSB0'
# be sure to set this to the same rate used on the Arduino
SERIAL_RATE = 115200


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)
    try:
        while True:
            # using ser.readline() assumes each line contains a single reading
            # sent using Serial.println() on the Arduino
            reading = get_reading(ser)
            # reading is a string...do whatever you want from here
            print(reading)
    except KeyboardInterrupt:
        ser.close()


def get_reading(ser=None):
    """Read a single line from the serial port and return it as a string.

    If `ser` is provided it will be reused; otherwise a temporary serial
    connection is opened and closed for the read.
    Returns None on timeout/empty read.
    """
    close_after = False
    if ser is None:
        ser = serial.Serial(SERIAL_PORT, SERIAL_RATE, timeout=1)
        close_after = True

    raw = ser.readline()
    if close_after:
        ser.close()

    if not raw:
        return None
    return raw.decode('utf-8').rstrip('\r\n')


if __name__ == "__main__":
    main()