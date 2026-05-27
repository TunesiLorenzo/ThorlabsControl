import serial
import time

# Configurazione seriale
PORT = "COM6"
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # attesa per stabilizzazione (utile per Arduino/MCU)
    # value = 2
    # cmd = f"DC 0 {value:.2f}\n"  # formato con 2 decimali
    # ser.write(cmd.encode('utf-8'))
    while 1:
        value = 2.3
        cmd = f"DC 0 {value:.2f}\n"   
        ser.write(cmd.encode('utf-8'))
        # print(f"Inviato: {cmd.strip()}")
        time.sleep(0.005)
        value = 0
        cmd = f"DC 0 {value:.2f}\n"   
        ser.write(cmd.encode('utf-8'))
        # print(f"Inviato: {cmd.strip()}")
        time.sleep(0.005)

    #     for i in range(0, 101):  # da 0 a 100 inclusi
    #         value = i / 100.0
    #         cmd = f"DC 0 {value:.2f}\n"  # formato con 2 decimali

    #         ser.write(cmd.encode('utf-8'))
    #         print(f"Inviato: {cmd.strip()}")

    #         time.sleep(0.001)

    ser.close()
    print("Comunicazione terminata.")

except serial.SerialException as e:
    print(f"Errore seriale: {e}")