import serial
import time

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

while True:
    time.sleep(2)

    ser.write(b"SA:135\n")

    time.sleep(2)

    ser.write(b"SA:0\n")

    time.sleep(2)

    ser.write(b"SA:270\n")