import serial
import time
import struct

s = serial.Serial("/dev/stm32")

while True:
    s.write(struct.pack("fff", 3.14, 2.71, 3.14))
    s.write(b"\r\n")
    time.sleep(0.1)

