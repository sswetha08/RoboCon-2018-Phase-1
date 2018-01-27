import serial
from time import sleep

ser = serial.Serial('COM3', 9600, timeout=1)
print(ser. name)
ser.write(b'\nThis is Avneesh Mishra')
ser.close()

