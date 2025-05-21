import serial
import time

# Change COM3 to the correct port for your setup
ser = serial.Serial('COM5', baudrate=115200, timeout=0.01)
time.sleep(3.5)

# Send some data
ser.write(b'hello\n')

# Read some data
response = ser.readline()
print("Received:", response)

# Don't forget to close when done
ser.close()

# while 1 == 1:
# 	hello = 1
