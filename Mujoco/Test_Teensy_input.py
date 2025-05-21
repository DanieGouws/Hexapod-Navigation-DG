import serial
import struct

ser = serial.Serial('COM5', baudrate=115200, timeout=0.01)

while True:
    # Wait for sync byte
    sync = ser.read(1)
    if sync != b'\xAB':
        continue

    # Read 18 floats for angles
    data = ser.read(4 * 18)
    if len(data) < 72:
        continue

    angles = struct.unpack('<18f', data)

    # Read 18 floats for speeds
    spd_data = ser.read(4 * 18)
    if len(spd_data) < 72:
        continue

    speeds = struct.unpack('<18f', spd_data)

    print("Angles (rad):", angles)
    print("Speeds (unit):", speeds)
