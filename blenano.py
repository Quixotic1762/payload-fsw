'''
Author: Ghanit Taunk, Pathik Raythana
'''

import serial
import time
import csv

ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
ser.reset_input_buffer()

def get_CRC(databytes):
    crc = 0
    for byte in databytes:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc ^= 0x07
            crc >>= 1
    return crc

reading = False
buffer = ""

def read_function():
    global reading, buffer  # Tell Python we're using the global variables
    with open ('/home/pi5p/Integrated /logs/nano_ble.txt', "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Current Time", 'temperature', 'roll', 'pitch', 'yaw', 'pressure', 'altitude', 'voltage'])
        while True:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    
            for char in data:
                if char == '#':
                    reading = True
                    buffer = ""
                elif char == '$' and reading:  # Match your Arduino code (you used '$' not '&')
                    reading = False
                    try:
                        inner = buffer  # Already between # and $
                        payload_str, crc_str = inner.rsplit(",", 1)
                        payload_bytes = payload_str.encode('utf-8')
                        calculated_crc = get_CRC(payload_bytes)
                        #crc_str = crc_str[:4]
                        received_crc = int(crc_str)

                        if calculated_crc == received_crc:
                            writer.writerow(payload_str.rsplit(','))
                            print(payload_str)
                        else:
                            print(f" CRC mismatch Received: {received_crc}, Calculated: {calculated_crc}")
                    except Exception as e:
                        print("Error while parsing: ", e)

                elif reading:
                    buffer += char
read_function()

