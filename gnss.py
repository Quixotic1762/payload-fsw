'''
Author: Ashwin Kumar
'''

import serial
import csv

ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)

def decimal(coord, direction):
    degrees = int(coord[:2])
    minutes = float(coord[2:])
    decimal = degrees + (minutes / 60)
    if direction in ["S", "W"]:
        decimal *= -1
    return decimal

def decimal_long(coord, direction):
    degrees = int(coord[:3])
    minutes = float(coord[3:])
    
    decimal = degrees + (minutes / 60)
    if direction in ["S", "W"]:
        decimal *= -1
    return decimal
    #print(decimal)

with open("/home/pi5p/Integrated /logs/gnss.txt", "a", newline="") as gnss_fd:
    writer = csv.writer(gnss_fd)
    writer.writerow(["UTC", "Latitude", "Longitude", "Altitude"])
    
    while True:
        raw = ser.readline().decode("utf-8", errors='ignore').strip()
    #print(raw)
        if raw.startswith("$GNGGA"):
            f_raw = raw.split(",")
            altitude = f_raw[9]
        
        if raw.startswith("$GNRMC"): 
            f_raw = raw.split(",")
    
            lat = f_raw[3]
            lat_dir = f_raw[4]
            dec_lat = decimal(lat, lat_dir)
            long = f_raw[5]
            long_dir = f_raw[6]
            dec_long = decimal_long(long, long_dir)
            writer.writerow([f_raw[1],dec_lat, dec_long, altitude])
            print(f"{dec_lat}, {dec_long}, {altitude}")
    

    

        
        