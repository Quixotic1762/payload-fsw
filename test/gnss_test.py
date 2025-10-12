'''
Author: Ashwin Kumar, Ghanit Taunk
'''
import subprocess
import time
def gnss_proc():
    import serial
    import csv

    ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
    gnss_proc_log = 'test_logs/gnss_proc_log'

    def decimal(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:2])
            minutes = float(coord[2:])
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None
        
    def decimal_long(coord, direction):
        if not coord:
            return None
        try:
            degrees = int(coord[:3])
            minutes = float(coord[3:])
    
            decimal = degrees + (minutes / 60)
            if direction in ["S", "W"]:
                decimal *= -1
            return decimal
        except ValueError:
            return None 
    #print(decimal)

    altitude = None
    num_sats = None
    
    with open(gnss_proc_log, "w", newline="") as gnss_fd:
        writer = csv.writer(gnss_fd)
        writer.writerow(["UTC","Latitude","Longitude","Sats","Altitude"])
        
        while True:
            raw = ser.readline().decode("utf-8", errors='ignore').strip()
            #print(raw)
            
            if raw.startswith("$GNGGA"):
                f_raw = raw.split(",")
                if len(f_raw) > 9:
                    if f_raw[9]:
                        try:
                            altitude = float(f_raw[9])
                        except ValueError:
                            altitude = None
                    if f_raw[7]:
                        try:
                            num_sats = int(f_raw[7])
                        except ValueError:
                            num_sats = None


            if raw.startswith("$GNRMC"): 
                f_raw = raw.split(",")
                if len(f_raw) > 6:
                    lat = f_raw[3]
                    lat_dir = f_raw[4]
                    dec_lat = decimal(lat, lat_dir)
                    long = f_raw[5]
                    long_dir = f_raw[6]
                    dec_long = decimal_long(long, long_dir)
                writer.writerow([f_raw[1],dec_lat, dec_long, num_sats, altitude])
                gnss_fd.flush()
                print(f"Lat: {dec_lat}, Long: {dec_long}, Sat: {num_sats}, Alt: {altitude}")
                line = getline('test_logs/gnss_proc_log')

def getline(proc_fp):
    line = subprocess.check_output(['tail','-n','1',proc_fp])
    return line.decode().split('\n')[0]


gnss_proc()

