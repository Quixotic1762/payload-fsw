import subprocess

blenano_proc_log = 'proc_files/blenano_proc_log'
blevsas_log = 'proc_files/blevsas_log'
gnss_proc_log = 'proc_files/gnss_proc_log'
hackrf_log = 'proc_files/hackrf_log'
telemetry_log = 'proc_files/telemetry_log'

def lora(telm_q, tx_enable):
    import sys
    import time
    import argparse
    from datetime import datetime
    import subprocess
    from lora_rpi5_interface import LoRa

    '''
    sends telemetry recieved via queue telm_q
    send_mode(lora, telm_str): sends telm_str
    '''
    def send(lora):
        ''' send msg -> if retval = 1 -> send again'''
        message = telm_q.get()
        return lora.send(message.encode())

    def main():
        try:
            '''
            lora = LoRa(
                frequency=args.freq,
                bandwidth=args.bw,
                spreading_factor=args.sf,
                coding_rate=args.cr,
                tx_power=args.power,
                verbose=args.debug
            )
            '''
            lora = LoRa()
            while True:
                enable = tx_enable.is_set()
                if (telm_q.qsize() > 0) and enable:
                    send(lora)
                        
                payload, rssi = lora.receive(timeout=1000)
                if payload:
                    if payload == 'ack':
                        tx_enable.set()
    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'lora' in locals():
                lora.close()
                # print("LoRa resources released.")
    
    #if __name__ == "__main__":
    main()

def hackrf_proc():
    import subprocess
    import csv
    import datetime
    import time
    
    def run_hackrf_sweep():
        with open(hackrf_log, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Frequency_MHz", "RSSI_dBm"])

            process = subprocess.Popen(["hackrf_sweep", "-f", "700:2700"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            for line in process.stdout:
                parts = line.strip().split(",")
                if len(parts) >= 7: 
                    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    try:
                        hz_low = int(parts[2])
                        hz_high = int(parts[3])
                        bin_width = float(parts[4])
                        rssi_values = parts[6:]

                        for i, rssi in enumerate(rssi_values):
                            freq_mhz = (hz_low + i * bin_width) / 1e6
                            writer.writerow([timestamp, freq_mhz, float(rssi)])
                            file.flush()
                            #print(f"{timestamp} | {freq_mhz:.2f} MHz | {rssi} dBm")
                        time.sleep(5)

                    except ValueError:
                        continue

    run_hackrf_sweep()

def gnss_proc():
    import serial
    import csv

    ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
    gnss_proc_log = 'proc_files/gnss_proc_log'

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
    
    with open(gnss_proc_log, "a", newline="") as gnss_fd:
        writer = csv.writer(gnss_fd)
        writer.writerow(["UTC", "Latitude", "Longitude", "Altitude"])
    
        while True:
            raw = ser.readline().decode("utf-8", errors='ignore').strip()
        #print(raw)
        
            if raw.startswith("$GNGGA"):
                f_raw = raw.split(",")
                if len(f_raw) > 9 and f_raw[9]:
                    try:
                        altitude = float(f_raw[9])
                    except ValueError:
                        altitude = None

            elif raw.startswith("$GNRMC"): 
                f_raw = raw.split(",")
                if len(f_raw) > 6:
                    lat = f_raw[3]
                    lat_dir = f_raw[4]
                    dec_lat = decimal(lat, lat_dir)
                    long = f_raw[5]
                    long_dir = f_raw[6]
                    dec_long = decimal_long(long, long_dir)
                writer.writerow([f_raw[1],dec_lat, dec_long, altitude])

def blenano_proc():
    import serial
    import csv
    import os
    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    ser.reset_input_buffer()

    ble_file = "proc_files/blenano_proc_log"
    volt_file = "proc_files/blevsas_log"
    
    with open(ble_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp", "Temperature", "Roll", "Pitch", "Yaw",
            "Pressure", "Altitude", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz"
        ])

    with open(volt_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Voltage"])

    buffer = ""
    while True:
        try:
            data = ser.read(ser.in_waiting or 1).decode("utf-8", errors="ignore").strip()
            buffer += data

            while "<" in buffer and ">" in buffer:
                start = buffer.find("<")
                end = buffer.find(">", start)
                if end == -1:
                    break  

                packet = buffer[start+1:end]  
                buffer = buffer[end+1:]  

                parts = packet.split(",")
                if not parts:
                    continue

                if parts[0] == "V" and len(parts) == 3:
                    _, ts, volt = parts
                    with open(volt_file, "a", newline="") as vf:
                        writer = csv.writer(vf)
                        writer.writerow([ts, volt])

                elif parts[0] == "S" and len(parts) == 17:
                    _, *sensor_data = parts
                    with open(ble_file, "a", newline="") as sf:
                        writer = csv.writer(sf)
                        writer.writerow(sensor_data)

        except Exception as e:
            print("Error reading serial data:", e)

def getline(proc_fp):
    line = subprocess.check_output(['tail','-n','1',proc_fp])
    return line.decode().split('\n')[0]