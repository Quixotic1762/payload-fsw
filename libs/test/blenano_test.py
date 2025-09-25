'''
Author: Ghanit Taunk, Pathik Raythana
'''

def nanoble():
    import serial
    import csv
    import os
    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    ser.reset_input_buffer()

    ble_file = 'test_logs/blenano_proc_log'
    volt_file = 'test_logs/blevsas_log'
    
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
                        print(f"{ts}, {volt}")

                elif parts[0] == "S" and len(parts) == 17:
                    _, *sensor_data = parts
                    with open(ble_file, "a", newline="") as sf:
                        writer = csv.writer(sf)
                        writer.writerow(sensor_data)
                        print(f"{sensor_data}")
00000000000000  
        except Exception as e:
            print("Error reading serial data:", e)
            # sys.exit(return code)
nanoble()