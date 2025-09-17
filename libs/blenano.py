'''
Author: Ghanit Taunk, Pathik Raythana
'''

def blenano_proc():
    import serial
    import time
    import csv
    import os

    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    ser.reset_input_buffer()

    ble_file = '/home/pi5/Documents/nano_ble.csv'
    volt_file = '/home/pi5/Documents/voltage.csv'

    if not os.path.exists(ble_file):
        with open(ble_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Temperature", "Roll", "Pitch", "Yaw", "Pressure", "Altitude"])

    if not os.path.exists(volt_file):
        with open(volt_file, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Voltage"])
            
    with open(ble_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Temperature", "Roll", "Pitch", "Yaw", "Pressure", "Altitude"])
    with open(volt_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Voltage"])

    def read_function():
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith("V"):
                    voltage_value = line[2:]
                    with open(volt_file, "a", newline="") as vf:
                        writer = csv.writer(vf)
                        writer.writerow([time.time(), voltage_value])
                    #print("Voltage:", voltage_value)

                elif line.startswith("S"):
                    sensor_data = line[2:].split(",")
                    with open(ble_file, "a", newline="") as sf:
                        writer = csv.writer(sf)
                        writer.writerow(sensor_data)
                   # print("Sensor:", sensor_data)
                    #q.put(sensor_data)

            except Exception as e:
                print("Error reading serial data:", e)

    read_function()

    '''
    def blenano_proc():
        import serial
        import time
        import csv
        import os

        ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
        ser.reset_input_buffer()

        ble_file = 'proc_files/blenano_proc_log'
        volt_file = 'proc_files/blevsas_log'
        ## Changed this part
        if (os.path.getsize(ble_file) == 0):
            with open(ble_file, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp", "Temperature", "Roll", "Pitch", "Yaw", "Pressure", "Altitude"])
        ## Changed this part
        if (os.path.getsize(volt_file) == 0):
            with open(volt_file, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["Timestamp", "Voltage"])

        def read_function():
            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    if line.startswith("V"):
                        voltage_value = line[2:]
                        with open(volt_file, "a", newline="") as vf:
                            writer = csv.writer(vf)
                            writer.writerow([voltage_value])
                        #print("Voltage:", voltage_value)

                    elif line.startswith("S"):
                        sensor_data = line[2:].split(",")
                        with open(ble_file, "a", newline="") as sf:
                            writer = csv.writer(sf)
                            writer.writerow(sensor_data)
                        #print("Sensor:", sensor_data)

                except Exception as e:
                    print("Error reading serial data:", e)

        read_function()

    '''