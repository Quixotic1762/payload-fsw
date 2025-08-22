from multiprocessing import Process

def main():
    gnss_proc = Process(target=gnss_proc)
    blenano_proc = Process(target=blenano_proc)

def gnss_proc(tel_q):
    import serial
    import csv

    ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)

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
    
    with open("/home/pi5p/integratedrpi/logs/gnss.txt", "a", newline="") as gnss_fd:
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
                tel_q.put([f_raw[1],dec_lat, dec_long, altitude])
                print(f"{dec_lat}, {dec_long}, {altitude}")

def blenano_proc():
    import serial
    import time
    import csv
    #ser = serial.Serial('/dev/ttyAMA4', 9600, timeout=1)
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
        with open ('/home/pi5p/integratedrpi/logs/nano_ble.txt', "a", newline="") as file:
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
                                q.put(payload_str)
                            else:
                                print(f" CRC mismatch Received: {received_crc}, Calculated: {calculated_crc}")
                        except Exception as e:
                            print("Error while parsing: ", e)
    
                    elif reading:
                        buffer += char
    read_function()

'''
def hackrf_procs():

def lora_proc():

def state_machine_proc():

def gsm_proc():
'''

if __name__ == '__main__':
    main()