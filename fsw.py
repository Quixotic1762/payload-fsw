'''
Author: Ashwin Kumar
'''

from multiprocessing import Process
import subprocess
import os
import time
# import csv

'''
gnss_proc_log = <filepath>
blenano_proc_log = <filepath>
blevsas_log = <filepath>
telemetry_log = <filepath>
hackrf_log = <filepath>

#,<0TEAM_ID>,<1MISSION_TIME>,<2PACKET_COUNT>,<3TEMP>,<4PRESSURE>,<5FREQ>,<6RSSI>,<7ROLL>,<8PITCH>,<9YAW>,<10GNSS_ALT>,<11GNSS_LAT>,<12GNSS_LONG>,<13GNSS_SAT>,<14VOLTAGE>,<15CURRENT>,<16STATE>,<17CHECKSUM>,$
'''

mission_timer_start = 0

blenano_proc_log = 'payload_fsw/proc_files/blenano_proc_log'
blevsas_log = 'payload_fsw/proc_files/blevsas_log'
gnss_proc_log = 'payload_fsw/proc_files/gnss_proc_log'
hackrf_log = 'payload_fsw/proc_files/hackrf_log'

def main():
    state    =  int(0)
    boot     =  int(0) 
    idle     =  int(1)
    ascent   =  int(2)
    descent  =  int(3)
    recovery =  int(4)

    while True:
        
        if (state == boot):
            gnss_proc = Process(target=gnss_proc)
            blenano_proc = Process(target=blenano_proc)
            hackrf_proc = Process(target=hackrf_proc)
            # lora_proc = Process(target=lora_proc) in recieve mode

            if (os.path.getsize(telemetry_log) > 0):
                last_tel = getline(telemetry_log)
                last_tel_arr = last_tel.split(',')[1:19]
                state = last_tel_arr[16]
        if (state == idle):
            mission_timer_start = time.time()
            ## polling of sensors -> read from files?
            ## generate telemetry strin
            ## wait for ack?
            telm_str = generate_telemetry()
            ## save telm_str?
            '''
            initial idea -> addd in write mode if file empty otherwise do append
            but the file will be empty by default and will have content if theres a power break so seems redundant
            -> think about a better design
            
            if (os.path.getsize(telemetry_log) > 0):
                with open(telemetry_log, 'a', newline='') as telemetry_fd:
                    telemetry_writer = csv.writer(telemetry_fd)
                    telemetry_writer.write(telm_str)
            else:
                with open(telemetry_log, 'w', newline='') as telemetry_fd:
                    telemetry_writer = csv.writer(telemetry_fd)
                    telemetry_writer.writerow(['#','<TEAM_ID>','<MISSION_TIME>','<PACKET_COUNT>','<TEMP>','<PRESSURE>','<FREQ>','<RSSI>','<ROLL>','<PITCH>','<YAW>','<GNSS_ALT>','<GNSS_LAT>','<GNSS_LONG>','<GNSS_SAT>','<VOLTAGE>','<CURRENT>','<STATE>','<CHECKSUM>','$'])
                    telemetry_writer.write(telm_str)
                '''
    if (state == ascent):
        #velocity decrease -> state change for 1 sec

'''
#,<0TEAM_ID>,<1MISSION_TIME>,<2PACKET_COUNT>,<3TEMP>,<4PRESSURE>,<5FREQ>,<6RSSI>,<7ROLL>,<8PITCH>,<9YAW>,<10GNSS_ALT>,<11GNSS_LAT>,<12GNSS_LONG>,<13GNSS_SAT>,<14VOLTAGE>,<15CURRENT>,<16STATE>,<17CHECKSUM>,$
'''
def generate_telemetry(counter):
    team_id = '050'
    mission_time = int(time.time() - mission_timer_start)

    ble = getline(blenano_proc_log)
    ble_arr = ble.split(',')
    temp = ble_arr[1]
    roll = ble_arr[2]
    pitch = ble_arr[3]
    yaw = ble_arr[4]
    pressure = ble_arr[5]

    gnss = getline(gnss_proc_log)
    gnss_arr = gnss.split(',')
    gnss_alt = gnss_arr[3]
    gnss_lat = gnss_arr[1]
    gnss_long = gnss_arr[2]
    gnss_sat = 

    vsas_volt = getline(blevsas_log)
    vsas_volt_list = vsas_volt.split(',')
    voltage = vsas_volt_list[1]


    hackrf_line = getline(hackrf_log)
    hackrf_arr = hackrf_line.split(',')
    freq = hackrf_arr[1]
    rssi = hackrf_arr[2]

    #current
    

    telm_str = f"{team_id}, {mission_time}, {counter}, {temp}, {pressure}, {freq}, {rssi}, {roll}, {pitch}, {yaw}, {gnss_alt}, {gnss_lat}, {gnss_long}, {gnss_sat}, {voltage}, "
    return telm_str

def gnss_proc(gnss_q):
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
                gnss_q.put([f_raw[1],dec_lat, dec_long, altitude])
                #print(f"{dec_lat}, {dec_long}, {altitude}")

def blenano_proc(nano_q):
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
                                nano_q.put(payload_str)
                            else:
                                print(f" CRC mismatch Received: {received_crc}, Calculated: {calculated_crc}")
                        except Exception as e:
                            print("Error while parsing: ", e)
    
                    elif reading:
                        buffer += char
    read_function()


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

# Update the Queue System to reading relevant files.
def lora_proc(nano_q, gnss_q):
    #!/usr/bin/env python3

    """
    LoRa Test Script for Raspberry Pi 5
    -----------------------------------
    This script tests basic functionality of the LoRa module.
    It supports both sending and receiving modes.

    Usage:
      python lora_test.py --mode send   # Send test messages every 5 seconds
      python lora_test.py --mode recv   # Continuously listen for messages
    
    Connections:
    - GPIO4 (pin 7)   -> RESET
    - GPIO17 (pin 11) -> DIO0
    - GPIO10 (pin 19) -> MOSI
    - GPIO9 (pin 21)  -> MISO
    - GPIO11 (pin 23) -> SCK
    - GPIO8 (pin 24)  -> NSS (CS)
    - Optional: GPIO23, GPIO24, GPIO25 -> DIO1, DIO2, DIO3
    """
    import sys
    import time
    import argparse
    from datetime import datetime
    import subprocess

    # Import the LoRa class from our main file
    # Make sure lora_rpi5_interface.py is in the same directory
    try:
        from lora_rpi5_interface import LoRa
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure lora_rpi5_interface.py is in the same directory as this script")
        sys.exit(1)

    def send_mode(lora, count=100):
        """Send test messages repeatedly"""
        counter = 0
        try:
            #while counter < count:
            while True:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                line = subprocess.check_output(['tail','-1',"/home/pi5/Documents/rssi_log.csv"])
                blenano_str = ','.join(nano_q.get()) # convert ble list to string
                gnss_str = ','.join(gnss_q.get()) # convert gnss list to string
                message = str(line)+ str(blenano_str)+str(counter)+str(gnss_str)

                #message = f"RPi5 LoRa Test #{counter} at {timestamp}"
                print(f"Sending: {message}")

                # Send the message
                lora.send(message.encode())
                print("Message sent!")

                counter += 1
                print(f"Waiting 5 seconds... ({counter}/{count})")
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("\nSending stopped by user.")
        finally:
            print(f"Sent {counter} messages.")

    def receive_mode(lora, timeout=None):
        """Listen for incoming messages"""
        message_count = 0
        start_time = time.time()

        try:
            print("Listening for LoRa messages... Press Ctrl+C to stop.")

            while True:
                if timeout and (time.time() - start_time > timeout):
                    print(f"Receive timeout after {timeout} seconds.")
                    break

                # Try to receive a packet (1 sec timeout per attempt)
                payload, rssi = lora.receive(timeout=1000)

                if payload:
                    message_count += 1
                    try:
                        decoded = payload.decode('utf-8')
                        print(f"\nMessage #{message_count} received:")
                        print(f"Data: {decoded}")
                        print(f"RSSI: {rssi} dBm")
                        print(f"Length: {len(payload)} bytes")
                    except UnicodeDecodeError:
                        print(f"\nBinary message received:")
                        print(f"Data (hex): {payload.hex()}")
                        print(f"RSSI: {rssi} dBm")
                        print(f"Length: {len(payload)} bytes")
                else:
                    # Print a dot to show we're still alive
                    sys.stdout.write(".")
                    sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nReceiving stopped by user.")
        finally:
            print(f"Received {message_count} messages.")

    def main():
        parser = argparse.ArgumentParser(description='Test LoRa functionality')
        parser.add_argument('--mode', choices=['send', 'recv'], required=True, 
                            help='Operation mode: send or receive')
        parser.add_argument('--freq', type=float, default=433.0,
                            help='Frequency in MHz (default: 433.0)')
        parser.add_argument('--bw', type=int, default=500000,
                            help='Bandwidth in Hz (default: 125000)')
        parser.add_argument('--sf', type=int, default=12,
                            help='Spreading Factor (default: 9)')
        parser.add_argument('--cr', type=int, default=5,
                            help='Coding Rate denominator (default: 5, which is 4/5)')
        parser.add_argument('--power', type=int, default=18,
                            help='TX Power in dBm (default: 17)')
        parser.add_argument('--count', type=int, default=100,
                            help='Number of messages to send (default: 100)')
        parser.add_argument('--timeout', type=int, default=None,
                            help='Timeout in seconds for receive mode (default: none/infinite)')
        parser.add_argument('--debug', action='store_true',
                            help='Enable debug mode with more verbose output')
        
        args = parser.parse_args()
        
        print("LoRa Test Application")
        print("=====================")
        print(f"Mode: {args.mode}")
        print(f"Frequency: {args.freq} MHz")
        print(f"Bandwidth: {args.bw} Hz")
        print(f"Spreading Factor: {args.sf}")
        print(f"Coding Rate: 4/{args.cr}")
        print(f"TX Power: {args.power} dBm")
        
        try:
            # Initialize LoRa
            print("\nInitializing LoRa module...")
            
            # Check SPI functionality first
            if args.debug:
                try:
                    import spidev
                    spi = spidev.SpiDev()
                    spi.open(0, 0)
                    print("SPI initialized successfully")
                    spi.close()
                except Exception as e:
                    print(f"SPI initialization failed: {e}")
                    print("Please check if SPI is enabled in raspi-config")
                    sys.exit(1)
            
            # Initialize LoRa
            lora = LoRa(
                frequency=args.freq,
                bandwidth=args.bw,
                spreading_factor=args.sf,
                coding_rate=args.cr,
                tx_power=args.power,
                verbose=args.debug
            )
            print("LoRa initialized successfully!")
            
            # Run selected mode
            if args.mode == 'send':
                send_mode(lora, args.count)
            else:  # 'recv'
                receive_mode(lora, args.timeout)
                
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if 'lora' in locals():
                lora.close()
                print("LoRa resources released.")
    
    #if __name__ == "__main__":
    main()

def gsm_proc():
    import serial
    import time

    ser = serial.Serial("/dev/ttyAMA2", 9600, timeout=1)
    def main():
        while True:
            send_sms('+918984955170', 'Helo from rpi and sim800')
            time.sleep(1)

    def send_at(command):
        tr_buffer = (command+'\r\n').encode(encoding="utf-8")
        ser.write((command+'\r\n').encode())
        while ser.readline():
            print(ser.readline().decode('utf-8'))

    def send_sms(number, message):
        send_at("AT+CMGF=1")
        set_nu = 'AT+CMGS="'+number+'"'
        send_at(set_nu)
        msg = message + chr(26)
        msg_bytes = msg.encode()
        ser.write(msg_bytes)    

    main()


def getline(proc_fp):
    line = subprocess.get_output(['tail','-n','1',proc_fp])
    return line.decode()


if __name__ == '__main__':
    main()