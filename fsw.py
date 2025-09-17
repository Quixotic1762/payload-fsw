'''
Author: Ashwin Kumar
'''

'''
gnss_proc_log = <filepath>
blenano_proc_log = <filepath>
blevsas_log = <filepath>
telemetry_log = <filepath>
hackrf_log = <filepath>

Telemetry Format
#,<0TEAM_ID>,<1MISSION_TIME>,<2PACKET_COUNT>,<3TEMP>,<4PRESSURE>,<5FREQ>,<6RSSI>,<7ROLL>,<8PITCH>,<9YAW>,<10GNSS_ALT>,<11GNSS_LAT>,<12GNSS_LONG>,<13GNSS_SAT>,<14VOLTAGE>,<15CURRENT>,<16STATE>,<17CHECKSUM>,$
'''

'''
PAYLOAD:
#<TEAM_ID> <MISSION_TIME> <PACKET_COUNT> <BLE_TEMP><BLE_PRESSURE> <BLE_ALT> <HACKRF_FREQ> <HACKRF_RSSI> <AX> <AY> <AZ> <GX> <GY> <GZ> <MX><MY><MZ> <SOFTWARE_STATE> <GNSS_LAT> <GNSS_LONG> <GNSS_ALT> <GNSS_TIME> <GNSS_SATS> <VOLTAGE> <CURRENT> <RPI_TEMP> <LORA_RSSI><ERROR_FLAGS><CHECKSUM><ACK>$\r\n
'''

from multiprocessing import Process, Value, Queue, Event
import subprocess
import os
import time
import zlib

mission_timer_start = 0

state = {
    0: "BOOT",
    1: "IDLE",
    2: "ASCENT",
    3: "DESCENT",
    4: "RECOVERY"
    }

# File Paths For the process logs 
blenano_proc_log = 'proc_files/blenano_proc_log'
blevsas_log = 'proc_files/blevsas_log'
gnss_proc_log = 'proc_files/gnss_proc_log'
hackrf_log = 'proc_files/hackrf_log'
telemetry_log = 'proc_files/telemetry_log'

MIN_ASCENT_ALTITUDE = 30

class state_change_parameters:
    last_check = 0
    last_altitude = 0
    last_acceleration = 0
    altitude_flag = [0,0]
    acceleration_flag = [0,0]
    min_altitude_flag = 0
    flag_sum = 0
    change_state = False

class recovery_change_parameters():
    last_check = 0 
    last_altitude = 0
    alt_flag = [0,0]
    velocity_flag = [0,0]
    last_vel_time = 0
    velocity = 0
    flag_sum = 0
    change_state = False


#ascent_conditions = state_change_parameters()

#descent_change = state_change_parameters()

#recovery_change = state_change_parameters()

state_change = state_change_parameters()

recovery_change = recovery_change_parameters()

def main(global_state, global_packet_count):
    boot     =  0
    idle     =  1
    ascent   =  2
    descent  =  3
    recovery =  4

    #gnss_proc = Process(target=gnss_proc)
    #blenano_proc = Process(target=blenano_proc)
    #ackrf_proc = Process(target=hackrf_proc)
    
    while True:
        time.sleep(0.1)
        state = global_state.value

        if (state == boot):
            print("Boot state")
            state_restore = file_manager(global_state)

            if (state_restore == 0):
                state = global_state.value
                lora_proc.start()
                tx_enable.set()
            
            if (state_restore == 1):
                global_state.value = idle
                lora_proc.start()
                state = global_state.value

            telemetry_log_fd = open(telemetry_log, "a", newline="")
            
        if (state == idle):
            print("Idle state")
            global mission_timer_start
            '''' this should happen once and then not again.'''
            mission_timer_start = time.time()
            
            telm_string = generate_telemetry(global_packet_count, global_state)
            telemetry_log_fd.write(telm_string)

            if not tx_enable.is_set():
                tx_enable.wait()
            telm_q.put(telm_string)
            ''''
            --> Wait for Ack -> Start Transmiting Telemetry Stringacceleration
            LoRa_proc is running in recieve mode:
                two options -> wait for ACK
                            -> create some kind of signal using os/multiprocessing.event
            '''

            '''
            state change parameter from ascent to descent V, A, H must change for more than 3.
            '''
            idle_to_ascent_change()
            if state_change.change_state == True:
                global_state.value = ascent
                state_change.change_state = False
    
        if (state == ascent):
            print("ascent state")
            '''
            telemetry storage
            --> LoRa in reception mode, switch to transmit mode and transmit telemetry
            --> if receieved something process it
            '''
            telm_string_ascent = generate_telemetry(global_packet_count, global_state)
            telemetry_log_fd.write(telm_string_ascent)
            # telm_string_ascent -> lora_proc -> transmission to ground
            '''
            1.5s -> 10m
            2s -> 18m
            5s -> 112m
            state change parameter ->
                -> v < 0
                -> constant acceleration in y (g downward) 
                -> altitude decreasing
                -> 2 seconds
                -> state change DESCENT.
            '''
            descent_change_check()
            if state_change.change_state == True:
                global_state.value = descent
        
        if (state == descent):
            print("descent state")
            '''
            check altitude if between 490 and 510 open parachute. 
            '''
            telm_string_descent = generate_telemetry(global_packet_count, global_state)
            telemetry_log_fd.write(telm_string_descent)
            '''
            signal ble to actuate fins, uart.
            '''
            '''
            state change parameters: 
                -> altitude below 10m and constant.
                -> velocity is zero
                -> constant acceleration
                -> for 2s   
            '''
            recovery_change_check()
            if recovery_change.change_state == True:
                global_state.value = recovery
        if (state == recovery):
            print("recovery")
            '''
            terminate lora proc
            terminate ble proc
            get lat long from gnss
            invoke gsm_proc
            '''
            gnss = getline(gnss_proc_log)
            gnss_arr = gnss.split(',')
            gnss_lat = gnss_arr[1]
            gnss_long = gnss_arr[2]
            sms_payload = f"{gnss_lat}, {gnss_long}"
            '''
            -> transmit sms_payload
            -> Trigger audio beacons
            '''
def cpy_src():
    file = open("src_log", "r", newline='')
    ble_fd = open('proc_files/blenano_proc_log', "w", newline='')
    while True:
        line = file.readline()
        ble_fd.write(line)
        ble_fd.flush()
        time.sleep(0.05)

def idle_to_ascent_change():
    global ascent_conditions
    global MIN_ASCENT_ALTITUDE

    current_check_time = time.time()
    ble = getline(blenano_proc_log)
    ble_arr = ble.split(',')

    current_altitude = float(ble_arr[6])
    current_acceleration = float(ble_arr[9])

    delta_time = current_check_time - state_change.last_check
    delta_altitude = current_altitude - state_change.last_altitude
    delta_acceleration = current_acceleration - state_change.last_acceleration

    if delta_time < 2:
        if delta_altitude > 0:
            state_change.altitude_flag[0] = 1
        else:
            state_change.altitude_flag[1] += 1

        if delta_acceleration > 0:
            state_change.acceleration_flag[0] = 1
        else:
            state_change.acceleration_flag[1] += 1
        
        if current_altitude > MIN_ASCENT_ALTITUDE:
            state_change.min_altitude_flag = 1
        else:
            state_change.min_altitude_flag = 0
    else:
        if (state_change.altitude_flag[1] == 0):
            state_change.flag_sum += state_change.altitude_flag[0]

        if (state_change.acceleration_flag[1] == 0):
            state_change.flag_sum += state_change. acceleration_flag[0]

        state_change.flag_sum += state_change.min_altitude_flag
        state_change.altitude_flag[1] = 0
        state_change.acceleration_flag[1] = 0
        #print(f"flag sum: {state_change.flag_sum}")

        if state_change.flag_sum >= 2:
            #print(f"flags met {state_change.flag_sum}")
            state_change.change_state = True
        state_change.last_check = current_check_time

    state_change.last_altitude = current_altitude
    state_change.last_acceleration = current_acceleration
    state_change.flag_sum = 0

def descent_change_check():
    global descent_change
    current_check_time = time.time()
    ble = getline(blenano_proc_log)
    ble_arr = ble.split(',')

    current_altitude = float(ble_arr[6])
    current_acceleration = float(ble_arr[9])

    delta_time = current_check_time - state_change.last_check
    delta_altitude = current_altitude - state_change.last_altitude
    delta_acceleration = current_acceleration - state_change.last_acceleration

    if delta_time < 2:
        if delta_altitude < 0:
            state_change.altitude_flag[0] = 1
        else:
            state_change.altitude_flag[1] += 1

        if delta_acceleration < 0:
            state_change.acceleration_flag[0] = 1
        else:
            state_change.acceleration_flag[1] += 1
    else:
        if state_change.altitude_flag[1] == 0:
            state_change.flag_sum += state_change.altitude_flag[0]
        if state_change.acceleration_flag[1] == 0:
            state_change.flag_sum += state_change.acceleration_flag[0]
        state_change.altitude_flag[1] = 0
        state_change.acceleration_flag[1] = 0

        if state_change.flag_sum >= 1:
            state_change.change_state = True
        state_change.last_check = current_check_time
    
    state_change.last_altitude = current_altitude
    state_change.last_acceleration = current_acceleration
    state_change.flag_sum = 0

def recovery_change_check():
    global recovery_change

    current_check_time = time.time()
    ble = getline(blenano_proc_log)
    ble_arr = ble.split(',')

    current_altitude = float(ble_arr[6])

    delta_time = float(current_check_time - recovery_change.last_check)
    delta_vel_time = float(current_check_time - recovery_change.last_vel_time)
    delta_altitude = current_altitude - recovery_change.last_altitude
    velocity = delta_altitude / delta_vel_time
    recovery_change.last_vel_time = current_check_time
    if delta_time < 2:
        if current_altitude < 10:
            recovery_change.alt_flag[0] = 1
        else:
            recovery_change.alt_flag[1] += 1
        if velocity < 5:
            recovery_change.velocity_flag[0] = 1
        else:
            recovery_change.velocity_flag[1] += 1
    else:
        if recovery_change.alt_flag[1] == 0:
            recovery_change.flag_sum += recovery_change.alt_flag[0]
        if recovery_change.velocity_flag[1] == 0:
            recovery_change.flag_sum += recovery_change.velocity_flag[0]

        recovery_change.alt_flag = [0,0]
        recovery_change.velocity_flag = [0,0]

        if recovery_change.flag_sum == 2:
            recovery_change.change_state = True
        recovery_change.last_check = current_check_time
    
    recovery_change.last_altitude = current_altitude
    recovery_change.flag_sum = 0

def file_manager(global_state):
    global state
    global telemetry_log
    working_dir = os.getcwd()
    working_dir = f"{working_dir}/telemetry_logs"
    dir_list = sorted(os.listdir('telemetry_logs/'))

    last_file = dir_list[-1]
    file_path = f"{working_dir}/{last_file}"
    
    last_line = getline(file_path)
    last_line_arr = last_line.split(",")

    try:
        current_state = [key for key, val in state.items() if val == last_line_arr[16]][0]
    except IndexError:
        return 0

    if (current_state == 4):
        new_file_split = last_file.split('_')
        file_num = int(new_file_split[2])
        file_num += 1
        new_file_split[2] = str(file_num)
        new_file = '_'.join(new_file_split)
        new_file_path = f"{working_dir}/{new_file}"
        file = open(new_file_path, "w", newline='')
        file.close()
        telemetry_log = new_file_path
        return 1
    
    global_state.value = current_state
    telemetry_log = file_path
    return 0
    
def generate_telemetry(global_packet_count, global_state):
    team_id = 'ASI-ROCKETRY-050'
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
    gnss_lat = gnss_arr[1]
    gnss_long = gnss_arr[2]
    gnss_alt = gnss_arr[3]
    gnss_sat = gnss_arr[4]

    vsas_volt = getline(blevsas_log)
    vsas_volt_list = vsas_volt.split(',')
    voltage = vsas_volt_list[1]

    hackrf_line = getline(hackrf_log)
    hackrf_arr = hackrf_line.split(',')
    freq = hackrf_arr[1]
    rssi = hackrf_arr[2]

    current = -32 # add code to calculate it 
    state = global_state.value

    packet_count = global_packet_count.value

    telm_str = f"{team_id}, {mission_time}, {packet_count}, {temp}, {pressure}, {freq}, {rssi}, {roll}, {pitch}, {yaw}, {gnss_alt}, {gnss_lat}, {gnss_long}, {gnss_sat}, {voltage}, {current}, {state}"
    checksum = zlib.crc32(telm_str.encode())
    telm_str = f"{telm_str}, {checksum}\n"

    return telm_str

def getline(proc_fp):
    line = subprocess.check_output(['tail','-n','1',proc_fp])
    return line.decode()


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
    import csv
    import serial

    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    ser.reset_input_buffer()

    ble_file = blenano_proc
    volt_file = blevsas_log

    
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

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Voltage
            if line.startswith("<V,") and line.endswith(">"):
                parts = line[1:-1].split(",")
                if len(parts) == 3:
                    _, ts, volt = parts
                    with open(volt_file, "a", newline="") as vf:
                        writer = csv.writer(vf)
                        writer.writerow([ts, volt])

            # Sensor
            elif line.startswith("<S,") and line.endswith(">"):
                parts = line[1:-1].split(",")
                if len(parts) == 17:  
                    _, *sensor_data = parts
                    with open(ble_file, "a", newline="") as sf:
                        writer = csv.writer(sf)
                        writer.writerow(sensor_data)
                    #print("Sensor:", sensor_data)

        except Exception as e:
            print("Error reading serial data:", e)
'''
def telecom_parse_proc(telecommand):

def gsm_proc():
'''

if __name__ == '__main__':
    global_state = Value("i", 0)
    global_packet_count = Value("i", 0)
    telm_q = Queue()
    tx_enable = Event()
    p1 = Process(target=cpy_src)
    p1.start()
    lora_proc = Process(target=lora, args=(telm_q,tx_enable,))
    time.sleep(0.15)
    main(global_state, global_packet_count)
    #file_manager(global_state)
    #print(telemetry_log)
    #telm = generate_telemetry(global_packet_count, global_state)
    #print(telm, end="")
    #print(global_state.value)
    #main(state)+
