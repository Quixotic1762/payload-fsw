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
from state_dep import *
from fsw_dep import *

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
telemetry_log = 'telmetry_logs/telemetry_log'

def main(global_state, global_packet_count):
    mission_timer_flag = 1
    boot     =  0
    idle     =  1
    ascent   =  2
    descent  =  3
    recovery =  4

    global blenano_proc

    #gnss_proc = Process(target=gnss_proc)
    blenano_proc = Process(target=blenano_proc)
    #ackrf_proc = Process(target=hackrf_proc)
    lora_proc = Process(target=lora, args=(telm_q,tx_enable,))
    blenano_proc.start()
    
    while True:
        time.sleep(1)
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
            if mission_timer_flag:
                mission_timer_start = time.time()
                mission_timer_flag -= 1
            
            telm_string = generate_telemetry(global_packet_count, global_state)
            telemetry_log_fd.write(telm_string)
            ''''
            --> Wait for Ack -> Start Transmiting Telemetry Stringacceleration
            LoRa_proc is running in recieve mode:
                two options -> wait for ACK
                            -> create some kind of signal using os/multiprocessing.event
            '''
            if not tx_enable.is_set():
                tx_enable.wait()

            telm_q.put(telm_string)
            print(telm_string)

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

def file_manager(global_state):
    global state
    global telemetry_log
    working_dir = os.getcwd()
    working_dir = f"{working_dir}/telemetry_logs"
    dir_list = sorted(os.listdir('telemetry_logs/'))

    last_file = dir_list[-1]
    file_path = f"{working_dir}/{last_file}"
    telemetry_log = file_path
    
    last_line = getline(file_path)
    last_line_arr = last_line.split(",")

    try:
        current_state = [key for key, val in state.items() if val == last_line_arr[16]][0]
    except IndexError:
        global_state.value = 1
        return 0

    if (current_state == 4):
        new_file_split = last_file.split('_')
        file_num = int(new_file_split[2])
        file_num += 1
        new_file_split[2] = str(file_num)
        new_file = '_'.join(new_file_split)
        new_file_path = f"{working_dir}/{new_file}"
        file = open(new_file_path, "w", newline='')
        file.write("<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<3TEMP>,<4PRESSURE>,<5FREQ>,<6RSSI>,<7ROLL>,<8PITCH>,<9YAW>,<10GNSS_ALT>,<11GNSS_LAT>,<12GNSS_LONG>,<13GNSS_SAT>,<14VOLTAGE>,<15CURRENT>,<16STATE>,<17CHECKSUM>\n")
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
    voltage = vsas_volt_list[1][:-1]

    hackrf_line = getline(hackrf_log)
    hackrf_arr = hackrf_line.split(',')
    freq = hackrf_arr[1]
    rssi = hackrf_arr[2]

    current = -32 # add code to calculate it 
    state = global_state.value

    packet_count = global_packet_count.value

    telm_str = f"{team_id},{mission_time},{packet_count},{temp},{pressure},{freq},{rssi},{roll},{pitch},{yaw},{gnss_alt},{gnss_lat},{gnss_long},{gnss_sat},{voltage},{current},{state}"
    checksum = zlib.crc32(telm_str.encode())
    telm_str = f"#,{telm_str},{checksum},$\r\n"

    return telm_str

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
    #p1.start()
    time.sleep(2)
    #blenano_proc = Process(target=blenano_proc)
    #blenano_proc.start()

    main(global_state, global_packet_count)