'''
Author: Ashwin Kumar
'''

'''
TODO->  
GNSS Satellite
Current Value Calculation
Function to check last state
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
1. Implement File Locks Or something To make sure you dont read while a line is being written.
2. Telemetry Transmission -> invoke the process, then implement a pipe or something along those lines to send it whenever a telemetry string is ready. || Perhaps a queue and switch states depending on whether there is a request to serve.
3. Interrupt Handlers for Telecommands
4. Use pipes on processes that need to transmit something.
5. LoRa stays in reception, switches to transmission when telemetry is ready, simulate interrupts in software basically. 
'''

from multiprocessing import *
import subprocess
import os
import time
import zlib

mission_timer_start = 0

# File Paths For the process logs 
blenano_proc_log = 'proc_files/blenano_proc_log'
blevsas_log = 'proc_files/blevsas_log'
gnss_proc_log = 'proc_files/gnss_proc_log'
hackrf_log = 'proc_files/hackrf_log'
telemetry_log = 'proc_files/telemetry_log'

state = {
0: "BOOT",
1: "IDLE",
2: "ASCENT",
3: "DESCENT",
4: "RECOVERY"
}

def main(global_state, global_packet_count):
    boot     =  0
    idle     =  1
    ascent   =  2
    descent  =  3
    recovery =  4
    gnss_proc = Process(target=gnss_proc)
    blenano_proc = Process(target=blenano_proc)
    hackrf_proc = Process(target=hackrf_proc)
    while True:
        state = global_state.value
        if (state == boot):
            if (file_manager(global_state) == 0):
                # lora_proc = Process(target=lora_proc) in recieve mode
                print("Start LoRa in Recieve Mode")
            if (file_manager(global_state) == 1):
                print("Start LoRa in Transmit Mode")
            telemetry_log_fd = open(telemetry_log, "a", newline="")

        if (state == idle):
            global mission_timer_start
            mission_timer_start = time.time()
            '''
            ->-> sensors are being read and stored in respective proc files
            ->-> generate telemetry string -> write to telemetry log file
            ->  The Processes must be writing to files constantly
                generate_telemetry() might read from it while its still writing, 
                add protection for that lock or something. 
            '''
            telm_string = generate_telemetry(global_packet_count)
            telemetry_log_fd.write(telm_string)
            ''''
            --> Wait for Ack -> Start Transmiting Telemetry String
            LoRa_proc is running in recieve mode:
                two options -> wait for ACK
                            -> create some kind of signal using os/multiprocessing.event
            '''

            '''
            state change parameter from ascent to descent V, A, H must change for more than 3.
            '''
        if (state == ascent):
            '''
            telemetry storage
            Interrupt based system for telecommands.
            '''
            telm_string_ascent = generate_telemetry(global_packet_count)
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
        if (state == descent):
            '''
            check altitude if between 490 and 510 open parachute. 
            '''
            telm_string_descent = generate_telemetry(global_packet_count)
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
        if (state == recovery):
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

    state = [key for key, val in state.items() if val == last_line_arr[16]][0]
    if (state == 4):
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
    global_state.value = state
    # start LoRa in Transmit Mode
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

'''
def gnss_proc(gnss_q):
    

def blenano_proc():
    

def hackrf_proc():
    
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
def gsm_proc():
'''

if __name__ == '__main__':
    global_state = Value("i", 0)
    global_packet_count = Value("i", 0)
    #file_manager(global_state)
    #print(telemetry_log)
    #telm = generate_telemetry(global_packet_count, global_state)
    #print(telm, end="")
    #print(global_state.value)
    #main(state)+
