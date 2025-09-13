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
6. Improve the file system access thing, its too slow.
'''

from multiprocessing import Process, Value, Queue
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

class ascent_state_parameters:
    last_check = 0
    last_altitude = 0
    last_acceleration = 0
    altitude_flag = [0,0]
    acceleration_flag = [0,0]
    min_altitude_flag = 0
    flag_sum = 0
    change_state = False

ascent_conditions = ascent_state_parameters()

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
        time.sleep(0.15)
        state = global_state.value
        if (state == boot):
            if (file_manager(global_state) == 0):
                # lora_proc = Process(target=lora_proc) in recieve mode
                print("Start LoRa in Recieve Mode")
            if (file_manager(global_state) == 1):
                print("Start LoRa in Transmit Mode")
            telemetry_log_fd = open(telemetry_log, "a", newline="")
            global_state.value = 1
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
            telm_string = generate_telemetry(global_packet_count, global_state)
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
            idle_to_ascent_change()
            if ascent_conditions.change_state == True:
                print("state = Ascent")
                global_state.value = ascent
    
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
        
        if (state == descent):
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

    delta_time = current_check_time - ascent_conditions.last_check
    delta_altitude = current_altitude - ascent_conditions.last_altitude
    delta_acceleration = current_acceleration - ascent_conditions.last_acceleration

    if delta_time < 2:
        if delta_altitude > 0:
            ascent_conditions.altitude_flag[0] = 1
        else:
            ascent_conditions.altitude_flag[1] += 1

        if delta_acceleration > 0:
            ascent_conditions.acceleration_flag[0] = 1
        else:
            ascent_conditions.acceleration_flag[1] += 1
        
        if current_altitude > MIN_ASCENT_ALTITUDE:
            ascent_conditions.min_altitude_flag = 1
        else:
            ascent_conditions.min_altitude_flag = 0
    else:
        if (ascent_conditions.altitude_flag[1] == 0):
            ascent_conditions.flag_sum += ascent_conditions.altitude_flag[0]

        if (ascent_conditions.acceleration_flag[1] == 0):
            ascent_conditions.flag_sum += ascent_conditions. acceleration_flag[0]

        ascent_conditions.flag_sum += ascent_conditions.min_altitude_flag
        ascent_conditions.altitude_flag[1] = 0
        ascent_conditions.acceleration_flag[1] = 0
        print(f"flag sum: {ascent_conditions.flag_sum}")

        if ascent_conditions.flag_sum >= 2:
            print(f"flags met {ascent_conditions.flag_sum}")
            ascent_conditions.change_state = True
        ascent_conditions.last_check = current_check_time

    ascent_conditions.last_altitude = current_altitude
    ascent_conditions.last_acceleration = current_acceleration
    ascent_conditions.flag_sum = 0

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
def gnss_proc():
    

def blenano_proc():
    

def hackrf_proc():
    
# Update the Queue System to reading relevant files.
def lora_proc(telm_queue):

def telecom_parse_proc(telecommand):

def gsm_proc():
'''

if __name__ == '__main__':
    global_state = Value("i", 0)
    global_packet_count = Value("i", 0)
    lora_telm_queue = Queue()
    p1 = Process(target=cpy_src)
    p1.start()
    time.sleep(0.15)
    main(global_state, global_packet_count)
    #file_manager(global_state)
    #print(telemetry_log)
    #telm = generate_telemetry(global_packet_count, global_state)
    #print(telm, end="")
    #print(global_state.value)
    #main(state)+
