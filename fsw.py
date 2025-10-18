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

from multiprocessing import Process, Value, Queue, Event, Lock
import subprocess
import os
import time
import zlib
from state_dep import *
from fsw_dep import *
import signal
import sys
import RPi.GPIO as GPIO


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

# Phone number to send the sms' to 
number =  "+919699060432"  # -> ashwin 

#=============================================================

actuate_fins_nano = "1"
deploy_parachute_nano = "2"
nano_alive = "3"

#=============================================================
BUZZER_PIN = 26
#=============================================================


def main(global_state, global_packet_count):
    mission_timer_flag = 1
    boot     =  0
    idle     =  1
    ascent   =  2
    descent  =  3
    recovery =  4

    global blenano_proc
    global hackrf_proc
    global gnss_proc
    global gsm_proc
    global rpicam_proc
    global ocp_shutdown

    global tx_enable

    sms_q = Queue()

    last_tx_timestamp = 0


    GPIO.setmode(GPIO.BCM)

    GPIO.setup(BUZZER_PIN, GPIO.OUT)

    GPIO.output(BUZZER_PIN, GPIO.LOW)

    blenano_proc = Process(target=blenano_proc, args=(arduino_flag,))
    gnss_proc = Process(target=gnss_proc)
    hackrf_proc = Process(target=hackrf_proc)
    lora_proc = Process(target=lora, args=(telm_q,tx_enable,global_packet_count,))
    gsm_proc = Process(target=gsm_proc, args=(sms_q,))
    measure_voltage_proc = Process(target=measure_voltage, args=(current_shared, voltage_shared, electrical_health_lock,))
    nano_health_check_proc = Process(target=check_arduino_health, args=(arduino_flag,))
    rpi_health = Process(target=health_check, args=(temp_event, voltage_event,ocp_event, current_shared, voltage_shared, electrical_health_lock,))
    rpicam_proc = Process(target=rpicam_proc, args=(pid_shared,))
    ocp_shutdown = Process(target=ocp_shutdown, args=(ocp_event,))
    blenano_proc.start()
    hackrf_proc.start()
    #lora_proc.start()
    gnss_proc.start()
    measure_voltage_proc.start()
    nano_health_check_proc.start()
    rpi_health.start()
    ocp_shutdown.start()

    #rpicam_proc.start()
    
    #gsm_proc.start()

    parachute_enable = 1
    fins_flag = 1

    beep(3)
    
    while True:
        time.sleep(0.1)
        state = global_state.value
        arduino_health = int(arduino_flag.is_set())
        temp_flag = int(temp_event.is_set())
        voltage_flag = int(voltage_event.is_set())
        e_flag = (arduino_health) + (temp_flag*2) + (voltage_flag*4)
        if (state == boot):
            print("Boot state")
            state_restore = file_manager(global_state)

            if (state_restore == 0):
                state = global_state.value
                beep(3)
                lora_proc.start()
                tx_enable.set()
            
            if (state_restore == 1):
                global_state.value = idle
                beep(3)
                lora_proc.start()
                state = global_state.value
                tx_enable.clear()

            telemetry_log_fd = open(telemetry_log, "a", newline="")
            
        if (state == idle):
            #print("Idle state")
            global mission_timer_start
            '''' this should happen once and then not again.'''
            if mission_timer_flag:
                mission_timer_start = time.time()
                mission_timer_flag -= 1
            
            telm_string = generate_telemetry(global_packet_count, global_state, current_shared, voltage_shared, tx_enable, e_flag)
            telemetry_log_fd.write(telm_string)
            '''            
            if not tx_enable.is_set():
                print("waiting for ack") 
                tx_enable.wait()
                print(tx_enable.is_set())
            '''
            
            if int(time.time() - last_tx_timestamp) >= 0.5:
                telm_q.put(telm_string)
                last_tx_timestamp = time.time()
            
            #print(telm_string)

            '''
            state change parameter from ascent to descent V, A, H must change for more than 3.
            '''
            idle_to_ascent_change()
            
            if state_change.change_state == True:
                global_state.value = ascent
                state_change.change_state = False
                beep(3)
    
        if (state == ascent):
            print("ascent state")
            '''
            telemetry storage
            --> LoRa in reception mode, switch 00000000000to transmit mode and transmit telemetry
            --> if receieved something process it
            '''
            telm_string_ascent = generate_telemetry(global_packet_count, global_state, current_shared, voltage_shared, tx_enable, e_flag)
            telemetry_log_fd.write(telm_string_ascent)
            # telm_string_ascent -> lora_proc -> transmission to ground

            if int(time.time() - last_tx_timestamp) >= 1:
                telm_q.put(telm_string_ascent)
                last_tx_timestamp = time.time()
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
                beep(3)
        
        if (state == descent):
            print("descent state")

            if fins_flag:
                nano_serial.write(actuate_fins_nano.encode())
                fins_flag = 0
   
            ble = getline(blenano_proc_log)
            ble_arr = ble.split(',')
            altitude = float(ble_arr[6])

            if (altitude < 510) and parachute_enable:
                nano_serial.write(deploy_parachute_nano.encode())
                parachute_enable = 0

            telm_string_descent = generate_telemetry(global_packet_count, global_state, current_shared, voltage_shared, tx_enable, e_flag)
            telemetry_log_fd.write(telm_string_descent)

            if int(time.time() - last_tx_timestamp) >= 1:
                telm_q.put(telm_string_descent)
                last_tx_timestamp = time.time()
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
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            
            telm_string = generate_telemetry(global_packet_count, global_state, current_shared, voltage_shared, tx_enable, e_flag)
            telemetry_log_fd.write(telm_string)

            sms_q.put(telm_string)


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
    last_line = last_line[0:-1]
    last_line_arr = last_line.split(",")

    try:
        current_state = [key for key, val in state.items() if val == last_line_arr[18]][0]
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
        file.write("#,<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<BLE_TEMP>,<BLE_PRESSURE>,<BLE_ALT>,<HACKRF_FREQ>,<HACKRF_RSSI>,<AX>,<AY>,<AZ>,<GX>,<GY>,<GZ>,<MX>,<MY>,<MZ>,<SOFTWARE_STATE>,<GNSS_LAT>,<GNSS_LONG>,<GNSS_ALT>,<GNSS_TIME>,<GNSS_SATS>,<VOLTAGE>,<CURRENT>,<RPI_TEMP>,<ERROR_FLAGS>,<CHECKSUM>,<ACK>,$\n")
        file.close()
        telemetry_log = new_file_path
        return 1
    
    global_state.value = current_state
    telemetry_log = file_path
    return 0

'''
gnss = "UTC","Latitude","Longitude","Sats","Altitude"
nanoble = "Timestamp", "Temperature", "Roll", "Pitch", "Yaw",
            "Pressure", "Altitude", "ax", "ay", "az",
            "gx", "gy", "gz", "mx", "my", "mz"
#<TEAM_ID> <MISSION_TIME> <PACKET_COUNT> <BLE_TEMP><BLE_PRESSURE> <BLE_ALT> <HACKRF_FREQ> <HACKRF_RSSI> <AX> <AY> <AZ> <GX> <GY> <GZ> <MX><MY><MZ> <SOFTWARE_STATE> <GNSS_LAT> <GNSS_LONG> <GNSS_ALT> <GNSS_TIME> <GNSS_SATS> <VOLTAGE> <CURRENT> <RPI_TEMP> <LORA_RSSI><ERROR_FLAGS><CHECKSUM><ACK>$\r\n
'''
def generate_telemetry(global_packet_count, global_state, current_shared, voltage_shared, tx_enable, e_flag):
    team_id = '050'
    mission_time = int(time.time() - mission_timer_start)

    try:
        ble = getline(blenano_proc_log)
        ble_arr = ble.split(',')
        temp = ble_arr[1]
        roll = ble_arr[2]
        pitch = ble_arr[3]
        yaw = ble_arr[4]
        pressure = ble_arr[5]
        ble_alt = ble_arr[6]
        ax, ay, az = ble_arr[7], ble_arr[8], ble_arr[9]
        gx, gy, gz = ble_arr[10], ble_arr[11], ble_arr[12]
        mx, my, mz = ble_arr[13], ble_arr[14], ble_arr[15][:-1]
    except:
        temp = -1
        roll = -1
        pitch = -1
        yaw = -1
        pressure = -1
        ble_alt = -1
        ax, ay, az = -1000,-1000,-1000
        gx, gy, gz = -1000,-1000,-1000
        mx, my, mz = -1000,-1000,-1000

    gnss = getline('proc_files/gnss_proc_log')
    gnss_arr = gnss.split(',')
    
    try:
        gnss_arr = gnss.split(',')
        gnss_time = gnss_arr[0]
        gnss_lat = float(gnss_arr[1])
        gnss_long = float(gnss_arr[2])
        gnss_sat = gnss_arr[3]
        gnss_alt = gnss_arr[4][:-1]
        
    except:
        gnss_time = 0 
        gnss_lat = 0.00
        gnss_long = 0.00
        gnss_sat = 0
        gnss_alt = 0

    vsas_volt = getline(blevsas_log)
    vsas_volt_list = vsas_volt.split(',')
    voltage = vsas_volt_list[1][:-1]

    hackrf_line = getline(hackrf_log)
    hackrf_arr = hackrf_line.split(',')
    freq = hackrf_arr[1]
    rssi = hackrf_arr[2][:-1]

    current = current_shared.value # add code to calculate it 
    voltage = voltage_shared.value
    state = global_state.value

    rpi_temperature = check_rpi_temp()
    
    error_flags = e_flag

    ack = int(tx_enable.is_set())

    packet_count = global_packet_count.value
    #global_packet_count.value += 1

    telm_str = f"{team_id},{mission_time},{packet_count},{temp},{pressure},{ble_alt},{freq},{rssi},{ax},{ay},{az},{gx},{gy},{gz},{mx},{my},{mz},{state},{gnss_lat:.2f},{gnss_long:.2f},{gnss_alt},{gnss_time},{gnss_sat},{voltage:.2f},{current:.2f},{rpi_temperature},{error_flags}"
    checksum = zlib.crc32(telm_str.encode())
    telm_str = f"#,{telm_str},00,{ack},$\r\n"

    return telm_str

def beep(duration):
    GPIO.output(BUZZER_PIN, GPIO.HIGH) # Turn the buzzer on
    time.sleep(duration)               # Wait for the specified duration
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def signal_handler(signum, frame):
    print(f"{signum} recieved")
    global lora_proc
    blenano_proc.terminate()
    hackrf_proc.terminate()
    lora_proc.terminate()
    #gnss_proc.start()
    #gsm_proc.start()
    sys.exit()


if __name__ == '__main__':
    electrical_health_lock = Lock()

    telm_q = Queue()
    
    tx_enable = Event()
    temp_event = Event()
    voltage_event = Event()
    arduino_flag = Event()
    parachute_deploy = Event()
    ocp_event = Event()

    current_shared = Value('f', 0.0) 
    voltage_shared = Value('f', 0.0)
    global_state = Value("i", 0)
    global_packet_count = Value("i", 0)
    pid_shared = Value("i", 0)
    #signal.signal(signal.SIGINT, signal_handler)
    #signal.signal(signal.SIGTERM, signal_handler)

    #p1.start()
    time.sleep(2)
    

    main(global_state, global_packet_count)
