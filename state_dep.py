import time
from fsw_dep import *
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

MIN_ASCENT_ALTITUDE = 30

state_change = state_change_parameters()

recovery_change = recovery_change_parameters()

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