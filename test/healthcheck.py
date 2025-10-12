import time
from multiprocessing import Event
import serial
arduino_flag = Event()
nano_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
try: 
    nano_stdout = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
except:
    print("Reconnect the nano USB")
nano_alive = '3'

 
def check_arduino_health(arduino_flag):
    global nano_stdout
    global nano_serial
    dev = 0
    reinit_flag = 0
    while True:
        if reinit_flag:
            try:
                dev = 1 - dev
                dev_path = f"/dev/ttyACM{dev}"
                nano_serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
                nano_stdout = serial.Serial(dev_path, 115200, timeout=1)
                reinit_flag = 0
            except:
                reinit_flag = 1 
                arduino_flag.clear()
                pass
             
        try:
            nano_serial.write(nano_alive.encode())
            buffer = nano_stdout.read(nano_stdout.in_waiting).decode('utf-8')
        except OSError:
            buffer = '0'
            arduino_flag.clear()
            reinit_flag = 1

        if "128" in buffer:
            reinit_flag = 0
            arduino_flag.set()
        print(f"Arduino Health: {arduino_flag.is_set()}")
        time.sleep(0.5)

check_arduino_health(arduino_flag)
