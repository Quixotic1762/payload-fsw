import serial
import time

nano_ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
nano_stdout = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
while True:
    fins = "1"
    parachute = "2"
    alive = "3"
    nano_ser.reset_input_buffer()

    nano_ser.write(fins.encode())
    print(nano_stdout.read(nano_stdout.in_waiting).decode('utf-8'))
    time.sleep(0.5)

    nano_ser.write(parachute.encode())
    nano_ser.reset_input_buffer()
    print(nano_stdout.read(nano_stdout.in_waiting).decode('utf-8'))
    time.sleep(0.5)
    
    nano_ser.write(alive.encode())
    buffer = nano_stdout.read(nano_stdout.in_waiting).decode('utf-8')

    if "128" in buffer:
        print("ACK recieved")
    
    time.sleep(2)