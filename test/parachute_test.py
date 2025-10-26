import serial

nano_ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

parachute_trigger = '2'
nano_ser.write(parachute_trigger.encode())
