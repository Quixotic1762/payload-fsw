from multiprocessing import Process
import subprocess
import time
import sys
import signal


bme_log = "test_logs/bme_proc"


def bme_proc():
    import board
    import busio
    import adafruit_bme680
    import csv

    global bme_log

    def termination_handler(signum, frame):
        bme_fd.close()
        print("Exiting")
        sys.exit()

    signal.signal(signal.SIGINT, termination_handler)
    signal.signal(signal.SIGTERM, termination_handler)


    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    sensor = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    sensor.sea_level_pressure = sensor.pressure

    bme_fd = open(bme_log, 'w', newline='')
    bme_writer = csv.writer(bme_fd)
    bme_writer.writerow(["Temperature","Pressure", "Altitude"])

    alpha = 0.1
    smoothed_alt = 0

    while True:
        try:
            temperature = sensor.temperature
            pressure = sensor.pressure
            altitude = sensor.altitude
        except Exception as e:
            print(e)
            temperature = 0
            pressure = 0
            altitude = 0

        smoothed_alt = (alpha * altitude) + (1 - alpha) * smoothed_alt
        bme_row = [round(temperature, 2), round(pressure, 2), round(smoothed_alt, 2)]
        bme_writer.writerow(bme_row)
        bme_fd.flush()
        time.sleep(0.05)

def print_line():
    global bme_log
    while True:
        line = getline(bme_log)
        print(line)
        time.sleep(0.5)
    
def getline(proc_fp):
    line = subprocess.check_output(['tail','-n','1',proc_fp])
    return line.decode().split('\n')[0]

def termination_handler(signum, frame):
    global bme_proc
    global print_line
    bme_proc.terminate()
    print_line.terminate()
    sys.exit()

bme_proc = Process(target=bme_proc)
print_line = Process(target=print_line)
signal.signal(signal.SIGINT, termination_handler)
signal.signal(signal.SIGTERM, termination_handler)


bme_proc.start()
print_line.start()


