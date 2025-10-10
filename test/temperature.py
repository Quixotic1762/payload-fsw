import subprocess
import time

def check_rpi_temp():
    rpi_temp = subprocess.check_output(['vcgencmd', 'measure_temp'])
    rpi_temp = rpi_temp.decode()
    rpi_temp = rpi_temp[:-1]
    rpi_temp = rpi_temp.split('=')[1][:-2]
    return (rpi_temp)

while True:
    time.sleep(2)
    print(check_rpi_temp())