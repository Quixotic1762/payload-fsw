import subprocess
import time

while True:
    rpi_temp = subprocess.check_output(['vcgencmd', 'measure_temp'])
    print(rpi_temp.decode())
    time.sleep(0.02)