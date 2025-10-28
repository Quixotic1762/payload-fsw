from gpiozero import OutputDevice
import time

nano_rst = OutputDevice(17, active_high=True, initial_value=True)


nano_rst.off()
time.sleep(0.1)
nano_rst.on()
