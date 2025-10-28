from gpiozero import Buzzer
import time
buzzer = Buzzer(26)

print("Buzzing...")
buzzer.on()     # turn buzzer ON
time.sleep(2)
buzzer.off()    # turn buzzer OFF
print("Done.")
