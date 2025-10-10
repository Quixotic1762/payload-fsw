import ina219_lib
import time
from multiprocessing import Lock, Value

def measure_voltage(current_shared, voltage_shared, electrical_health_lock):
    ina219 = ina219_lib.INA219(i2c_bus=1,addr=0x43)
    while True:
        with electrical_health_lock:
            voltage = int(ina219.getBusVoltage_V())
            voltage = "{:6.3f}".format(voltage)
            voltage_shared.value = float(voltage)
            current = ina219.getCurrent_mA()
            current = "{:6.3f}".format(current/1000)
            current_shared.value = float(current)
            print(f"Voltage: {voltage_shared.value}, current: {current_shared.value}")
        time.sleep(0.2)

voltage_shared = Value('f', 0)
current_shared = Value('f', 0)
electrical_health_lock = Lock()

measure_voltage(voltage_shared, current_shared, electrical_health_lock)