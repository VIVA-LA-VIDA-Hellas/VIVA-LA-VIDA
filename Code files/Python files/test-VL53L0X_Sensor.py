import time
from vl53l0x_custom import VL53L0X_Sensor

sensor1 = VL53L0X_Sensor(5)  # XSHUT on GPIO5, address set to 0x05
sensor2 = VL53L0X_Sensor(6)  # GPIO6, address 0x06

try:
    while True:
        print(f"Sensor 5: {sensor1.read_distance()} mm")
        print(f"Sensor 6: {sensor2.read_distance()} mm")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
