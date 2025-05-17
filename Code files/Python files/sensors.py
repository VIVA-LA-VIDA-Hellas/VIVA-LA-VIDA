# sensors.py

import board
import busio
import time
import digitalio
import adafruit_vl53l0x
import adafruit_tcs34725

i2c = busio.I2C(board.SCL, board.SDA)

# Setup TOF sensor shutdown pins
xshut_left = digitalio.DigitalInOut(board.D17)
xshut_right = digitalio.DigitalInOut(board.D27)
xshut_left.direction = digitalio.Direction.OUTPUT
xshut_right.direction = digitalio.Direction.OUTPUT

xshut_left.value = False
xshut_right.value = False
time.sleep(0.1)

xshut_left.value = True
time.sleep(0.1)
sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x32)

xshut_right.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
sensor_right.set_address(0x31)

sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x32)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)

# Color sensor
#sensor_color = adafruit_tcs34725.TCS34725(i2c)
#sensor_color.gain = 16
#sensor_color.integration_time = 2.4