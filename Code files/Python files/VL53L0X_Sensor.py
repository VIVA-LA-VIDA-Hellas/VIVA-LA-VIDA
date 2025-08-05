import time
import board
import busio
import digitalio
import adafruit_vl53l0x

class VL53L0X_Sensor:
    def __init__(self, xshut_gpio_pin):
        # Setup I2C bus internally
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Setup XSHUT pin by GPIO number
        self.xshut = digitalio.DigitalInOut(board.pin.GPIO(xshut_gpio_pin))
        self.xshut.direction = digitalio.Direction.OUTPUT

        # Power off sensor first
        self.xshut.value = False
        time.sleep(0.1)

        # Power on sensor
        self.xshut.value = True
        time.sleep(0.1)

        # Initialize sensor with default I2C address
        self.sensor = adafruit_vl53l0x.VL53L0X(self.i2c)

        # Set address to GPIO pin number as hex
        self.address = xshut_gpio_pin
        self.sensor.set_address(self.address)
        time.sleep(0.1)

    def read_distance(self):
        return self.sensor.range
