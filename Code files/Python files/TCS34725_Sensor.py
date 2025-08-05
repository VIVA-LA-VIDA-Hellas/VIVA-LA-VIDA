import time
import board
import digitalio
import busio
import adafruit_tcs34725

class TCS34725_Sensor:
    VALID_INTEGRATION_TIMES = [1, 4, 16, 60]  # Allowed integration times in ms

    def __init__(self, transistor_gpio_pin=12, gain=4, integration_time=16):
        if integration_time not in self.VALID_INTEGRATION_TIMES:
            raise ValueError(f"Integration time must be one of {self.VALID_INTEGRATION_TIMES} ms")

        # Setup I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Setup transistor control pin (to power sensor on/off)
        self.transistor = digitalio.DigitalInOut(getattr(board, f"GPIO{transistor_gpio_pin}"))
        self.transistor.direction = digitalio.Direction.OUTPUT

        self.sensor = None
        self.transistor_gpio_pin = transistor_gpio_pin
        self.gain = gain
        self.integration_time = integration_time

    def open(self):
        # Power on sensor via transistor
        self.transistor.value = True
        time.sleep(0.1)

        # Initialize the sensor instance
        if self.sensor is None:
            self.sensor = adafruit_tcs34725.TCS34725(self.i2c)
            self.sensor.gain = self.gain
            self.sensor.integration_time = self.integration_time

    def close(self):
        # Power off sensor
        self.transistor.value = False
        self.sensor = None

    def read_color(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.color_raw  # returns (r, g, b, clear)

    def read_red(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.color_raw[0]

    def read_green(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.color_raw[1]

    def read_blue(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.color_raw[2]

    def read_lux(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.lux

    def read_color_temperature(self):
        if self.sensor is None:
            raise RuntimeError("Sensor is not powered on. Call open() first.")
        return self.sensor.color_temperature
