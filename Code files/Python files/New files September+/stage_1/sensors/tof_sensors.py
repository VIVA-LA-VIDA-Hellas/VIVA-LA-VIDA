import time
import RPi.GPIO as GPIO
import board
import busio
import adafruit_vl53l0x

class TOFSensors:
    def __init__(self, xshut_pins, i2c_addresses):
        """
        xshut_pins: λίστα με GPIO pins [left, front, right, back]
        i2c_addresses: λίστα με μοναδικές I2C διευθύνσεις [0x30, 0x31, 0x32, 0x33]
        """
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.xshut_pins = xshut_pins
        self.i2c_addresses = i2c_addresses
        self.sensors = {}

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Κλείνουμε όλους τους αισθητήρες
        for pin in self.xshut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        time.sleep(0.1)

        # Ενεργοποιούμε έναν-έναν και αλλάζουμε διεύθυνση
        sensor_names = ["left", "front", "right", "back"]
        for i, pin in enumerate(self.xshut_pins):
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.1)
            sensor = adafruit_vl53l0x.VL53L0X(self.i2c)
            sensor.set_address(self.i2c_addresses[i])
            self.sensors[sensor_names[i]] = sensor
            print(f"[TOF] {sensor_names[i]} ενεργός στη διεύθυνση 0x{self.i2c_addresses[i]:X}")
            time.sleep(0.05)

    def get_distances(self):
        """
        Επιστρέφει αποστάσεις σε εκατοστά (cm)
        π.χ. {"front": 45.6, "left": 23.7, "right": 50.1, "back": 31.4}
        """
        data = {}
        for name, sensor in self.sensors.items():
            try:
                # μετατροπή mm → cm
                data[name] = round(sensor.range / 10, 2)
            except Exception:
                data[name] = None
        return data
