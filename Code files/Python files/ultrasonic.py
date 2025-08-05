import RPi.GPIO as GPIO
import time
import statistics

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.1)

    def _send_pulse(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)  # 10µs pulse
        GPIO.output(self.trig, False)

    def _measure_pulse(self, timeout=0.02):
        start_time = time.time()
        while GPIO.input(self.echo) == 0:
            if time.time() - start_time > timeout:
                return -1
        start = time.time()
        while GPIO.input(self.echo) == 1:
            if time.time() - start > timeout:
                return -1
        end = time.time()
        return end - start

    def get_distance(self, samples=5):
        readings = []

        for _ in range(samples):
            self._send_pulse()
            duration = self._measure_pulse()
            if duration != -1:
                distance = (duration * 34300) / 2  # cm
                readings.append(distance)
            time.sleep(0.01)

        if not readings:
            return -1  # No valid readings

        return round(statistics.median(readings), 2)

    def cleanup(self):
        GPIO.cleanup([self.trig, self.echo])
