import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.1)

    def get_distance(self):
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        timeout = time.time() + 0.04
        while GPIO.input(self.echo) == 0 and time.time() < timeout:
            pulse_start = time.time()

        timeout = time.time() + 0.04
        while GPIO.input(self.echo) == 1 and time.time() < timeout:
            pulse_end = time.time()

        try:
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * 34300) / 2  # σε εκατοστά
            return round(distance, 1)  # ✳️ Επιστρέφει με 1 δεκαδικό ψηφίο
        except:
            return None

    @staticmethod
    def cleanup():
        GPIO.cleanup()
