import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

class PCAController:
    def __init__(self, address=0x40, freq=50):
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=address)
        self.pca.frequency = freq

    def set_pwm(self, channel, value):
        value = max(0, min(0xFFFF, value))  # clamp to 16-bit
        self.pca.channels[channel].duty_cycle = value

    def cleanup(self):
        self.pca.deinit()

class Servo:
    def __init__(self, pca, channel, min_us=500, max_us=2500):
        self.pca = pca
        self.channel = channel
        self.min_pwm = int(min_us / 20000 * 0xFFFF)
        self.max_pwm = int(max_us / 20000 * 0xFFFF)

    def set_angle(self, angle):  # 0 - 180 degrees
        angle = max(0, min(180, angle))
        pwm_val = self.min_pwm + (self.max_pwm - self.min_pwm) * angle // 180
        self.pca.set_pwm(self.channel, pwm_val)

class DCMotor:
    def __init__(self, pca, in1, in2):
        self.pca = pca
        self.in1 = in1
        self.in2 = in2

    def run(self, speed):  # -100 (rev) to 100 (fwd)
        speed = max(-100, min(100, speed))
        pwm_val = int(abs(speed) / 100 * 0xFFFF)

        if speed > 0:
            self.pca.set_pwm(self.in1, pwm_val)
            self.pca.set_pwm(self.in2, 0)
        elif speed < 0:
            self.pca.set_pwm(self.in1, 0)
            self.pca.set_pwm(self.in2, pwm_val)
        else:
            self.pca.set_pwm(self.in1, 0)
            self.pca.set_pwm(self.in2, 0)
