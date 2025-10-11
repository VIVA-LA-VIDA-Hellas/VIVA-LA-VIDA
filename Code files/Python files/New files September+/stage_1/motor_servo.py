from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio

class MotorServoController:
    def __init__(self, freq=50):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = freq

        # Κανάλια
        self.servo_ch = 0       # Τιμόνι
        self.motor_fwd_ch = 1   # Μοτέρ εμπρός
        self.motor_rev_ch = 2   # Μοτέρ πίσω

        self.steering_servo = servo.Servo(self.pca.channels[self.servo_ch])

        print("[INFO] PCA9685 initialized (Servo: ch0 | Motor: ch1-2)")

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        self.steering_servo.angle = angle

    def set_motor_speed(self, speed_percent):
        speed_percent = max(-100, min(100, speed_percent))
        pwm_value = int(abs(speed_percent) / 100 * 65535)

        if speed_percent > 0:
            self.pca.channels[self.motor_rev_ch].duty_cycle = 0
            self.pca.channels[self.motor_fwd_ch].duty_cycle = pwm_value
        elif speed_percent < 0:
            self.pca.channels[self.motor_fwd_ch].duty_cycle = 0
            self.pca.channels[self.motor_rev_ch].duty_cycle = pwm_value
        else:
            self.stop_motor()

    def stop_motor(self):
        self.pca.channels[self.motor_fwd_ch].duty_cycle = 0
        self.pca.channels[self.motor_rev_ch].duty_cycle = 0

    def cleanup(self):
        self.stop_motor()
        self.pca.deinit()
        print("[INFO] PCA9685 stopped cleanly.")
