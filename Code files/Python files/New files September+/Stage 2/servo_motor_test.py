import time
from pca9685_control import set_servo_angle, set_motor_speed

MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2  # reverse channel
SERVO_CHANNEL = 0

set_servo_angle(SERVO_CHANNEL, 90)

set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
time.sleep(5)
