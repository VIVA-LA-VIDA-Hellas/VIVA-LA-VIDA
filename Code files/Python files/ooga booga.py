import board
import busio
import time
import digitalio
import vl53l0x_module
import adafruit_vl53l0x
import RPi.GPIO as GPIO
import pca9685_module

# Constants
STEERING_CENTER = 48
STEERING_LEFT = 20
STEERING_RIGHT = 150

THROTTLE = 80
BASE_SPEED = 75
TARGET_DISTANCE_MM = 300

# PID settings
Kp = 0.3
Ki = 0.01
Kd = 0.1

pid_integral = 0
last_error = 0

LEFT_MOTOR_CHANNEL = 1
RIGHT_MOTOR_CHANNEL = 2

# Setup I2C and devices
vl53l0x_module.setup_vl53l0x()
i2c = busio.I2C(board.SCL, board.SDA)

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)
GPIO.output(12, GPIO.HIGH)

# Setup XSHUT pins for VL53L0X sensors
xshut_left = digitalio.DigitalInOut(board.D5)
xshut_right = digitalio.DigitalInOut(board.D6)
xshut_front = digitalio.DigitalInOut(board.D16)
xshut_back = digitalio.DigitalInOut(board.D26)

xshut_left.direction = digitalio.Direction.OUTPUT
xshut_right.direction = digitalio.Direction.OUTPUT
xshut_front.direction = digitalio.Direction.OUTPUT
xshut_back.direction = digitalio.Direction.OUTPUT

# Power down all TOF sensors
xshut_left.value = False
xshut_right.value = False
xshut_front.value = False
xshut_back.value = False
time.sleep(0.1)

# Power up each sensor and set unique address
xshut_left.value = True
time.sleep(0.1)
sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x30)

xshut_right.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
sensor_right.set_address(0x31)

xshut_front.value = True
time.sleep(0.1)
sensor_front = adafruit_vl53l0x.VL53L0X(i2c)
sensor_front.set_address(0x32)

xshut_back.value = True
time.sleep(0.1)
sensor_back = adafruit_vl53l0x.VL53L0X(i2c)
sensor_back.set_address(0x33)

# Re-initialize sensors with new addresses
sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x30)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)
sensor_front = adafruit_vl53l0x.VL53L0X(i2c, address=0x32)
sensor_back = adafruit_vl53l0x.VL53L0X(i2c, address=0x33)

# Motor and steering initialization
GPIO.output(12, GPIO.LOW)
pca9685_module.set_servo_angle(0, STEERING_CENTER)
pca9685_module.set_motor(1, 2, THROTTLE)

# Helper Functions
def stop_motor():
    set_motor_speeds(0, 0)

def set_motor_speeds(left_speed, right_speed):
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))
    pca9685_module.set_motor(LEFT_MOTOR_CHANNEL, 0, left_speed)
    pca9685_module.set_motor(RIGHT_MOTOR_CHANNEL, 0, right_speed)

def rotate_sharp(direction):
    if direction == 'left':
        pca9685_module.set_servo_angle(0, STEERING_LEFT)
    else:
        pca9685_module.set_servo_angle(0, STEERING_RIGHT)
    set_motor_speeds(70, 70)

def rotate_wide(direction):
    if direction == 'left':
        pca9685_module.set_servo_angle(0, 30)
    else:
        pca9685_module.set_servo_angle(0, 120)
    set_motor_speeds(BASE_SPEED, BASE_SPEED)

# === Initial Direction Detection ===
turn_direction = None
pca9685_module.set_servo_angle(0, STEERING_CENTER)
set_motor_speeds(BASE_SPEED, BASE_SPEED)

while True:
    if sensor_left.range > 500:
        turn_direction = 'left'
        print("Initial direction: left")
        stop_motor()
        break
    elif sensor_right.range > 500:
        turn_direction = 'right'
        print("Initial direction: right")
        stop_motor()
        break
    time.sleep(0.05)

# === Wide Turn to Align with Wall ===
rotate_wide(turn_direction)
while True:
    if turn_direction == 'left' and sensor_left.range < 400:
        print("Aligned with wall (left)")
        stop_motor()
        break
    elif turn_direction == 'right' and sensor_right.range < 400:
        print("Aligned with wall (right)")
        stop_motor()
        break
    time.sleep(0.01)

# === Main PID-Controlled Loop ===
for _ in range(11):
    print("=== PID wall-following phase ===")
    pid_integral = 0
    last_error = 0

    while True:
        distance_mm = sensor_left.range if turn_direction == 'left' else sensor_right.range

        error = TARGET_DISTANCE_MM - distance_mm
        pid_integral += error
        derivative = error - last_error
        correction = Kp * error + Ki * pid_integral + Kd * derivative
        last_error = error

        if turn_direction == 'left':
            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction
        else:
            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction

        set_motor_speeds(left_speed, right_speed)
        print(f"Distance: {distance_mm} mm | Error: {error:.1f} | Correction: {correction:.1f}")

        if distance_mm > 500:
            print(f"Gap detected on fixed {turn_direction} side")
            stop_motor()
            break

        time.sleep(0.05)

    print(f"Turning sharply {turn_direction}")
    rotate_sharp(turn_direction)

    while True:
        if turn_direction == 'left' and sensor_left.range < 400:
            print("Aligned again with wall (left)")
            break
        elif turn_direction == 'right' and sensor_right.range < 400:
            print("Aligned again with wall (right)")
            break
        time.sleep(0.01)

# === Shutdown ===
print("All loops completed. Stopping motors.")
stop_motor()
GPIO.cleanup()
