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

TURN_ADJUST_LEFT = 80
TURN_ADJUST_RIGHT = 100

THROTTLE = 80
TOF_THRESHOLD = 70
ALIGN_THRESHOLD = 10

FRONT_DISTANCE_THRESHOLD = 100

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

# Power down both TOF sensors
xshut_left.value = False
xshut_right.value = False
xshut_front.value = False
xshut_back.value = False 
time.sleep(0.1)

# Power up left sensor and set its address
xshut_left.value = True
time.sleep(0.1)
sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x30)

# Power up right sensor and set its address
xshut_right.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
sensor_right.set_address(0x31)

xshut_front.value = True
time.sleep(0.1)
xshut_front = adafruit_vl53l0x.VL53L0X(i2c)
xshut_front.set_address(0x32)

xshut_back.value = True
time.sleep(0.1)
xshut_back = adafruit_vl53l0x.VL53L0X(i2c)
xshut_back.set_address(0x33)

# Initialize sensors with new addresses
sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x30)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)
xshut_front = adafruit_vl53l0x.VL53L0X(i2c, address=0x32)
xshut_back = adafruit_vl53l0x.VL53L0X(i2c, address=0x33)

GPIO.output(12, GPIO.LOW)
pca9685_module.set_servo_angle(0, 48)
pca9685_module.set_motor(1, 2, THROTTLE)

def stop_motor():
    pca9685_module.set_motor(1, 2, 50)

# Main loop
while True:
    left_distance = sensor_left.range / 10  # mm to cm
    right_distance = sensor_right.range / 10


    print(f"Left: {left_distance:.1f} cm, Right: {right_distance:.1f} cm")

    # Priority 1: Hard turn if any sensor exceeds TOF_THRESHOLD
    if left_distance > TOF_THRESHOLD:
        print("Turning hard left (priority override)")
        pca9685_module.set_servo_angle(0, STEERING_LEFT)
        time.sleep(0.1)
        continue

    if right_distance > TOF_THRESHOLD:
        print("Turning hard right (priority override)")
        pca9685_module.set_servo_angle(0, STEERING_RIGHT)
        time.sleep(0.1)
        continue

    # Priority 2: Alignment if both distances are ≤ threshold
    print("Going straight")
    pca9685_module.set_servo_angle(0, STEERING_CENTER)
    time.sleep(0.1)

    if left_distance > right_distance:
        print("Adjusting right until right ≤ 7cm")
        while sensor_right.range / 10 > ALIGN_THRESHOLD:
            # Check if new reading breaks the hard turn condition
            if sensor_left.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard left required")
                pca9685_module.set_servo_angle(0, STEERING_LEFT)
                break
            if sensor_right.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard right required")
                pca9685_module.set_servo_angle(0, STEERING_RIGHT)
                break
            pca9685_module.set_servo_angle(0, TURN_ADJUST_LEFT)
            time.sleep(0.05)

    elif right_distance > left_distance:
        print("Adjusting left until left ≤ 7cm")
        while sensor_left.range / 10 > ALIGN_THRESHOLD:
            # Check if new reading breaks the hard turn condition
            if sensor_left.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard left required")
                pca9685_module.set_servo_angle(0, STEERING_LEFT)

                break
            if sensor_right.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard right required")
                pca9685_module.set_servo_angle(0, STEERING_RIGHT)
                break
            pca9685_module.set_servo_angle(0, TURN_ADJUST_RIGHT)

            time.sleep(0.05)

