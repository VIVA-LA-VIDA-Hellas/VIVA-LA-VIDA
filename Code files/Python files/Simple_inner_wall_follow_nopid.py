'''No pid steering, find direction with distance sensors, no loops simple inner wall follow'''

import board
import busio
import time
import digitalio
import adafruit_vl53l0x
from adafruit_servokit import ServoKit
from adafruit_motorkit import MotorKit

# Constants
STEERING_CENTER = 90
STEERING_LEFT = 5
STEERING_RIGHT = 165

TURN_ADJUST_LEFT = 80
TURN_ADJUST_RIGHT = 100

FORWARD_SPEED = 0.0
TOF_THRESHOLD = 70  # in cm
ALIGN_THRESHOLD = 7  # in cm

# Setup I2C and devices
i2c = busio.I2C(board.SCL, board.SDA)
kit = ServoKit(channels=8, i2c=i2c, address=0x40)
kit2 = MotorKit()

kit.servo[0].angle = STEERING_CENTER
kit2.motor3.throttle = FORWARD_SPEED

# Setup TOF sensors
xshut_left = digitalio.DigitalInOut(board.D17)
xshut_right = digitalio.DigitalInOut(board.D27)
xshut_left.direction = digitalio.Direction.OUTPUT
xshut_right.direction = digitalio.Direction.OUTPUT

xshut_left.value = False
xshut_right.value = False
time.sleep(0.1)

xshut_left.value = True
time.sleep(0.1)
sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x32)

xshut_right.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
sensor_right.set_address(0x31)

sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x32)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)

# Main loop
while True:
    left_distance = sensor_left.range / 10  # mm to cm
    right_distance = sensor_right.range / 10

    print(f"Left: {left_distance:.1f} cm, Right: {right_distance:.1f} cm")

    # Priority 1: Hard turn if any sensor exceeds TOF_THRESHOLD
    if left_distance > TOF_THRESHOLD:
        print("Turning hard left (priority override)")
        kit.servo[0].angle = STEERING_LEFT
        time.sleep(0.1)
        continue

    if right_distance > TOF_THRESHOLD:
        print("Turning hard right (priority override)")
        kit.servo[0].angle = STEERING_RIGHT
        time.sleep(0.1)
        continue

    # Priority 2: Alignment if both distances are ≤ threshold
    print("Going straight")
    kit.servo[0].angle = STEERING_CENTER
    time.sleep(0.1)

    if left_distance > right_distance:
        print("Adjusting right until right ≤ 7cm")
        while sensor_right.range / 10 > ALIGN_THRESHOLD:
            # Check if new reading breaks the hard turn condition
            if sensor_left.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard left required")
                kit.servo[0].angle = STEERING_LEFT
                break
            if sensor_right.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard right required")
                kit.servo[0].angle = STEERING_RIGHT
                break
            kit.servo[0].angle = TURN_ADJUST_LEFT
            time.sleep(0.05)

    elif right_distance > left_distance:
        print("Adjusting left until left ≤ 7cm")
        while sensor_left.range / 10 > ALIGN_THRESHOLD:
            # Check if new reading breaks the hard turn condition
            if sensor_left.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard left required")
                kit.servo[0].angle = STEERING_LEFT
                break
            if sensor_right.range / 10 > TOF_THRESHOLD:
                print("Interrupting alignment: hard right required")
                kit.servo[0].angle = STEERING_RIGHT
                break
            kit.servo[0].angle = TURN_ADJUST_RIGHT
            time.sleep(0.05)

    time.sleep(0.1)

