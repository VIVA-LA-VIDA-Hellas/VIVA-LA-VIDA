import time
from adafruit_servokit import ServoKit
import board
import busio
import digitalio
import adafruit_vl53l0x
from adafruit_as7341 import AS7341

# Setup I2C (shared communication line for sensors)
i2c = busio.I2C(board.SCL, board.SDA)

# Setup XSHUT pins for VL53L0X sensors
xshut_left = digitalio.DigitalInOut(board.D17)
xshut_right = digitalio.DigitalInOut(board.D27)
xshut_left.direction = digitalio.Direction.OUTPUT
xshut_right.direction = digitalio.Direction.OUTPUT

# Power down both TOF sensors
xshut_left.value = False
xshut_right.value = False
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

# Initialize sensors with new addresses
sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x30)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)

# Color sensor
color_sensor = AS7341(i2c)

# Check if the color sensor is detected and initialized
if color_sensor:
    print("Color sensor initialized successfully.")
else:
    print("Color sensor initialization failed!")

# Turn on the sensor LED for better detection
color_sensor.led_current = 0
color_sensor.led = True

# Function to create a simple bar graph of the readings
def bar_graph(read_value):
    scaled = int(read_value / 1000)
    return "[%5d] " % read_value + (scaled * "*")

# ServoKit for motor control
kit = ServoKit(channels=8, i2c=i2c, address=0x40)

# ---------------- MOVEMENT FUNCTIONS ------------------
def turn_left(duration):
    print(f"Turning Left for {duration:.2f} sec")
    kit.servo[0].angle = 70
    kit.continuous_servo[1].throttle = 1
    time.sleep(duration)
    kit.continuous_servo[1].throttle = 0

def turn_right(duration):
    print(f"Turning Right for {duration:.2f} sec")
    kit.servo[0].angle = 130
    kit.continuous_servo[1].throttle = 1
    time.sleep(duration)
    kit.continuous_servo[1].throttle = 0

# ---------------- MAIN LOOP ------------------
try:
    time.sleep(1)

    # Get initial distances from TOF sensors
    left = sensor_left.range / 10  # mm to cm
    right = sensor_right.range / 10

    if left > right:
        reference_side = "left"
        waiting_for = "right"
    else:
        reference_side = "right"
        waiting_for = "left"

    print(f"Reference: {reference_side}, waiting for '{waiting_for}' to reach 5 cm.")

    # Detection and tracking
    at_target_position = False
    locked_color = None
    total_detections = 0
    blue_count = 0
    orange_count = 0
    motor_time = 1.5

    # Color detection thresholds
    blue_threshold = 900
    orange_threshold = 900
    color_margin = 100

    while total_detections < 12:
        # Read TOF distances
        left_distance = sensor_left.range / 10
        right_distance = sensor_right.range / 10
        distance_diff = abs(left_distance - right_distance)

        # Read relevant color sensor channels (in lux or detected light levels)
        blue = color_sensor.channel_480nm or 0
        orange = color_sensor.channel_630nm or 0

        # Debugging the color sensor readings
        print(f"Color Sensor Readings → 480nm (Blue): {blue} | 630nm (Orange): {orange}")

        # Bar graph for color readings
        print("F3 - 480nm//Blue   %s" % bar_graph(blue))
        print("F7 - 630nm/Orange  %s" % bar_graph(orange))

        # Decision logic variables
        turn_decision = None
        color_detected = None

        print("\n--- Iteration Report --------------------------------------")
        print(f"TOF Distances → Left: {left_distance:.1f} cm | Right: {right_distance:.1f} cm")
        print(f"Distance Difference: {distance_diff:.1f} cm")
        
        if blue > orange and blue > blue_threshold and (blue - orange > color_margin):
            if locked_color != "blue":
                locked_color = "blue"
                print("🔵 Color Lock Updated → BLUE")
            color_detected = "blue"
            turn_decision = "right"
            print("Action: Blue detected! Turning RIGHT.")
            turn_right(motor_time)
            blue_count += 1
            total_detections += 1

        elif orange > blue and orange > orange_threshold and (orange - blue > color_margin):
            if locked_color != "orange":
                locked_color = "orange"
                print("Color Lock Updated → ORANGE")
            color_detected = "orange"
            turn_decision = "left"
            print("Action: Orange detected! Turning LEFT.")
            turn_left(motor_time)
            orange_count += 1
            total_detections += 1

        else:
            # No strong color detected; fallback to TOF decision
            if left_distance > right_distance:
                turn_decision = "right"
                print("No color lock. Based on TOF → Turning RIGHT.")
                turn_right(motor_time)
            else:
                turn_decision = "left"
                print("No color lock. Based on TOF → Turning LEFT.")
                turn_left(motor_time)

        # Position tracking
        if not at_target_position:
            print(f"Target Check → Waiting for '{waiting_for}' to reach ≤ 5 cm.")
            if (waiting_for == "left" and left_distance <= 5) or (waiting_for == "right" and right_distance <= 5):
                print("Target position reached!")
                kit.continuous_servo[1].throttle = 0
                at_target_position = True
            else:
                print("Moving forward toward target...")
                kit.continuous_servo[1].throttle = 0.5
                time.sleep(0.5)
                kit.continuous_servo[1].throttle = 0
        else:
            print("Holding position. Monitoring for color...")

        print(f"Servo Angle: {kit.servo[0].angle if kit.servo[0].angle else 'N/A'}")
        print(f"Motor Throttle: {kit.continuous_servo[1].throttle}")
        print(f"Status → Blue: {blue_count}, Orange: {orange_count}, Total: {total_detections}")
        print("--------------------------------------------------------------")
        time.sleep(1)

    print("12 detections complete. Shutting down.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    kit.continuous_servo[1].throttle = 0
    print("Motors off. Program ended.")

