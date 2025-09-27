'''First mission with distance sensors for direction detection
 following middle lane canny for path-not working'''

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import board
import busio
import digitalio
from adafruit_servokit import ServoKit
from adafruit_motorkit import MotorKit
import adafruit_vl53l0x

# Initialize I2C and sensors
i2c = busio.I2C(board.SCL, board.SDA)

# Setup VL53L0X ToF Sensors (left and right)
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

# Initialize servo kit for steering
kit_servo = ServoKit(channels=8)
kit_servo.servo[0].angle = 90  # Center the steering at start

# Initialize motor kit for DC motor control
kit_motor = MotorKit()

# Motor control functions
def rotate_motor_forward():
    print("Rotating motor forward")
    kit_motor.motor3.throttle = 0.3  #speed forward

def rotate_motor_backward():
    print("Rotating motor backward")
    kit_motor.motor3.throttle = -0.3  #speed backward

def stop_motor():
    print("Stopping motor")
    kit_motor.motor3.throttle = 0.0  # Stop the motor

# Function to turn slightly left
def turn_left_slightly():
    kit_servo.servo[0].angle = 130  # Adjust the servo to slightly turn left
    rotate_motor_forward()

# Function to turn slightly right
def turn_right_slightly():
    kit_servo.servo[0].angle = 70  # Adjust the servo to slightly turn right
    rotate_motor_forward()

# PID setup for steering
kP = 0.12
kI = 0.0008
kD = 0.4
previous_error = 0
integral = 0

def empty(x):
    pass

# Initial values for HSV trackbars
h_min, h_max = 3, 18
s_min, s_max = 220, 255
v_min, v_max = 0, 255

# Initialize Picamera2 for capturing frames
picam2 = Picamera2()
picam2.start()

# Give the camera a moment to adjust settings
time.sleep(2)

# Start moving the rover forward
rotate_motor_forward()

while True:
    # Get TOF sensor distances
    left_distance = sensor_left.range / 10  # Convert mm to cm
    right_distance = sensor_right.range / 10

    # Priority Check for Obstacle Avoidance (Distance Sensors)
    if left_distance < 15:
        print("Left distance < 15 cm, turning slightly right")
        turn_right_slightly()  # Turn slightly right to avoid wall on the left
        continue  # Skip lane-following code when obstacle is detected
    elif right_distance < 15:
        print("Right distance < 15 cm, turning slightly left")
        turn_left_slightly()  # Turn slightly left to avoid wall on the right
        continue  # Skip lane-following code when obstacle is detected

    # Capture frame using picamera2
    img = picam2.capture_array()
    
    # Rotate image by 180 degrees
    img = cv2.flip(img, -1)

    # Convert the image from RGB to BGR for OpenCV compatibility
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Convert the image to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask based on the HSV range values
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)

    # Create masks for White and Black
    white_lower = np.array([0, 0, 40])
    white_upper = np.array([255, 100, 255])
    mask_white = cv2.inRange(imgHSV, white_lower, white_upper)

    black_lower = np.array([0, 0, 0])
    black_upper = np.array([255, 255, 40])
    mask_black = cv2.inRange(imgHSV, black_lower, black_upper)

    # Edge detection on masks
    edges_white = cv2.Canny(mask_white, 100, 100)
    edges_black = cv2.Canny(mask_black, 100, 100)

    # Initialize variables for lane tracking
    height, width = edges_black.shape
    left_edges = []
    right_edges = []

    # Loop through each row to find edges
    for i in range(height):
        row = edges_black[i, :]

        # Detect leftmost edge in the left half
        left = np.where(row[:width // 2] > 0)[0]
        if len(left) > 0:
            left_edges.append(left[-1])  # Take the last detected edge on the left side
        else:
            left_edges.append(None)  # No edge detected

        # Detect rightmost edge in the right half
        right = np.where(row[width // 2:] > 0)[0]
        if len(right) > 0:
            right_edges.append(right[0] + (width // 2))  # Adjust index for right side
        else:
            right_edges.append(None)  # No edge detected

    # Draw lines based on detected edges
    for i in range(height):
        left_avg = left_edges[i]
        right_avg = right_edges[i]

        if left_avg is not None and right_avg is not None:
            # Calculate the midpoint and draw the path
            middle_line = (left_avg + right_avg) // 2

            # Draw the path line (white) with increased thickness
            cv2.line(edges_black, (middle_line, i), (middle_line, i), (255, 255, 255), 3)  # Increased thickness

            # PID for steering
            frame_center = width // 2
            error = middle_line - frame_center
            integral += error
            derivative = error - previous_error

            angle_offset = int(kP * error + kI * integral + kD * derivative)
            steerinq_angle = max(45, min(135, 90 + angle_offset))

            kit_servo.servo[0].angle = steerinq_angle
            previous_error = error

    # Show edges with path lines drawn
    cv2.imshow("Path on Edges", edges_black)

    # Show the input image (no turn message)
    cv2.imshow("Input", img)

    # Break the loop when 'q' or 'Esc' is pressed
    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
        break

# Stop the motor when the program ends
stop_motor()

# Release the video capture object and close windows
picam2.stop()
cv2.destroyAllWindows()
