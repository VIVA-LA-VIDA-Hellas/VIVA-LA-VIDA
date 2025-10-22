import smbus2
import time
import threading
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2
from pca9685_control import set_servo_angle, set_motor_speed
import board
import busio

# ================== Common Constants ==================
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0

CENTER_ANGLE = 90
TURN_ANGLE = 120
CENTER_DEADZONE = 5
SERVO_STEP = 6.0

set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE
Turn = "No"

# ==== CAMERA CALIBRATION PARAMETERS ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== START CAMERA ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

# ================== MPU6050 Setup ==================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
GYRO_ZOUT_H = 0x47
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_gyro_z_bias(samples=200):
    total = 0.0
    for _ in range(samples):
        raw = read_raw_data(GYRO_ZOUT_H)
        total += raw / 131.0
        time.sleep(0.005)
    return total / samples

mpu6050_init()
print("Measuring gyro bias... Keep sensor still.")
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

# ================== GPIO Setup ==================
GPIO.setmode(GPIO.BCM)
TRIG_FRONT, ECHO_FRONT = 22, 23
TRIG_LEFT, ECHO_LEFT = 27, 17
for trig, echo in [(TRIG_FRONT, ECHO_FRONT), (TRIG_LEFT, ECHO_LEFT)]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trig, GPIO.LOW)
time.sleep(0.2)

def measure_distance(trigger_pin, echo_pin, timeout=0.03, retries=2):
    for _ in range(retries):
        GPIO.output(trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trigger_pin, GPIO.LOW)
        start_time = time.time()
        pulse_start = start_time
        while GPIO.input(echo_pin) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start - start_time > timeout:
                break
        else:
            pulse_end = time.time()
            while GPIO.input(echo_pin) == GPIO.HIGH:
                if time.time() - pulse_end > timeout:
                    break
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 34300 / 2
            return distance
        time.sleep(0.01)
    return None

# ================== States ==================
STATE_FORWARD = 0
STATE_BACK_AND_TURN = 1
state = STATE_FORWARD

def move_forward(speed=25):
    set_motor_speed(MOTOR_FWD, MOTOR_REV, speed)

def move_backward(speed=25):
    set_motor_speed(MOTOR_REV, MOTOR_FWD, speed)

def stop_motors():
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)

def y_at_center(lines):
    if lines is None:
        return -1
    y_values = []
    for x1, y1, x2, y2 in lines[:, 0]:
        if x1 != x2:
            slope = (y2 - y1) / (x2 - x1)
            y = slope * (center_x - x1) + y1
            y_values.append(y)
        elif x1 == center_x:
            y_values.append(max(y1, y2))
    return max(y_values) if y_values else -1

# ==== COLOR DETECTION HELPERS (from obstacle code) ====
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    return largest, (x, y), (x + w, y + h), w*h

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

# ==== Obstacle Avoidance State ====
in_obstacle_avoid = False
avoid_start_time = None
avoid_direction = None
AVOID_DURATION = 1.5
AVOID_SPEED = 15

# ================== Main Loop ==================
try:
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # --- IMU yaw ---
        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # --- CAMERA capture ---
        imageog = picam2.capture_array()
        img = imageog.copy()
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        center_x = img.shape[1] // 2

        # --- Blue/Orange for turning ---
        orange_lower = np.array([0, 70, 150])
        orange_upper = np.array([20, 200, 230])
        mask_orange = cv2.inRange(imgHSV, orange_lower, orange_upper)
        blue_lower = np.array([70, 0, 130])
        blue_upper = np.array([125, 170, 170])
        mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper)

        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue = cv2.Canny(mask_blue, 50, 150)

        lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi / 180, 50, 50, 10)
        lines_blue = cv2.HoughLinesP(edges_blue, 1, np.pi / 180, 50, 50, 10)

        orange_y = y_at_center(lines_orange)
        blue_y = y_at_center(lines_blue)

        if orange_y > blue_y and orange_y > 400:
            Turn = "Right"
            TURN_ANGLE = 120
        elif blue_y > orange_y and blue_y > 400:
            Turn = "Left"
            TURN_ANGLE = 60
        else:
            Turn = "No"

        # --- Red/Green for obstacles ---
        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))

        red_data = get_largest_contour(mask_red)
        green_data = get_largest_contour(mask_green)

        car_width, car_height = 200, 80
        bottom_y = img.shape[0] - 10
        car_top_left = (center_x - car_width // 2, bottom_y - car_height)
        car_bottom_right = (center_x + car_width // 2, bottom_y)
        car_box = (*car_top_left, *car_bottom_right)

        # --- Obstacle Avoid Logic ---
        if not in_obstacle_avoid:
            if red_data and boxes_intersect(car_box, (*red_data[1], *red_data[2])):
                in_obstacle_avoid = True
                avoid_direction = "right"
                avoid_start_time = time.time()
                set_servo_angle(SERVO_CHANNEL, 70)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
            elif green_data and boxes_intersect(car_box, (*green_data[1], *green_data[2])):
                in_obstacle_avoid = True
                avoid_direction = "left"
                avoid_start_time = time.time()
                set_servo_angle(SERVO_CHANNEL, 110)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)

        if in_obstacle_avoid:
            if time.time() - avoid_start_time >= AVOID_DURATION:
                in_obstacle_avoid = False
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 20)
            else:
                continue  # skip the rest of the logic while avoiding

        # --- Ultrasonic ---
        dist_front = measure_distance(TRIG_FRONT, ECHO_FRONT)
        dist_left = measure_distance(TRIG_LEFT, ECHO_LEFT)

        print(f"Yaw={current_yaw:.2f}° | Front={dist_front}cm | Left={dist_left}cm | State={state} | Turn={Turn}")

        # --- Movement Logic ---
        if state == STATE_FORWARD:
            move_forward(20)

            if Turn == "Left":
                set_servo_angle(SERVO_CHANNEL, 60)
                with yaw_lock:
                    yaw = 0.0
                state = STATE_BACK_AND_TURN

            if Turn == "Right":
                set_servo_angle(SERVO_CHANNEL, 120)
                with yaw_lock:
                    yaw = 0.0
                state = STATE_BACK_AND_TURN

        elif state == STATE_BACK_AND_TURN:
            move_forward(20)
            set_servo_angle(SERVO_CHANNEL, TURN_ANGLE)
            if abs(current_yaw) >= 80:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                move_forward(20)
                with yaw_lock:
                    yaw = 0.0
                state = STATE_FORWARD

        time.sleep(0.02)

except KeyboardInterrupt:
    stop_motors()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    GPIO.cleanup()
    print("Stopped and reset.")
