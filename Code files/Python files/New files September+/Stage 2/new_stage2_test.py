import smbus2
import time
import threading
import RPi.GPIO as GPIO
from pca9685_control import set_servo_angle, set_motor_speed
from picamera2 import Picamera2
import cv2
import numpy as np
# ================== Common Constants ==================
MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2   # reverse channel
SERVO_CHANNEL = 0

time.sleep(0.2)

# ==== CAMERA CALIBRATION PARAMETERS FOR FISHEYE CORRECTION ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== START CAMERA ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()  

Turn = "No"

# ================== MPU6050 Setup ==================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
GYRO_ZOUT_H  = 0x47

bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low  = bus.read_byte_data(MPU6050_ADDR, addr+1)
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

# ================== Yaw Tracking ==================
yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

CENTER_ANGLE = 90
TURN_ANGLE = 120
CENTER_DEADZONE = 5
current_servo_angle = CENTER_ANGLE
SERVO_STEP = 6.0

set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)

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

# ================== Helper Motor Functions ==================
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
        if x1 != x2:  # avoid vertical divide-by-zero
            slope = (y2 - y1) / (x2 - x1)
            y = slope * (center_x - x1) + y1
            y_values.append(y)
        else:
            # vertical line, take min/max depending on which side of center
            if x1 == center_x:
                y_values.append(max(y1, y2))
    return max(y_values) if y_values else -1

# ================== Main Loop ==================
try:
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Gyro read
        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw
        
        # Capture frame
        imageog = picam2.capture_array()
        img = imageog.copy()  # Already RGB from Picamera2

        # Fisheye correction (optional)
        DIM = img.shape[1], img.shape[0]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), K, DIM, cv2.CV_16SC2
        )
        # img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        # Convert to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Orange mask
        orange_lower = np.array([0, 110, 110])
        orange_upper = np.array([20, 160, 230])
        mask_orange = cv2.inRange(imgHSV, orange_lower, orange_upper)

        # Blue mask
        blue_lower = np.array([100, 60, 80])
        blue_upper = np.array([120, 120, 170])
        mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper)

        # Edge detection
        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue = cv2.Canny(mask_blue, 50, 150)

        # Detect lines using Hough Transform
        lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        lines_blue = cv2.HoughLinesP(edges_blue, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        img_lines = img.copy()

        # Draw lines for visualization
        if lines_orange is not None:
            for x1, y1, x2, y2 in lines_orange[:, 0]:
                cv2.line(img_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)

        if lines_blue is not None:
            for x1, y1, x2, y2 in lines_blue[:, 0]:
                cv2.line(img_lines, (x1, y1), (x2, y2), (0, 165, 255), 2)

        # ---- Decide based on which line is lower at the center x ----
        center_x = img.shape[1] // 2

        orange_y = y_at_center(lines_orange)
        blue_y = y_at_center(lines_blue)

        if orange_y > blue_y:
            print("Turn right (orange line lower at center)")
            Turn = "Right"
        elif blue_y > orange_y:
            print("Turn left (blue line lower at center)")
            Turn = "Left"
        else:
            print("No line detected at center")
            Turn = "No"


        # Ultrasonic distances
        dist_front = measure_distance(TRIG_FRONT, ECHO_FRONT)
        dist_left  = measure_distance(TRIG_LEFT, ECHO_LEFT)

        print(f"Yaw={current_yaw:.2f}° | Front={dist_front}cm | Left={dist_left}cm | State={state}")

        if state == STATE_FORWARD:
            # Straight motion
            move_forward(25)

            # Slight auto-centering logic from IMU
            if -CENTER_DEADZONE <= current_yaw <= CENTER_DEADZONE:
                target_servo = CENTER_ANGLE
            elif current_yaw > CENTER_DEADZONE:
                target_servo = CENTER_ANGLE + 5  # small correction left
            else:
                target_servo = CENTER_ANGLE - 5  # small correction right

            # Smooth servo motion
            if current_servo_angle < target_servo:
                current_servo_angle += SERVO_STEP
                if current_servo_angle > target_servo:
                    current_servo_angle = target_servo
            elif current_servo_angle > target_servo:
                current_servo_angle -= SERVO_STEP
                if current_servo_angle < target_servo:
                    current_servo_angle = target_servo
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)

            # Check condition to start reversing
            if turn == "Left":
            #(dist_front is not None and dist_left is not None and
                #dist_front < 20 and dist_left > 80):
                print("Obstacle ahead + open left side → reverse-turn mode")
                set_servo_angle(SERVO_CHANNEL, TURN_ANGLE)
                with yaw_lock:
                    yaw = 0.0
                state = STATE_BACK_AND_TURN

        elif state == STATE_BACK_AND_TURN:
            move_backward(25)
            set_servo_angle(SERVO_CHANNEL, TURN_ANGLE)

            # Detect 90° turn via IMU
            if abs(current_yaw) >= 90:
                print(f"90° turn complete (Yaw={current_yaw:.2f}°) → go straight")
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                move_forward(25)
                with yaw_lock:
                    yaw = 0.0
                state = STATE_FORWARD

        time.sleep(0.02)

except KeyboardInterrupt:
    stop_motors()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    GPIO.cleanup()
    print("Stopped and reset.")
