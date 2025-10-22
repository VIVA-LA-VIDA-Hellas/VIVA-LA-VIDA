import cv2
import numpy as np
from picamera2 import Picamera2
import time
import smbus2
import threading
from pca9685_control import set_servo_angle, set_motor_speed

# ================== CONSTANTS ==================
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR = 95
LEFT_NEAR = 110
RIGHT_FAR = 85
RIGHT_NEAR = 70
KP = 0.5
STEP = 5
SERVO_UPDATE_DELAY = 0.04
MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5
BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 15
CENTER_DEADZONE = 5
FAST_SERVO_STEP = 4
BLUE_TURN_THRESHOLD_Y = 400  # For blue/orange intersection detection

# ================== SERVO INITIALIZATION ==================
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ================== CAMERA SETUP ==================
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ================== MPU6050 SETUP ==================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
GYRO_ZOUT_H  = 0x47
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low  = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_gyro_z_bias(samples=200):
    total = 0.0
    for _ in range(samples):
        total += read_raw_data(GYRO_ZOUT_H) / 131.0
        time.sleep(0.005)
    return total / samples

mpu6050_init()
print("Measuring gyro bias... Keep sensor still.")
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

# ================== HELPER FUNCTIONS ==================
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest_contour)
    return largest_contour, (x, y), (x + w, y + h), w * h

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

def imu_realign(target_angle=90, speed=15):
    """Use IMU yaw to straighten robot after avoidance."""
    global yaw, last_time
    print("Starting IMU realignment...")
    with yaw_lock:
        yaw = 0.0
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw
        if abs(current_yaw) <= CENTER_DEADZONE:
            print(f"Aligned (Yaw={current_yaw:.2f}°)")
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
            set_servo_angle(SERVO_CHANNEL, target_angle)
            break
        if current_yaw > CENTER_DEADZONE:
            set_servo_angle(SERVO_CHANNEL, target_angle + 5)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, speed)
        elif current_yaw < -CENTER_DEADZONE:
            set_servo_angle(SERVO_CHANNEL, target_angle - 5)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, speed)
        time.sleep(0.02)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    set_servo_angle(SERVO_CHANNEL, target_angle)
    print("IMU correction complete.")

def y_at_center(lines, center_x):
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

# ================== VARIABLES ==================
blue_backward_start = None
in_blue_backward = False
avoidance_mode = False
Turn = "No"
state = "FORWARD"
in_turn = False
motor_speed = 0
set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
time.sleep(0.2)

# ================== MAIN LOOP ==================
try:
    while True:
        # === Update IMU Yaw ===
        now = time.time()
        dt = now - last_time
        last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # === Capture Image ===
        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        scale = 1080 / h_img
        img_small = cv2.resize(img, (int(w_img * scale), 1080))
        imgHSV = cv2.cvtColor(img_small, cv2.COLOR_RGB2HSV)
        img_contours = img_small.copy()

        # === RED / GREEN AVOIDANCE ===
        mask_red1 = cv2.inRange(imgHSV, np.array([0, 100, 80]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170, 100, 80]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35, 60, 60]), np.array([95, 255, 255]))
        red_data = get_largest_contour(mask_red)
        green_data = get_largest_contour(mask_green)

        center_x = img_contours.shape[1] // 2
        car_width, car_height = 350, 100
        bottom_y = img_contours.shape[0] - 10
        car_top_left = (center_x - car_width // 2, bottom_y - car_height)
        car_bottom_right = (center_x + car_width // 2, bottom_y)
        car_box = (*car_top_left, *car_bottom_right)
        cv2.rectangle(img_contours, car_top_left, car_bottom_right, (255, 0, 0), -1)

        # === BLUE / ORANGE LINES ===
        orange_lower = np.array([0, 70, 150])
        orange_upper = np.array([20, 200, 230])
        mask_orange = cv2.inRange(imgHSV, orange_lower, orange_upper)
        blue_lower = np.array([70, 0, 130])
        blue_upper = np.array([125, 170, 170])
        mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper)

        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue = cv2.Canny(mask_blue, 50, 150)
        lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi/180, 50, 50, 10)
        lines_blue = cv2.HoughLinesP(edges_blue, 1, np.pi/180, 50, 50, 10)

        orange_y = y_at_center(lines_orange, center_x)
        blue_y = y_at_center(lines_blue, center_x)

        # === Check for red/green obstacle ===
        if not avoidance_mode:
            if red_data and boxes_intersect(car_box, red_data[1] + red_data[2]):
                avoidance_mode = True
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                blue_backward_start = time.time()
            elif green_data and boxes_intersect(car_box, green_data[1] + green_data[2]):
                avoidance_mode = True
                set_servo_angle(SERVO_CHANNEL, LEFT_NEAR)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                blue_backward_start = time.time()

        if avoidance_mode:
            if time.time() - blue_backward_start >= BLUE_BACK_DURATION:
                avoidance_mode = False
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
                imu_realign(CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 20)
            else:
                cv2.imshow('Contours', img_contours)
                if cv2.waitKey(1) in [27, ord('q')]:
                    break
                continue  # skip track logic while avoiding

        # === BLUE/ORANGE TRACK TURN LOGIC ===
        if state == "FORWARD":
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 25)
            if orange_y > blue_y and orange_y > BLUE_TURN_THRESHOLD_Y:
                Turn = "Right"
                with yaw_lock:
                    yaw = 0.0
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR)
                state = "TURNING"
            elif blue_y > orange_y and blue_y > BLUE_TURN_THRESHOLD_Y:
                Turn = "Left"
                with yaw_lock:
                    yaw = 0.0
                set_servo_angle(SERVO_CHANNEL, LEFT_NEAR)
                state = "TURNING"

        elif state == "TURNING":
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 20)
            if abs(current_yaw) >= 80:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                with yaw_lock:
                    yaw = 0.0
                state = "FORWARD"

        # === DISPLAY ===
        cv2.putText(img_contours, f"Yaw={current_yaw:.2f}  Turn={Turn}  Avoid={avoidance_mode}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow('Contours', img_contours)
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    print("Stopped and reset.")
