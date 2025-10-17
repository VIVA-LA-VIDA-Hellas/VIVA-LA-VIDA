import cv2
import numpy as np
import time
import threading
import smbus2
from picamera2 import Picamera2
from pca9685_control import set_servo_angle, set_motor_speed

# ================== HARDWARE CONSTANTS ==================
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR = 85
LEFT_NEAR = 55
RIGHT_FAR = 95
RIGHT_NEAR = 120

STEP = 8
SERVO_UPDATE_DELAY = 0.04

MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

MOTOR_FWD = 2
MOTOR_REV = 1
MOTOR_SPEED = 25  # motor always running

# ================== IMU CONSTANTS ==================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
GYRO_ZOUT_H = 0x47

IMU_LEFT_ANGLE = 120
IMU_RIGHT_ANGLE = 60
IMU_MAX_CORRECTION = 60
IMU_DAMPING_FACTOR = 0.6
IMU_BLEND_DURATION = 0.3
BLEND_FACTOR = 0.3

# ================== INITIAL SETUP ==================
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
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

# Servo setup
current_servo_angle = CENTER_ANGLE
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
set_motor_speed(MOTOR_FWD, MOTOR_REV, MOTOR_SPEED)

# Camera setup
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ================== HELPER FUNCTIONS ==================
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    return largest, (x, y), (x + w, y + h), w * h

def compute_servo_angle(color, area, center_x, box_center_x):
    # Vision proportional control with line-following adjustment
    norm_area = max(MIN_AREA, min(MAX_AREA, area))
    closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)  # 0..1
    if color == "Red":
        base_angle = RIGHT_FAR + closeness * (RIGHT_NEAR - RIGHT_FAR)
        # line-following adjustment: robot right of red box → turn left
        if box_center_x < center_x:
            base_angle = LEFT_NEAR
    else:  # Green
        base_angle = LEFT_FAR - closeness * (LEFT_FAR - LEFT_NEAR)
        # line-following adjustment: robot left of green box → turn right
        if box_center_x > center_x:
            base_angle = RIGHT_NEAR
    return int(base_angle)

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

# Manual yaw reset
def reset_yaw_listener():
    global yaw
    while True:
        input("Press ENTER to reset yaw to 0°\n")
        with yaw_lock:
            yaw = 0.0
            print("Yaw reset to 0° (manual)")

threading.Thread(target=reset_yaw_listener, daemon=True).start()

# ================== VARIABLES ==================
last_color = None
frame_count = 0
last_update_time = time.time()
last_vision_angle = CENTER_ANGLE
last_vision_time = time.time()

# ================== MAIN LOOP ==================
try:
    print("Starting main loop. Press 'q' or ESC to quit.")
    while True:
        loop_start = time.time()

        # Update yaw
        now = time.time()
        dt = now - last_time
        last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # Capture image
        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        center_x = w_img // 2

        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Masks
        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))

        # Detect boxes
        red_data = get_largest_contour(mask_red)
        green_data = get_largest_contour(mask_green)
        boxes = []
        if red_data:
            _, tl, br, area = red_data
            boxes.append(("Red", area, (*tl, *br)))
        if green_data:
            _, tl, br, area = green_data
            boxes.append(("Green", area, (*tl, *br)))

        # Car box for intersection
        car_width = 800
        car_height = 225
        bottom_y = h_img - 10
        car_top_left = (center_x - car_width // 2, bottom_y - car_height)
        car_bottom_right = (center_x + car_width // 2, bottom_y)
        car_box = (*car_top_left, *car_bottom_right)

        # ===== Decide target angle =====
        target_angle = CENTER_ANGLE
        vision_mode = len(boxes) > 0

        # Intersection check
        intersection_triggered = False
        for _, _, box_coords in boxes:
            if boxes_intersect(car_box, box_coords):
                target_angle = CENTER_ANGLE
                intersection_triggered = True
                break

        # Vision proportional control
        if not intersection_triggered and boxes:
            boxes.sort(key=lambda b: b[1], reverse=True)
            chosen_color, chosen_area, box_coords = boxes[0]
            box_center_x = (box_coords[0] + box_coords[2]) // 2

            if last_color == chosen_color:
                frame_count += 1
            else:
                frame_count = 0
                last_color = chosen_color

            if frame_count >= COLOR_HOLD_FRAMES:
                target_angle = compute_servo_angle(chosen_color, chosen_area, center_x, box_center_x)

            # Store last vision angle/time for blending
            last_vision_angle = target_angle
            last_vision_time = time.time()
        else:
            frame_count = 0
            last_color = None

        # IMU proportional control
        limited_yaw = max(-IMU_MAX_CORRECTION, min(IMU_MAX_CORRECTION, current_yaw))
        yaw_proportion = limited_yaw / IMU_MAX_CORRECTION
        imu_target_angle = CENTER_ANGLE + yaw_proportion * (IMU_LEFT_ANGLE - IMU_RIGHT_ANGLE) * IMU_DAMPING_FACTOR
        imu_target_angle = int(max(LEFT_NEAR, min(RIGHT_NEAR, imu_target_angle)))

        # Blend vision & IMU
        time_since_vision = time.time() - last_vision_time
        if vision_mode:
            blended_target = last_vision_angle
        elif time_since_vision < IMU_BLEND_DURATION:
            blended_target = int(last_vision_angle * (1 - BLEND_FACTOR) + imu_target_angle * BLEND_FACTOR)
        else:
            blended_target = imu_target_angle

        # Smooth servo movement
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            if abs(current_servo_angle - blended_target) > STEP:
                current_servo_angle += STEP if current_servo_angle < blended_target else -STEP
            else:
                current_servo_angle = blended_target
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        # Motor always on
        set_motor_speed(MOTOR_FWD, MOTOR_REV, MOTOR_SPEED)

        # Visualization
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Green Mask", mask_green)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    print("Stopped and reset.")
