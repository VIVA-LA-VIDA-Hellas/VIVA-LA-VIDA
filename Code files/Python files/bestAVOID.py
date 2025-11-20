import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import smbus2
import threading

# ==== CONSTANTS ====
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR = 95
LEFT_NEAR = 130
RIGHT_FAR = 85
RIGHT_NEAR = 50

STEP = 3
FAST_SERVO_STEP = 6
SERVO_UPDATE_DELAY = 0.02

MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

# ==== BLUE-BOX BACKWARD LOGIC ====
blue_backward_start = None
in_blue_backward = False
BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 15

# ==== AVOIDANCE LOGIC ====
AVOID_BACK_DURATION = 1.0
AVOID_SPEED = 15

# ==== NORMAL LINE FOLLOWING SPEED ====
NORMAL_SPEED = 13

# ==== IMU SETUP ====
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47
bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

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
print("Measuring gyro bias, keep sensor still...")
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

# ==== YAW TRACKING ====
yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

def reset_yaw_listener():
    global yaw
    while True:
        input("Press ENTER to reset yaw to 0°")
        with yaw_lock:
            yaw = 0.0
            print("Yaw reset to 0°")

threading.Thread(target=reset_yaw_listener, daemon=True).start()

# ==== SERVO SETUP ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA SETUP ====
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ==== FUNCTIONS ====
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest_contour)
    return largest_contour, (x, y), (x + w, y + h), w*h

def compute_servo_angle(color, area):
    norm_area = max(MIN_AREA, min(MAX_AREA, area))
    closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)
    if color == "Red":
        return int(LEFT_FAR + closeness * (LEFT_NEAR - LEFT_FAR))
    else:
        return int(RIGHT_FAR - closeness * (RIGHT_FAR - RIGHT_NEAR))

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

# ==== STATE VARIABLES ====
last_color = None
frame_count = 0
last_update_time = time.time()
state = "normal"
state_start = time.time()

try:
    motors_started = True
    avoidance_mode = False
    avoid_direction = None
    avoid_start_time = None

    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    current_servo_angle = CENTER_ANGLE
    last_update_time = time.time()

    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # ==== READ IMU ====
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt

        # ==== CAMERA & MASKS ====
        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        scale = 1080 / h_img
        img_small = cv2.resize(img, (int(w_img * scale), 1080))
        imgHSV = cv2.cvtColor(img_small, cv2.COLOR_RGB2HSV)

        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))

        img_contours = img_small.copy()
        boxes = []

        red_data = get_largest_contour(mask_red)
        if red_data:
            _, top_left, bottom_right, area = red_data
            boxes.append(("Red", area, (*top_left, *bottom_right)))

        green_data = get_largest_contour(mask_green)
        if green_data:
            _, top_left, bottom_right, area = green_data
            boxes.append(("Green", area, (*top_left, *bottom_right)))

        # ==== CAR BOX ====
        center_x = img_contours.shape[1] // 2
        car_width, car_height = 350, 100
        bottom_y = img_contours.shape[0] - 10
        car_box = (center_x - car_width//2, bottom_y - car_height,
                   center_x + car_width//2, bottom_y)

        target_angle = CENTER_ANGLE
        motor_speed = 0

        # ==== BLUE-BACKWARD LOGIC ====
        if not in_blue_backward:
            if red_data and boxes_intersect(car_box, (*red_data[1], *red_data[2])):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = LEFT_NEAR
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
            elif green_data and boxes_intersect(car_box, (*green_data[1], *green_data[2])):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = RIGHT_NEAR
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)

        if in_blue_backward:
            step = FAST_SERVO_STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)

            if time.time() - blue_backward_start >= BLUE_BACK_DURATION:
                in_blue_backward = False
                motor_speed = NORMAL_SPEED
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                continue

        # ==== AVOIDANCE LOGIC ====
        if motors_started and not avoidance_mode:
            for color, _, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode = True
                    avoid_direction = "right" if color == "Green" else "left"
                    avoid_start_time = time.time()
                    motor_speed = AVOID_SPEED
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -motor_speed)
                    target_angle = RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR
                    state = "avoid"
                    state_start = time.time()
                    break

        if avoidance_mode:
            elapsed = time.time() - avoid_start_time
            if elapsed < AVOID_BACK_DURATION:
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                target_angle = CENTER_ANGLE
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    state = "normal"

        # ==== NORMAL LINE-FOLLOWING & IMU CORRECTION ====
        if not avoidance_mode and state == "normal":
            motor_speed = NORMAL_SPEED
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

            # default target is center
            target_angle = CENTER_ANGLE

            if boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]

                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    # base angle from color
                    if chosen_color == "Red":
                        color_angle = LEFT_NEAR
                        target_angle = color_angle + int(yaw * 2)  # IMU assists left turn
                    elif chosen_color == "Green":
                        color_angle = RIGHT_NEAR
                        target_angle = color_angle - int(yaw * 2)  # IMU assists right turn
                    else:
                        target_angle = compute_servo_angle(chosen_color, chosen_area)

            # clamp servo limits
            target_angle = max(60, min(130, target_angle))


        # ==== SERVO SMOOTHING ====
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if avoidance_mode else STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            # clamp final servo value
            current_servo_angle = max(60, min(130, current_servo_angle))
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        # ==== SHOW RESULTS ====
        cv2.imshow('Contours', img_contours)    
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
