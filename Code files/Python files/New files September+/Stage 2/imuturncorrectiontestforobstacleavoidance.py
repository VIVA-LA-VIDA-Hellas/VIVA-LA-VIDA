import cv2
import numpy as np
from picamera2 import Picamera2
import time
import smbus2
import threading
from pca9685_control import set_servo_angle, set_motor_speed


# ==== CONSTANTS ====
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR = 95
LEFT_NEAR = 110
RIGHT_FAR = 85
RIGHT_NEAR = 70
LEFT_CENTER = 65
RIGHT_CENTER = 115
KP = 0.5
STEP = 5
SERVO_UPDATE_DELAY = 0.04
MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5
BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 15

# ==== SERVO SETUP ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA SETUP ====
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ==== MPU6050 SETUP ====
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
CENTER_DEADZONE = 5


# ==== FUNCTIONS ====
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest_contour)
    return largest_contour, (x, y), (x + w, y + h), w * h


def compute_servo_angle(color, area):
    norm_area = max(MIN_AREA, min(MAX_AREA, area))
    closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)
    if color == "Red":
        return int(RIGHT_FAR + closeness * (RIGHT_NEAR - RIGHT_FAR))
    else:
        return int(LEFT_FAR - closeness * (LEFT_FAR - LEFT_NEAR))


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


# ==== VARIABLES ====
blue_backward_start = None
in_blue_backward = False
last_color = None
frame_count = 0
last_update_time = time.time()
state = "normal"
state_start = time.time()
HOLD_DURATION = 1.0
motors_started = False
avoidance_mode = False
avoid_direction = None
avoid_start_time = None
FAST_SERVO_STEP = 4

set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
time.sleep(0.2)


# ==== MAIN LOOP ====
try:
    while True:
        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        scale = 1080 / h_img
        img_small = cv2.resize(img, (int(w_img * scale), 1080))
        imgHSV = cv2.cvtColor(img_small, cv2.COLOR_RGB2HSV)

        # ==== MASKS ====
        mask_red1 = cv2.inRange(imgHSV, np.array([0, 100, 80]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170, 100, 80]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35, 60, 60]), np.array([95, 255, 255]))

        img_contours = img_small.copy()
        boxes = []

        red_data = get_largest_contour(mask_red)
        if red_data:
            _, top_left, bottom_right, area = red_data
            cv2.rectangle(img_contours, top_left, bottom_right, (0, 0, 255), 2)
            cv2.putText(img_contours, "Red Box", (top_left[0], top_left[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            boxes.append(("Red", area, (*top_left, *bottom_right)))

        green_data = get_largest_contour(mask_green)
        if green_data:
            _, top_left, bottom_right, area = green_data
            cv2.rectangle(img_contours, top_left, bottom_right, (0, 255, 0), 2)
            cv2.putText(img_contours, "Green Box", (top_left[0], top_left[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            boxes.append(("Green", area, (*top_left, *bottom_right)))

        # ==== CAR BOX ====
        center_x = img_contours.shape[1] // 2
        car_width, car_height = 350, 100
        bottom_y = img_contours.shape[0] - 10
        car_top_left = (center_x - car_width // 2, bottom_y - car_height)
        car_bottom_right = (center_x + car_width // 2, bottom_y)
        car_box = (*car_top_left, *car_bottom_right)
        cv2.rectangle(img_contours, car_top_left, car_bottom_right, (255, 0, 0), -1)

        target_angle = CENTER_ANGLE
        motor_speed = 0

        # ==== BACKWARD LOGIC ====
        if not in_blue_backward:
            if red_data and boxes_intersect(car_box, red_data[1] + red_data[2]):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = RIGHT_NEAR
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
            elif green_data and boxes_intersect(car_box, green_data[1] + green_data[2]):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = LEFT_NEAR
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
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
                imu_realign(CENTER_ANGLE)   # <-- IMU correction here
                motor_speed = 15
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                cv2.imshow('Contours', img_contours)
                if cv2.waitKey(1) in [27, ord('q')]:
                    break
                continue

        # ==== NORMAL FOLLOWING ====
        if not avoidance_mode and state == "normal":
            motor_speed = 30
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

            if boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]

                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    box_center_x = (box_coords[0] + box_coords[2]) // 2
                    if chosen_color == "Red":
                        if box_center_x > center_x:
                            target_angle = LEFT_NEAR
                        else:
                            target_angle = RIGHT_NEAR
                    elif chosen_color == "Green":
                        if box_center_x < center_x:
                            target_angle = RIGHT_NEAR
                        else:
                            target_angle = LEFT_NEAR
                    else:
                        target_angle = compute_servo_angle(chosen_color, chosen_area)
            else:
                frame_count = 0
                last_color = None

        # ==== SERVO SMOOTHING ====
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if avoidance_mode else STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        # ==== DISPLAY ====
        cv2.imshow('Contours', img_contours)
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
