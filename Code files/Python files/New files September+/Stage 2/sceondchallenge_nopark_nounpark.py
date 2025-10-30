# ======= BOX AVOID + IMU (yours) + SHAPE-ONLY LINE TURN (ALWAYS LEFT, FIXED + 5s MIN TURN INTERVAL + STRONGER NO-BOX IMU) =======

import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import smbus2
import threading

# ToF deps
import board
import busio
import digitalio
import adafruit_vl53l0x

# ==== CONSTANTS ====
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR = 95
LEFT_NEAR = 130
RIGHT_FAR = 85
RIGHT_NEAR = 50

STEP = 6
FAST_SERVO_STEP = 9
SERVO_UPDATE_DELAY = 0.00

MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

# ToF side-collision threshold
SIDE_COLLIDE_CM = 20.0

# ==== BLUE-BOX BACKWARD LOGIC ====
blue_backward_start = None
in_blue_backward = False
BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 18

# ==== AVOIDANCE LOGIC ====
AVOID_BACK_DURATION = 1.0
AVOID_SPEED = 20

# ==== NORMAL LINE FOLLOWING SPEED ====
NORMAL_SPEED  = 15

# ==== LINE TURN (shape-only, diagonal band) ====
TURN_LEFT_SERVO = 60
TURN_IMU_TARGET_DEG = 90.0
TURN_COOLDOWN_S = 0.7
TURN_MIN_INTERVAL_S = 5.0
CANNY_LO, CANNY_HI = 60, 160
BLUR_KSIZE = 5
HOUGH_THRESHOLD = 60
HOUGH_MIN_LENGTH = 120
HOUGH_MAX_GAP = 20
LINE_DETECT_CONSEC_FRAMES = 2
LINE_ORIENT_MIN_DEG = 25
LINE_ORIENT_MAX_DEG = 65
LINE_MASK_THICKNESS = 8

SETTLE_DURATION = 0.0
settle_until_ts = 0.0

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

# adaptive drift control
DRIFT_GZ_THRESH = 0.8
BIAS_ALPHA = 0.002
STRAIGHT_SERVO_WINDOW = 8
DRIFT_DECAY_RATE = 0.20

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

# ---- IMU keep-straight (proportional) ----
YAW_DEADBAND_DEG_BASE = 3.0
YAW_KP_BASE = 1.0
SERVO_CORR_LIMIT_BASE = 22
YAW_DEADBAND_DEG_STRONG = 2.0
YAW_KP_STRONG = 1.04
SERVO_CORR_LIMIT_STRONG = 26

def imu_center_servo(current_yaw_deg: float, deadband: float, kp: float, limit: float) -> int:
    if abs(current_yaw_deg) <= deadband:
        return CENTER_ANGLE
    corr = kp * current_yaw_deg
    corr = max(-limit, min(limit, corr))
    return int(CENTER_ANGLE + corr)

# ==== SERVO SETUP ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA SETUP ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

# ==== ToF SETUP ====
i2c = busio.I2C(board.SCL, board.SDA)
xshut_pins = {
    "left": board.D16,
    "right": board.D25,
    "front": board.D26,
    "back": board.D24
}
addresses = {
    "left": 0x30,
    "right": 0x31,
    "front": 0x32,
    "back": 0x33
}

xshuts = {}
for name, pin in xshut_pins.items():
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False
    xshuts[name] = x
time.sleep(0.1)

sensors = {}
for name in ["left", "right", "front", "back"]:
    xshuts[name].value = True
    time.sleep(0.05)
    s = adafruit_vl53l0x.VL53L0X(i2c)
    s.set_address(addresses[name])
    s.start_continuous()
    sensors[name] = s
    print(f"[TOF] {name.upper()} active at {hex(addresses[name])}")

def tof_cm(sensor):
    try:
        val = sensor.range / 10.0
        if val <= 0 or val > 150:
            return 999
        return val
    except:
        return 999

# ==== HELPERS ====
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest_contour)
    return largest_contour, (x, y), (x + w, y + h), w*h

def thin_shape_reject(candidate, min_extent=0.30, ar_lo=0.35, ar_hi=3.0):
    if not candidate:
        return None
    cnt, tl, br, area = candidate
    x1, y1 = tl; x2, y2 = br
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return None
    ar = w / float(h)
    if not (ar_lo <= ar <= ar_hi):
        return None
    extent = cv2.contourArea(cnt) / float(w * h)
    if extent < min_extent:
        return None
    return candidate

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

def valid_orientation(x1,y1,x2,y2):
    dx = x2-x1; dy = y2-y1
    ang = abs(np.degrees(np.arctan2(dy, dx)))
    return LINE_ORIENT_MIN_DEG <= ang <= LINE_ORIENT_MAX_DEG

def preprocess_edges(img_bgr):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray = clahe.apply(gray)
    gray = cv2.GaussianBlur(gray, (BLUR_KSIZE, BLUR_KSIZE), 0)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)
    return edges

def detect_line_and_mask(edges, h, w):
    band_y1, band_y2 = int(h*0.45), int(h*0.90)
    roi = edges.copy()
    roi[:band_y1,:] = 0
    roi[band_y2:,:] = 0
    lines = cv2.HoughLinesP(
        roi, rho=1, theta=np.pi/180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LENGTH,
        maxLineGap=HOUGH_MAX_GAP
    )
    line_mask = np.zeros((h, w), dtype=np.uint8)
    seg = None
    if lines is not None:
        for ln in lines:
            x1,y1,x2,y2 = ln[0]
            if valid_orientation(x1,y1,x2,y2):
                if seg is None:
                    seg = (x1,y1,x2,y2)
                cv2.line(line_mask, (x1,y1), (x2,y2), 255, LINE_MASK_THICKNESS)
    return (seg is not None), seg, line_mask

# ==== STATE VARIABLES ====
last_color = None
frame_count = 0
last_update_time = time.time()
state = "normal"
state_start = time.time()

line_seen_streak = 0
last_turn_end_time = -1.0
turn_active = False

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

        raw_gz_dps = read_raw_data(GYRO_ZOUT_H)/131.0
        Gz = raw_gz_dps - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        if (not in_blue_backward) and (not avoidance_mode) and (not turn_active):
            if abs(current_servo_angle - CENTER_ANGLE) <= STRAIGHT_SERVO_WINDOW:
                if abs(raw_gz_dps) < DRIFT_GZ_THRESH:
                    gyro_z_bias = (1.0 - BIAS_ALPHA)*gyro_z_bias + BIAS_ALPHA*raw_gz_dps
                yaw *= (1.0 - min(1.0, DRIFT_DECAY_RATE * dt))

        img = picam2.capture_array()
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        h_img, w_img = img_bgr.shape[:2]

        edges = preprocess_edges(img_bgr)
        line_seen_raw, line_seg, line_mask = detect_line_and_mask(edges, h_img, w_img)
        time_since_last_turn = (time.time() - last_turn_end_time)
        out_of_cooldown = (time_since_last_turn >= TURN_COOLDOWN_S)
        out_of_min_interval = (time_since_last_turn >= TURN_MIN_INTERVAL_S)

        if not turn_active and out_of_cooldown and out_of_min_interval:
            line_seen_streak = line_seen_streak + 1 if line_seen_raw else 0
            line_seen = line_seen_streak >= LINE_DETECT_CONSEC_FRAMES
        else:
            line_seen_streak = 0
            line_seen = False

        ORANGE_H_LO, ORANGE_H_HI = 10, 32
        ORANGE_S_MIN, ORANGE_V_MIN = 80, 110
        mask_orange = cv2.inRange(
            imgHSV,
            np.array([ORANGE_H_LO, ORANGE_S_MIN, ORANGE_V_MIN], dtype=np.uint8),
            np.array([ORANGE_H_HI, 255, 255], dtype=np.uint8),
        )

        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]),   np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
        mask_red  = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_orange))
        if line_mask is not None:
            mask_red = cv2.bitwise_and(mask_red, cv2.bitwise_not(line_mask))

        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))
        if line_mask is not None:
            mask_green = cv2.bitwise_and(mask_green, cv2.bitwise_not(line_mask))

        k3 = np.ones((3,3), np.uint8)
        k5 = np.ones((5,5), np.uint8)
        def morph(m):
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k3)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k5)
            return m
        mask_red   = morph(mask_red)
        mask_green = morph(mask_green)

        img_contours = img_bgr.copy()
        boxes = []

        red_data = thin_shape_reject(get_largest_contour(mask_red, min_area=MIN_AREA))
        if red_data:
            _, top_left, bottom_right, area = red_data
            boxes.append(("Red", area, (*top_left, *bottom_right)))
            cv2.rectangle(img_contours, top_left, bottom_right, (0,0,255), 2)
            cv2.putText(img_contours, f"Red {int(area)}", (top_left[0], max(20, top_left[1]-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        green_data = thin_shape_reject(get_largest_contour(mask_green, min_area=MIN_AREA))
        if green_data:
            _, top_left, bottom_right, area = green_data
            boxes.append(("Green", area, (*top_left, *bottom_right)))
            cv2.rectangle(img_contours, top_left, bottom_right, (0,255,0), 2)
            cv2.putText(img_contours, f"Green {int(area)}", (top_left[0], max(20, top_left[1]-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        center_x = img_contours.shape[1] // 2
        car_width, car_height = 200, 75
        bottom_y = img_contours.shape[0] - 10
        car_box = (center_x - car_width//2, bottom_y - car_height,
                   center_x + car_width//2, bottom_y)

        if line_seg:
            x1,y1,x2,y2 = line_seg
            cv2.line(img_contours, (x1,y1), (x2,y2), (0,255,255), 3)

        cv2.rectangle(img_contours, (car_box[0],car_box[1]), (car_box[2],car_box[3]), (255,255,255), 1)

        target_angle = CENTER_ANGLE
        motor_speed = 0

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
                with yaw_lock:
                    yaw = 0.0 
                in_blue_backward = False
                current_servo_angle = CENTER_ANGLE
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                settle_until_ts = time.time() + SETTLE_DURATION
                motor_speed = NORMAL_SPEED
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                cv2.imshow('Contours', img_contours)
                if cv2.waitKey(1) in [27, ord('q')]:
                    break
                continue

        if not avoidance_mode:
            for color, _, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode = True
                    avoid_direction = "right" if color == "Green" else "left"
                    avoid_start_time = time.time()
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                    target_angle = RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR
                    state = "avoid"
                    state_start = time.time()
                    break

        if avoidance_mode:
            elapsed = time.time() - (avoid_start_time or time.time())
            if elapsed < AVOID_BACK_DURATION:
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                target_angle = imu_center_servo(current_yaw, YAW_DEADBAND_DEG_BASE, YAW_KP_BASE, SERVO_CORR_LIMIT_BASE)
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    state = "normal"
                    with yaw_lock:
                        yaw = 0.0
                    settle_until_ts = time.time() + SETTLE_DURATION

        if (not avoidance_mode) and (not in_blue_backward) and (not turn_active) and line_seen and out_of_min_interval:
            turn_active = True
            with yaw_lock:
                yaw = 0.0
            set_servo_angle(SERVO_CHANNEL, TURN_LEFT_SERVO)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

        if turn_active:
            target_angle = TURN_LEFT_SERVO
            if abs(current_yaw) >= TURN_IMU_TARGET_DEG:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                with yaw_lock:
                    yaw = 0.0
                turn_active = False
                last_turn_end_time = time.time()
                settle_until_ts = time.time() + SETTLE_DURATION

        if not avoidance_mode and not in_blue_backward and not turn_active and state == "normal":
            motor_speed = NORMAL_SPEED
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

            if not boxes:
                target_angle = imu_center_servo(
                    current_yaw,
                    YAW_DEADBAND_DEG_STRONG,
                    YAW_KP_STRONG,
                    SERVO_CORR_LIMIT_STRONG
                )

                # ToF side protection while IMU-correcting with no boxes
                l = tof_cm(sensors["left"])
                r = tof_cm(sensors["right"])
                if l < SIDE_COLLIDE_CM and r >= SIDE_COLLIDE_CM and not boxes:
                    target_angle = 125
                elif r < SIDE_COLLIDE_CM and l >= SIDE_COLLIDE_CM and not boxes:
                    target_angle = 55
                #elif l < r and not boxes:
                    #target_angle = 110
                #elif r < l and not boxes:
                    #target_angle  = 70
            else:
                target_angle = imu_center_servo(
                    current_yaw,
                    YAW_DEADBAND_DEG_BASE,
                    YAW_KP_BASE,
                    SERVO_CORR_LIMIT_BASE
                )
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]

                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    if chosen_color == "Red":
                        color_angle = LEFT_NEAR
                        target_angle = max(50, min(130, color_angle + int(current_yaw * 2)))
                    elif chosen_color == "Green":
                        color_angle = RIGHT_NEAR
                        target_angle = max(50, min(130, color_angle - int(current_yaw * 2)))
                    else:
                        target_angle = max(50, min(130, compute_servo_angle(chosen_color, chosen_area)))

        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            fast = (avoidance_mode or in_blue_backward or turn_active)
            step = FAST_SERVO_STEP if fast else STEP

            if time.time() < settle_until_ts and not turn_active and not in_blue_backward:
                desired = CENTER_ANGLE
            else:
                desired = target_angle

            if abs(current_servo_angle - desired) > step:
                current_servo_angle += step if current_servo_angle < desired else -step
            else:
                current_servo_angle = desired

            current_servo_angle = max(50, min(130, current_servo_angle))
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        cv2.imshow('Contours', img_contours)
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
