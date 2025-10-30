# ======= FULL CODE: BOX AVOID + IMU + ORANGE/BLUE SHARP-TURN DETECTION =======

import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import smbus2
import threading
import board
import busio
import digitalio
import adafruit_vl53l0x

# ================== CONSTANTS ==================
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_NEAR = 130
RIGHT_NEAR = 50
FAST_SERVO_STEP = 9
STEP = 6
SERVO_UPDATE_DELAY = 0.00

MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5
SIDE_COLLIDE_CM = 20.0

BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 18
NORMAL_SPEED = 15
AVOID_BACK_DURATION = 1.0
AVOID_SPEED = 20

TURN_LEFT_SERVO = 60
TURN_IMU_TARGET_DEG = 90.0
TURN_COOLDOWN_S = 0.7
TURN_MIN_INTERVAL_S = 5.0

# ==== Canny / line params ====
CANNY_LO, CANNY_HI = 60, 160
BLUR_KSIZE = 5

# ==== MPU6050 ====
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

# ==== CAMERA ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

# ==== ToF sensors ====
i2c = busio.I2C(board.SCL, board.SDA)
xshut_pins = {"left": board.D16, "right": board.D25, "front": board.D26, "back": board.D24}
addresses = {"left": 0x30, "right": 0x31, "front": 0x32, "back": 0x33}

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
    w, h = x2-x1, y2-y1
    if w <=0 or h <=0: return None
    ar = w/float(h)
    if not (ar_lo <= ar <= ar_hi): return None
    extent = cv2.contourArea(cnt)/float(w*h)
    if extent < min_extent: return None
    return candidate

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

def imu_center_servo(current_yaw_deg: float, deadband: float, kp: float, limit: float) -> int:
    if abs(current_yaw_deg) <= deadband:
        return CENTER_ANGLE
    corr = kp * current_yaw_deg
    corr = max(-limit, min(limit, corr))
    return int(CENTER_ANGLE + corr)

def preprocess_edges(img_bgr):
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (BLUR_KSIZE, BLUR_KSIZE), 0)
    edges = cv2.Canny(gray, CANNY_LO, CANNY_HI)
    return edges

# ==== STATE VARIABLES ====
last_color = None
frame_count = 0
last_turn_end_time = -1.0
turn_active = False
in_blue_backward = False
blue_backward_start = None
avoidance_mode = False
avoid_direction = None
avoid_start_time = None
settle_until_ts = 0.0
current_servo_angle = CENTER_ANGLE

set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
time.sleep(0.2)
last_update_time = time.time()

# ================= MAIN LOOP =================
try:
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
            yaw *= 0.8  # drift decay

        img = picam2.capture_array()
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        h_img, w_img = img_bgr.shape[:2]

        # ---- ORANGE / BLUE line masks ----
        mask_orange = cv2.inRange(imgHSV, np.array([0,70,150]), np.array([20,200,230]))
        mask_blue   = cv2.inRange(imgHSV, np.array([70,0,130]), np.array([125,170,170]))
        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue   = cv2.Canny(mask_blue, 50, 150)
        lines_orange = cv2.HoughLinesP(edges_orange,1,np.pi/180,50,50,10)
        lines_blue   = cv2.HoughLinesP(edges_blue,1,np.pi/180,50,50,10)

        def y_at_center(lines):
            if lines is None: return -1
            y_vals = []
            cx = w_img//2
            for x1,y1,x2,y2 in lines[:,0]:
                if x1!=x2:
                    slope = (y2-y1)/(x2-x1)
                    y = slope*(cx-x1)+y1
                    y_vals.append(y)
                else:
                    if x1==cx: y_vals.append(max(y1,y2))
            return max(y_vals) if y_vals else -1

        orange_y = y_at_center(lines_orange)
        blue_y   = y_at_center(lines_blue)

        Turn = "No"
        TURN_ANGLE = CENTER_ANGLE
        if orange_y > blue_y and orange_y > 400:
            Turn = "Right"
            TURN_ANGLE = 120
        elif blue_y > orange_y and blue_y > 400:
            Turn = "Left"
            TURN_ANGLE = 60

        # ---- Blue/Red box detection ----
        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))

        red_data = thin_shape_reject(get_largest_contour(mask_red, MIN_AREA))
        green_data = thin_shape_reject(get_largest_contour(mask_green, MIN_AREA))

        car_width, car_height = 200, 75
        bottom_y = h_img - 10
        center_x = w_img//2
        car_box = (center_x-car_width//2, bottom_y-car_height,
                   center_x+car_width//2, bottom_y)

        boxes = []
        if red_data: boxes.append(("Red", red_data[3], (*red_data[1], *red_data[2])))
        if green_data: boxes.append(("Green", green_data[3], (*green_data[1], *green_data[2])))

        # ====== LOGIC: BLUE BACKWARD ======
        if not in_blue_backward:
            for color, area, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    in_blue_backward = True
                    blue_backward_start = time.time()
                    TURN_ANGLE = LEFT_NEAR if color=="Red" else RIGHT_NEAR
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                    break

        if in_blue_backward:
            step = FAST_SERVO_STEP
            if abs(current_servo_angle - TURN_ANGLE) > step:
                current_servo_angle += step if current_servo_angle < TURN_ANGLE else -step
            else:
                current_servo_angle = TURN_ANGLE
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)

            if time.time() - blue_backward_start >= BLUE_BACK_DURATION:
                with yaw_lock: yaw = 0.0
                in_blue_backward = False
                current_servo_angle = CENTER_ANGLE
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
            continue

        # ====== SHARP-TURN LOGIC ======
        now_turn = time.time()
        time_since_last_turn = now_turn - last_turn_end_time
        out_of_cooldown = time_since_last_turn >= TURN_COOLDOWN_S
        out_of_min_interval = time_since_last_turn >= TURN_MIN_INTERVAL_S

        if not turn_active and out_of_cooldown and out_of_min_interval and Turn!="No":
            turn_active = True
            with yaw_lock: yaw=0.0
            set_servo_angle(SERVO_CHANNEL, TURN_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

        if turn_active:
            if abs(current_yaw) >= TURN_IMU_TARGET_DEG:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                with yaw_lock: yaw=0.0
                turn_active = False
                last_turn_end_time = time.time()

        # ====== NORMAL DRIVE / IMU CENTERING ======
        if not in_blue_backward and not turn_active:
            target_angle = imu_center_servo(current_yaw, 2.0, 1.04, 26)
            if time.time() < settle_until_ts:
                target_angle = CENTER_ANGLE
            if abs(current_servo_angle - target_angle) > STEP:
                current_servo_angle += STEP if current_servo_angle < target_angle else -STEP
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

except KeyboardInterrupt:
    pass
finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
