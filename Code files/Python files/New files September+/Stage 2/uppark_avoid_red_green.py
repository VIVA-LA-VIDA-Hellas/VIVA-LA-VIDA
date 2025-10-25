# ======= UNPARK + VISION RUN (COMBINED) =======
# Your logic, fixed & enhanced:
# - Phase A: UNPARK using VL53L0X (left fwd -> right rev -> left hold), with debounced triggers + timeouts
# - Phase B: RUN using camera+IMU with black-wall gate, orange/blue detection, HSV calibration, and
#            forced blue-turn direction (no deciding based on line).

import time
import threading
import cv2
import numpy as np

# I2C / GPIO / ToF
import board
import busio
import digitalio
import adafruit_vl53l0x
from gpiozero import Button

# Camera + PWM controller
from picamera2 import Picamera2
from pca9685_control import set_servo_angle, set_motor_speed

# ===============================
# BUTTON (start the whole flow)
# ===============================
start_button = Button(20)

# ===============================
# MOTOR / SERVO CHANNELS
# ===============================
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0

# ===============================
# —— UNPARK CONSTANTS ——
# ===============================
# Servo angles for unpark
SERVO_CENTER      = 88
SERVO_LEFT_LIMIT  = 55   # more left
SERVO_RIGHT_LIMIT = 130  # more right

# Speeds and distances
UNPARK_FWD_SPEED   = 13
UNPARK_REV_SPEED   = 13
UNPARK_FWD_FRONT_CM  = 6.0   # go forward until front < this
UNPARK_REV_BACK_CM   = 7.5   # go reverse  until back  < this

# Safety / stability
UNPARK_STABLE_READS = 3       # consecutive hits needed
UNPARK_READ_HZ      = 10      # 10Hz reads in stable_trigger
UNPARK_TIMEOUT_S    = 6.0     # per step timeout so you never hang

# ===============================
# —— RUN CONSTANTS (VISION/IMU) ——
# ===============================
CENTER_ANGLE = 90
LEFT_ANGLE = 105
RIGHT_ANGLE = 75
LEFT_FAR = 95
LEFT_NEAR = 110
RIGHT_FAR = 85
RIGHT_NEAR = 60
CENTER_DEADZONE = 5
STEP = 3
FAST_SERVO_STEP = 6
SERVO_UPDATE_DELAY = 0.02

# Speeds
NORMAL_SPEED = 12
BLUE_BACK_SPEED = 16
AVOID_SPEED = 12
IMU_CORRECT_SPEED = 10
BLUE_BACK_DURATION = 1.5
AVOID_BACK_DURATION = 1.0

# Blue-turn, **forced direction** (no choosing from line)
BLUE_TURN_DIRECTION = "right"      # <-- set "left" or "right"
BLUE_TURN_YAW_TARGET = 88
BLUE_TURN_SPEED = 17
BLUE_TURN_MAX_TIME = 2.5

# Blue trigger gating
BLUE_TURN_MIN_AREA = 2600
BLUE_TURN_TRIGGER_Y = 405
BLUE_TURN_TRIPLINE_Y = 400
BLUE_TURN_CENTER_TOL = 90
BLUE_TURN_HOLD_FRAMES = 12
BLUE_TURN_YAW_OK = 10

# Cooldowns
BLUE_TURN_COOLDOWN_S = 2.0
BLUE_DISARM_CLEAR_FRAMES = 6

# Black walls (gate everything under the walls)
BLACK_LO = np.array([0,   0,   0],   dtype=np.uint8)
BLACK_HI = np.array([180, 255, 70],  dtype=np.uint8)
WALL_MIN_AREA = 3000
WALL_UNDER_MARGIN = 6

# Detection / smoothing
MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

# Calibration UI
CAL_MODE = True            # press 'c' to toggle
SHOW_MASKS = True          # press 'm' to toggle
CAL_WINDOW = "CAL_ORANGE_BLUE"
ORANGE_DEFAULT = [5, 60, 50, 30, 255, 255]
BLUE_DEFAULT   = [90, 60, 45, 140, 255, 255]

# ===============================
# I2C & VL53L0X SETUP (UNPARK)
# ===============================
i2c = busio.I2C(board.SCL, board.SDA)

xshut_pins = {
    "left": board.D16,
    "right": board.D25,
    "front": board.D26,
    "back": board.D24
}
addresses = {
    "left":  0x30,
    "right": 0x31,
    "front": 0x32,
    "back":  0x33
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
    except Exception:
        return 999

def stable_trigger(sensor, threshold, below=True, count=3, timeout_s=6.0, label=""):
    """Debounced ToF trigger with timeout safeguard."""
    hits = 0
    start = time.time()
    period = 1.0 / UNPARK_READ_HZ
    while True:
        if time.time() - start > timeout_s:
            print(f"[TIMEOUT] stable_trigger {label} — continuing anyway.")
            return False
        val = tof_cm(sensor)
        print(f"[{label}] {val:.2f} cm")
        if below and 0 < val < threshold:
            hits += 1
        elif not below and val > threshold:
            hits += 1
        else:
            hits = 0
        if hits >= count:
            return True
        time.sleep(period)

# ===============================
# IMU (GYRO) – only yaw via MPU6050
# ===============================
import smbus2
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

print("Measuring gyro bias... Keep sensor still.")
mpu6050_init()
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

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

# ===============================
# CAMERA
# ===============================
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# (optional) lock AWB for consistency
try:
    md = picam2.capture_metadata()
    gains = md.get("ColourGains", (1.8, 1.8))
    picam2.set_controls({"AwbEnable": False, "ColourGains": gains})
except Exception as e:
    print("AWB lock skipped:", e)

# ===============================
# VISION HELPERS
# ===============================
def setup_trackbar_window():
    cv2.namedWindow(CAL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CAL_WINDOW, 520, 220)
    for name, vals in [("ORANGE", ORANGE_DEFAULT), ("BLUE", BLUE_DEFAULT)]:
        hmin,smin,vmin,hmax,smax,vmax = vals
        cv2.createTrackbar(f"{name}_Hmin", CAL_WINDOW, hmin, 180, lambda x: None)
        cv2.createTrackbar(f"{name}_Smin", CAL_WINDOW, smin, 255, lambda x: None)
        cv2.createTrackbar(f"{name}_Vmin", CAL_WINDOW, vmin, 255, lambda x: None)
        cv2.createTrackbar(f"{name}_Hmax", CAL_WINDOW, hmax, 180, lambda x: None)
        cv2.createTrackbar(f"{name}_Smax", CAL_WINDOW, smax, 255, lambda x: None)
        cv2.createTrackbar(f"{name}_Vmax", CAL_WINDOW, vmax, 255, lambda x: None)

def read_trackbar_range(prefix):
    hmin = cv2.getTrackbarPos(f"{prefix}_Hmin", CAL_WINDOW)
    smin = cv2.getTrackbarPos(f"{prefix}_Smin", CAL_WINDOW)
    vmin = cv2.getTrackbarPos(f"{prefix}_Vmin", CAL_WINDOW)
    hmax = cv2.getTrackbarPos(f"{prefix}_Hmax", CAL_WINDOW)
    smax = cv2.getTrackbarPos(f"{prefix}_Smax", CAL_WINDOW)
    vmax = cv2.getTrackbarPos(f"{prefix}_Vmax", CAL_WINDOW)
    return [hmin,smin,vmin,hmax,smax,vmax]

def range_to_np(lohi):
    return np.array(lohi[:3], dtype=np.uint8), np.array(lohi[3:], dtype=np.uint8)

def get_largest_line(mask, min_area=500, ar_lo=0.67, ar_hi_inv=1.5):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = None
    best_area = 0
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area: continue
        x,y,w,h = cv2.boundingRect(c)
        if h == 0: continue
        ar = w / h
        if ar > ar_hi_inv or ar < ar_lo:
            if a > best_area:
                best = (c,(x,y),(x+w,y+h),a)
                best_area = a
    return best

def get_largest_rect(mask, min_area=800):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    best = None
    best_area = 0
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area: continue
        x,y,w,h = cv2.boundingRect(c)
        ar = w / h if h>0 else 0
        if 0.5 < ar < 2.2:
            if a > best_area:
                best = (c,(x,y),(x+w,y+h),a)
                best_area = a
    return best

def boxes_intersect(b1, b2):
    x1,y1,x2,y2 = b1
    X1,Y1,X2,Y2 = b2
    return not (x2 < X1 or x1 > X2 or y2 < Y1 or y1 > Y2)

def get_wall_floor_y(mask_black, min_area=WALL_MIN_AREA):
    contours,_ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0
    max_bottom = 0
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area: continue
        x,y,w,h = cv2.boundingRect(c)
        btm = y + h
        if btm > max_bottom:
            max_bottom = btm
    return max_bottom

def boost_v_channel(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    v2 = clahe.apply(v)
    hsv2 = cv2.merge([h,s,v2])
    return cv2.cvtColor(hsv2, cv2.COLOR_HSV2BGR)

# ===============================
# STATE (RUN)
# ===============================
current_servo_angle = CENTER_ANGLE
last_update_time = time.time()
in_blue_backward = False
blue_backward_start = None

state = "normal"
avoidance_mode = False
avoid_direction = None
avoid_start_time = None

turning_blue = False
blue_turn_dir = None
blue_turn_start_time = None

blue_hold_count = 0
last_blue_bottom = None
blue_armed = True
last_turn_time = 0.0
blue_absent_count = 0

last_color = None
frame_count = 0

# ===============================
# ———— PHASE A: UNPARK ————
# ===============================
print("\n=== UNPARK SEQUENCE (press button to start) ===")
start_button.wait_for_press()
print("Button pressed! Starting UNPARK...")

try:
    # Center and stop
    set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.4)

    # 1) LEFT + forward until FRONT < threshold (debounced, with timeout)
    print("[UNPARK 1] LEFT & forward until FRONT < %.1f cm" % UNPARK_FWD_FRONT_CM)
    set_servo_angle(SERVO_CHANNEL, SERVO_LEFT_LIMIT)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_FWD_SPEED)
    stable_trigger(
        sensors["front"],
        UNPARK_FWD_FRONT_CM,
        below=True,
        count=UNPARK_STABLE_READS,
        timeout_s=UNPARK_TIMEOUT_S,
        label="FRONT"
    )
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    # 2) RIGHT + reverse until BACK < threshold
    print("[UNPARK 2] RIGHT & reverse until BACK < %.1f cm" % UNPARK_REV_BACK_CM)
    set_servo_angle(SERVO_CHANNEL, SERVO_RIGHT_LIMIT)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -UNPARK_REV_SPEED)
    stable_trigger(
        sensors["back"],
        UNPARK_REV_BACK_CM,
        below=True,
        count=UNPARK_STABLE_READS,
        timeout_s=UNPARK_TIMEOUT_S,
        label="BACK"
    )
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    # 3) LEFT again and hold forward gently to exit
    print("[UNPARK 3] LEFT & gentle forward to exit box")
    set_servo_angle(SERVO_CHANNEL, SERVO_LEFT_LIMIT)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_FWD_SPEED)
    time.sleep(0.9)  # small nudge out (tune if needed)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.1)

    print("[UNPARK DONE] Switching to VISION RUN...")
except KeyboardInterrupt:
    print("[UNPARK] interrupted; stopping.")
    set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    raise

# ===============================
# ———— PHASE B: VISION RUN ————
# ===============================
if CAL_MODE:
    setup_trackbar_window()

try:
    # init center
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    current_servo_angle = CENTER_ANGLE
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    while True:
        # IMU update
        now = time.time()
        dt = now - last_time
        last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # Frame
        img = picam2.capture_array()
        img_boost = boost_v_channel(img)
        imgHSV = cv2.cvtColor(img_boost, cv2.COLOR_BGR2HSV)

        # Masks
        mask_red1 = cv2.inRange(imgHSV, np.array([0,   110,  80]), np.array([10,  255, 255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170, 110,  80]), np.array([180, 255, 255]))
        mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(imgHSV, np.array([40,60,60]), np.array([90,255,255]))
        mask_black = cv2.inRange(imgHSV, BLACK_LO, BLACK_HI)

        # Calibrated OR default for orange/blue (union)
        if CAL_MODE and cv2.getWindowProperty(CAL_WINDOW, 0) >= 0:
            o_rng = read_trackbar_range("ORANGE")
            b_rng = read_trackbar_range("BLUE")
        else:
            o_rng = ORANGE_DEFAULT
            b_rng = BLUE_DEFAULT
        o_lo,o_hi = range_to_np(o_rng)
        b_lo,b_hi = range_to_np(b_rng)

        mask_orange = cv2.inRange(imgHSV, o_lo, o_hi)
        mask_orange = cv2.bitwise_or(mask_orange,
                     cv2.inRange(imgHSV, np.array(ORANGE_DEFAULT[:3], np.uint8), np.array(ORANGE_DEFAULT[3:], np.uint8)))
        mask_blue   = cv2.inRange(imgHSV, b_lo, b_hi)
        mask_blue   = cv2.bitwise_or(mask_blue,
                     cv2.inRange(imgHSV, np.array(BLUE_DEFAULT[:3], np.uint8), np.array(BLUE_DEFAULT[3:], np.uint8)))

        # Morphology
        k3 = np.ones((3,3), np.uint8)
        k5 = np.ones((5,5), np.uint8)
        for m in (mask_red, mask_orange, mask_green, mask_blue, mask_black):
            cv2.morphologyEx(m, cv2.MORPH_OPEN,  k3, dst=m)
            cv2.morphologyEx(m, cv2.MORPH_CLOSE, k5, dst=m)
        horiz_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (31,3))
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, horiz_kernel)
        mask_blue   = cv2.morphologyEx(mask_blue,   cv2.MORPH_CLOSE, horiz_kernel)

        # Wall gate
        wall_floor_y = get_wall_floor_y(mask_black, WALL_MIN_AREA)
        wall_gate = wall_floor_y + WALL_UNDER_MARGIN

        # Detect
        red_data    = get_largest_rect(mask_red,   min_area=1200)
        green_data  = get_largest_rect(mask_green, min_area=1200)
        orange_data = get_largest_line(mask_orange, min_area=600)
        blue_data   = get_largest_line(mask_blue,   min_area=700)
        if orange_data is None:
            orange_data = get_largest_line(mask_orange, min_area=350, ar_lo=0.6, ar_hi_inv=1.6)
        if blue_data is None:
            blue_data   = get_largest_line(mask_blue,   min_area=450, ar_lo=0.6, ar_hi_inv=1.6)

        # Prefer red over orange if overlap/similar
        def to_box(d): 
            _, tl, br, _a = d; return (tl[0],tl[1],br[0],br[1])
        if red_data and orange_data:
            if boxes_intersect(to_box(red_data), to_box(orange_data)):
                orange_data = None

        img_contours = img.copy()
        boxes = []
        def under_wall_ok(box): return box[3] >= wall_gate

        if red_data:
            bx = to_box(red_data); 
            if under_wall_ok(bx): boxes.append(("Red", red_data[3], bx))
        if green_data:
            bx = to_box(green_data); 
            if under_wall_ok(bx): boxes.append(("Green", green_data[3], bx))
        if orange_data:
            bx = to_box(orange_data); 
            if under_wall_ok(bx): boxes.append(("Orange", orange_data[3], bx))

        # Car box
        h_img,w_img = img.shape[:2]
        center_x = w_img//2
        car_width,car_height = 200,50
        bottom_y = h_img - 10
        car_box = (center_x - car_width//2, bottom_y - car_height,
                   center_x + car_width//2, bottom_y)

        # Blue handling (not under bumper)
        blue_present_this_frame = False
        if blue_data:
            bx = to_box(blue_data)
            if under_wall_ok(bx) and not boxes_intersect(car_box, bx):
                boxes.append(("Blue", blue_data[3], bx))
                blue_present_this_frame = True
                last_blue_seen_time_local = now
                blue_absent_count = 0
        if not blue_present_this_frame:
            blue_absent_count += 1

        # ======= BLUE-BACKWARD (collide red/green under bumper) =======
        target_angle = CENTER_ANGLE
        if not in_blue_backward:
            for cname,_area,box in boxes:
                if cname in ["Red","Green"] and boxes_intersect(car_box, box):
                    in_blue_backward = True
                    blue_backward_start = time.time()
                    target_angle = LEFT_NEAR if cname=="Red" else RIGHT_NEAR
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                    break

        if in_blue_backward:
            step = FAST_SERVO_STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)

            if time.time() - blue_backward_start >= BLUE_BACK_DURATION:
                in_blue_backward = False
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                # fall through so UI keeps updating

        # ======= AVOIDANCE (red/green in car box) =======
        if not in_blue_backward:
            global_avoid_triggered = False
            if not avoidance_mode:
                for color,_a,box in boxes:
                    if color in ["Red","Green"] and boxes_intersect(car_box, box):
                        avoidance_mode = True
                        avoid_direction = "right" if color=="Green" else "left"
                        avoid_start_time = time.time()
                        set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                        target_angle = RIGHT_NEAR if avoid_direction=="right" else LEFT_NEAR
                        state = "avoid"
                        global_avoid_triggered = True
                        break

            if avoidance_mode:
                elapsed = time.time() - avoid_start_time
                if elapsed < AVOID_BACK_DURATION:
                    set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR if avoid_direction=="right" else LEFT_NEAR)
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                else:
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                    target_angle = CENTER_ANGLE
                    if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                        avoidance_mode = False
                        state = "normal"

        # ======= BLUE TURN STATE =======
        if state == "blue_turn":
            set_motor_speed(MOTOR_FWD, MOTOR_REV, BLUE_TURN_SPEED)
            target_angle = LEFT_NEAR if blue_turn_dir=="left" else RIGHT_NEAR
            elapsed_turn = time.time() - blue_turn_start_time if blue_turn_start_time else 0
            if abs(yaw) >= BLUE_TURN_YAW_TARGET or elapsed_turn >= BLUE_TURN_MAX_TIME:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
                time.sleep(0.1)
                with yaw_lock: yaw = 0.0
                state = "normal"
                turning_blue = False
                blue_hold_count = 0
                blue_armed = False
                last_turn_time = time.time()
                target_angle = CENTER_ANGLE

        # ======= NORMAL (line-follow + gated blue trigger) =======
        if state == "normal" and not avoidance_mode and not in_blue_backward:
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

            # re-arm blue
            if not blue_armed:
                if (now - last_turn_time) >= BLUE_TURN_COOLDOWN_S and blue_absent_count >= BLUE_DISARM_CLEAR_FRAMES:
                    blue_armed = True
                    blue_hold_count = 0
                    last_blue_bottom = None

            use_box_control = True
            if abs(current_yaw) > 65 and any(b[0]!="Orange" for b in boxes):
                use_box_control = False

            if use_box_control and boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]
                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    if chosen_color == "Red":
                        target_angle = LEFT_NEAR + int(yaw*2)
                        blue_hold_count = 0
                    elif chosen_color == "Green":
                        target_angle = RIGHT_NEAR - int(yaw*2)
                        blue_hold_count = 0
                    elif chosen_color == "Orange":
                        target_angle = CENTER_ANGLE
                        blue_hold_count = 0
                    elif chosen_color == "Blue":
                        # Detect only; direction is pre-set
                        x1,y1,x2,y2 = box_coords
                        cx = (x1+x2)//2
                        bottom = y2
                        width = (x2-x1); height=(y2-y1)
                        close_enough = (chosen_area >= BLUE_TURN_MIN_AREA) and (bottom >= BLUE_TURN_TRIGGER_Y)
                        centered = abs(cx - center_x) <= BLUE_TURN_CENTER_TOL
                        yaw_ok = abs(current_yaw) <= BLUE_TURN_YAW_OK
                        line_like = width > height
                        crossed_tripwire = False
                        if last_blue_bottom is not None:
                            crossed_tripwire = (last_blue_bottom < BLUE_TURN_TRIPLINE_Y) and (bottom >= BLUE_TURN_TRIPLINE_Y)
                        last_blue_bottom = bottom

                        if blue_armed and close_enough and centered and yaw_ok and line_like and crossed_tripwire:
                            blue_hold_count += 1
                        else:
                            blue_hold_count = 0

                        if blue_armed and blue_hold_count >= BLUE_TURN_HOLD_FRAMES:
                            turning_blue = True
                            blue_turn_dir = BLUE_TURN_DIRECTION    # <-- FORCED direction
                            with yaw_lock: yaw = 0.0
                            blue_turn_start_time = time.time()
                            state = "blue_turn"
                        else:
                            # gently bias toward intended side while waiting
                            target_angle = LEFT_NEAR if BLUE_TURN_DIRECTION=="left" else RIGHT_NEAR
            else:
                # IMU only
                if current_yaw > CENTER_DEADZONE:
                    target_angle = LEFT_ANGLE
                elif current_yaw < -CENTER_DEADZONE:
                    target_angle = RIGHT_ANGLE
                else:
                    target_angle = CENTER_ANGLE
                set_motor_speed(MOTOR_FWD, MOTOR_REV, IMU_CORRECT_SPEED)

            target_angle = max(60, min(130, target_angle))

        # ======= SERVO SMOOTHING =======
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if (avoidance_mode or state=="blue_turn") else STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            current_servo_angle = max(60, min(130, current_servo_angle))
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        # ======= DEBUG WINDOWS =======
        img_contours = img.copy()
        # wall gate line
        img_contours = cv2.line(img_contours, (0, wall_gate), (img.shape[1]-1, wall_gate), (0,255,255), 2)
        cv2.imshow("Contours", img_contours)

        if SHOW_MASKS:
            row1 = np.hstack([
                cv2.cvtColor(mask_orange, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_blue,   cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_green,  cv2.COLOR_GRAY2BGR),
            ])
            row2 = np.hstack([
                cv2.cvtColor(mask_red,    cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_black,  cv2.COLOR_GRAY2BGR),
                np.zeros_like(cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)),
            ])
            masks_viz = np.vstack([row1, row2])
            cv2.imshow("Masks (Orange | Blue | Green / Red | Black | — )", masks_viz)

        print(f"Yaw={current_yaw:.2f}° | Servo={current_servo_angle:.1f} | Target={target_angle} | "
              f"State={state} | BlueHold={blue_hold_count} | Armed={blue_armed} | "
              f"Absent={blue_absent_count} | WallGateY={wall_gate} | CAL={CAL_MODE} SHOW_MASKS={SHOW_MASKS}")

        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord('q')]:
            break
        if key == ord('c'):
            CAL_MODE = not CAL_MODE
            if CAL_MODE:
                setup_trackbar_window()
            else:
                if cv2.getWindowProperty(CAL_WINDOW, 0) >= 0:
                    cv2.destroyWindow(CAL_WINDOW)
        if key == ord('m'):
            SHOW_MASKS = not SHOW_MASKS

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, 90)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    print("Stopped and reset.")
