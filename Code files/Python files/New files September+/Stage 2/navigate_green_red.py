import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import board
import busio


# ==== CONSTANTS ====
MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2  # reverse channelimport smbus2
import time
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from pca9685_control import set_servo_angle, set_motor_speed

# ==== CONSTANTS ====
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0

# Servo angles
CENTER_ANGLE = 90
LEFT_ANGLE = 105
RIGHT_ANGLE = 75
LEFT_FAR = 95
LEFT_NEAR = 110
RIGHT_FAR = 85
RIGHT_NEAR = 60
CENTER_DEADZONE = 5
SERVO_STEP = 6.0
STEP = 3
FAST_SERVO_STEP = 6
SERVO_UPDATE_DELAY = 0.02

# Motor speeds (unchanged from your last message)
NORMAL_SPEED = 12          # a bit faster
BLUE_BACK_SPEED = 16
AVOID_SPEED = 12
IMU_CORRECT_SPEED = 10

BLUE_BACK_DURATION = 1.5
AVOID_BACK_DURATION = 1.0

# ==== BLACK WALL SETTINGS ====
BLACK_LO = np.array([0,   0,   0],   dtype=np.uint8)
BLACK_HI = np.array([180, 255, 70],  dtype=np.uint8)
WALL_MIN_AREA = 3000
WALL_UNDER_MARGIN = 6

# MPU6050
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
GYRO_ZOUT_H = 0x47

# Camera detection
MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

# Blue-turn behavior
BLUE_TURN_DIRECTION = "right"      # "left" or "right"
BLUE_TURN_HOLD = COLOR_HOLD_FRAMES
BLUE_TURN_YAW_TARGET = 88
BLUE_TURN_SPEED = 17
BLUE_TURN_MAX_TIME = 2.5

# >>> Blue trigger gating <<<
BLUE_TURN_MIN_AREA = 2600
BLUE_TURN_TRIGGER_Y = 405
BLUE_TURN_TRIPLINE_Y = 400
BLUE_TURN_CENTER_TOL = 90
BLUE_TURN_HOLD_FRAMES = 12
BLUE_TURN_YAW_OK = 10

# >>> Tripwire & cooldown <<<
BLUE_TURN_TRIPLINE_Y = 380
BLUE_TURN_COOLDOWN_S = 2.0
BLUE_DISARM_CLEAR_FRAMES = 6

# ==== GLOBAL VARIABLES ====
yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()
current_servo_angle = CENTER_ANGLE

# Blue-backward state
in_blue_backward = False
blue_backward_start = None

# Camera/avoidance state
last_color = None
frame_count = 0
last_update_time = time.time()
state = "normal"
state_start = time.time()
avoidance_mode = False
avoid_direction = None
avoid_start_time = None

# Blue turn state
turning_blue = False
blue_turn_dir = None
blue_turn_start_yaw = 0.0
blue_turn_start_time = None

# Blue gated trigger hold
blue_hold_count = 0

# Tripwire / cooldown state
last_blue_bottom = None
blue_armed = True
last_blue_seen_time = 0.0
last_turn_time = 0.0
blue_absent_count = 0

# ==== NEW: Calibration & debug ====
CAL_MODE = True            # press 'c' to toggle
SHOW_MASKS = True          # press 'm' to toggle
CAL_WINDOW = "CAL_ORANGE_BLUE"

# Defaults (very loose) — masks will be: (CAL) OR (DEFAULT_LOOSE)
ORANGE_DEFAULT = [5, 60, 50, 30, 255, 255]   # [Hmin,Smin,Vmin,Hmax,Smax,Vmax]
BLUE_DEFAULT   = [90, 60, 45, 140, 255, 255]

# Trackbar helpers
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

# ==== I2C BUS ====
bus = smbus2.SMBus(1)

# ==== MPU6050 FUNCTIONS ====
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

# ==== YAW RESET THREAD ====
def reset_yaw_listener():
    global yaw
    while True:
        input("Press ENTER to reset yaw to 0°")
        with yaw_lock:
            yaw = 0.0
            print("Yaw reset to 0°")

threading.Thread(target=reset_yaw_listener, daemon=True).start()

# ==== INITIALIZE SERVO ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)

# ==== CAMERA SETUP ====
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# Lock AWB/AE after warm-up
try:
    md = picam2.capture_metadata()
    gains = md.get("ColourGains", (1.8, 1.8))
    picam2.set_controls({
        "AwbEnable": False,
        "ColourGains": gains,
    })
except Exception as e:
    print("AWB/AE lock skipped:", e)

# ==== HELPERS (from your code, unchanged in behavior) ====
def get_largest_box(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / h
        if 0.8 < aspect_ratio < 1.2:
            boxes.append((cnt, (x, y), (x+w, y+h), area))
    if not boxes:
        return None
    return max(boxes, key=lambda b: b[3])

def get_largest_line(mask, min_area=500, ar_lo=0.67, ar_hi_inv=1.5):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    lines = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if h <= 0: 
            continue
        aspect_ratio = w / h
        if aspect_ratio > ar_hi_inv or aspect_ratio < ar_lo:
            lines.append((cnt, (x, y), (x+w, y+h), area))
    if not lines:
        return None
    return max(lines, key=lambda b: b[3])

def get_largest_rect(mask, min_area=800):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    rects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / h if h > 0 else 0
        if 0.5 < aspect_ratio < 2.2:
            rects.append((cnt, (x, y), (x+w, y+h), area))
    if not rects:
        return None
    return max(rects, key=lambda b: b[3])

def compute_servo_angle(color, area):
    norm_area = max(MIN_AREA, min(MAX_AREA, area))
    closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)
    if color == "Red":
        return int(LEFT_FAR + closeness * (LEFT_NEAR - LEFT_FAR))
    elif color == "Orange":
        return int(CENTER_ANGLE - closeness * 20)
    elif color == "Blue":
        return LEFT_ANGLE
    else:  # Green
        return int(RIGHT_FAR - closeness * (RIGHT_FAR - RIGHT_NEAR))

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

def get_wall_floor_y(mask_black, min_area=WALL_MIN_AREA):
    contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0
    max_bottom = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        bottom = y + h
        if bottom > max_bottom:
            max_bottom = bottom
    return max_bottom

# NEW: gentle CLAHE on V channel (helps dull lines)
def boost_v_channel(img_hsv):
    h,s,v = cv2.split(img_hsv)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    v2 = clahe.apply(v)
    return cv2.merge([h,s,v2])

# ==== MAIN LOOP ====
try:
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    if CAL_MODE:
        setup_trackbar_window()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        # ==== IMU: UPDATE YAW ====
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # ==== CAMERA CAPTURE ====
        img = picam2.capture_array()

        # HSV + V-boost
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imgHSV = boost_v_channel(imgHSV)

        # ---- FIXED masks (kept) for red/green/black ----
        mask_red1 = cv2.inRange(imgHSV, np.array([0,   110,  80]), np.array([10,  255, 255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170, 110,  80]), np.array([180, 255, 255]))
        mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green  = cv2.inRange(imgHSV, np.array([40,  60,  60]), np.array([90, 255, 255]))
        mask_black  = cv2.inRange(imgHSV, BLACK_LO, BLACK_HI)

        # ---- ORANGE/BLUE: calibrated OR default (union) ----
        if CAL_MODE and cv2.getWindowProperty(CAL_WINDOW, 0) >= 0:
            o_rng = read_trackbar_range("ORANGE")
            b_rng = read_trackbar_range("BLUE")
        else:
            o_rng = ORANGE_DEFAULT
            b_rng = BLUE_DEFAULT

        o_lo, o_hi = range_to_np(o_rng)
        b_lo, b_hi = range_to_np(b_rng)

        mask_orange_cal = cv2.inRange(imgHSV, o_lo, o_hi)
        mask_blue_cal   = cv2.inRange(imgHSV, b_lo, b_hi)

        # very loose defaults to ensure something shows while tuning
        mask_orange_default = cv2.inRange(imgHSV,
                                          np.array(ORANGE_DEFAULT[:3], dtype=np.uint8),
                                          np.array(ORANGE_DEFAULT[3:], dtype=np.uint8))
        mask_blue_default   = cv2.inRange(imgHSV,
                                          np.array(BLUE_DEFAULT[:3], dtype=np.uint8),
                                          np.array(BLUE_DEFAULT[3:], dtype=np.uint8))

        # UNION: if either detects, we keep it
        mask_orange = cv2.bitwise_or(mask_orange_cal, mask_orange_default)
        mask_blue   = cv2.bitwise_or(mask_blue_cal,   mask_blue_default)

        # morphology
        kernel3 = np.ones((3,3), np.uint8)
        kernel5 = np.ones((5,5), np.uint8)
        for m in (mask_red, mask_orange, mask_green, mask_blue, mask_black):
            cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel3, dst=m)
            cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel5, dst=m)

        # Emphasize horizontal bands for orange/blue
        horiz_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (31, 3))
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, horiz_kernel)
        mask_blue   = cv2.morphologyEx(mask_blue,   cv2.MORPH_CLOSE, horiz_kernel)

        # ---- Compute wall floor (lowest black) ----
        wall_floor_y = get_wall_floor_y(mask_black, WALL_MIN_AREA)
        wall_gate = wall_floor_y + WALL_UNDER_MARGIN

        # ---- DETECT LARGEST OBJECTS ----
        red_data    = get_largest_rect(mask_red,    min_area=1200)
        green_data  = get_largest_rect(mask_green,  min_area=1200)

        # First pass
        orange_data = get_largest_line(mask_orange, min_area=600)
        blue_data   = get_largest_line(mask_blue,   min_area=700)

        # Fallback pass if missed
        if orange_data is None:
            orange_data = get_largest_line(mask_orange, min_area=350, ar_lo=0.6, ar_hi_inv=1.6)
        if blue_data is None:
            blue_data   = get_largest_line(mask_blue,   min_area=450, ar_lo=0.6, ar_hi_inv=1.6)

        # ---- Prefer RED over ORANGE if overlap or similar ----
        if red_data and orange_data:
            _, (rx, ry), (rx2, ry2), rarea = red_data
            _, (ox, oy), (ox2, oy2), oarea = orange_data
            red_box = (rx, ry, rx2, ry2)
            orange_box = (ox, oy, ox2, oy2)
            if boxes_intersect(red_box, orange_box) or (abs(rarea - oarea) < 0.3 * max(rarea, oarea)):
                orange_data = None

        img_contours = img.copy()
        boxes = []

        # under-wall filter
        def under_wall_ok(bbox):
            return bbox[3] >= wall_gate

        if red_data:
            _, top_left, bottom_right, area = red_data
            box = (*top_left, *bottom_right)
            if under_wall_ok(box):
                boxes.append(("Red", area, box))

        if orange_data:
            _, top_left, bottom_right, area = orange_data
            box = (*top_left, *bottom_right)
            if under_wall_ok(box):
                boxes.append(("Orange", area, box))

        if green_data:
            _, top_left, bottom_right, area = green_data
            box = (*top_left, *bottom_right)
            if under_wall_ok(box):
                boxes.append(("Green", area, box))

        # Car box for collision logic
        h_img, w_img = img.shape[:2]
        center_x = w_img // 2
        car_width, car_height = 200, 50
        bottom_y = h_img - 10
        car_box = (center_x - car_width//2, bottom_y - car_height,
                   center_x + car_width//2, bottom_y)

        # Blue handling
        blue_present_this_frame = False
        blue_bottom_this_frame = None
        blue_bbox_this_frame = None

        if blue_data:
            _, top_left, bottom_right, area = blue_data
            blue_box = (*top_left, *bottom_right)
            if under_wall_ok(blue_box) and not boxes_intersect(car_box, blue_box):
                boxes.append(("Blue", area, blue_box))
                blue_present_this_frame = True
                blue_bottom_this_frame = blue_box[3]
                blue_bbox_this_frame = blue_box

        # track blue presence for cooldown/disarm logic
        if blue_present_this_frame:
            last_blue_seen_time = now
            blue_absent_count = 0
        else:
            blue_absent_count += 1

        target_angle = CENTER_ANGLE
        motor_speed = NORMAL_SPEED

        # ==== BLUE-BACKWARD LOGIC ====
        if not in_blue_backward:
            for color_name, _, box_coords in boxes:
                if color_name in ["Red","Green"] and boxes_intersect(car_box, box_coords):
                    in_blue_backward = True
                    blue_backward_start = time.time()
                    target_angle = LEFT_NEAR if color_name=="Red" else RIGHT_NEAR
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
                motor_speed = NORMAL_SPEED
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                # keep loop going
                # continue  # (not adding continue so debug windows still update)

        # ==== AVOIDANCE LOGIC (only for Red/Green) ====
        if not avoidance_mode:
            for color, _, box_coords in boxes:
                if color in ["Red", "Green"] and boxes_intersect(car_box, box_coords):
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

        # ==== BLUE TURN STATE ====
        if state == "blue_turn":
            set_motor_speed(MOTOR_FWD, MOTOR_REV, BLUE_TURN_SPEED)
            target_angle = LEFT_NEAR if blue_turn_dir == "left" else RIGHT_NEAR

            elapsed_turn = time.time() - blue_turn_start_time if blue_turn_start_time else 0
            if abs(yaw) >= BLUE_TURN_YAW_TARGET or elapsed_turn >= BLUE_TURN_MAX_TIME:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
                time.sleep(0.1)
                with yaw_lock:
                    yaw = 0.0
                state = "normal"
                turning_blue = False
                blue_hold_count = 0
                blue_armed = False
                last_turn_time = time.time()
                target_angle = CENTER_ANGLE

        # ==== NORMAL LINE-FOLLOWING + IMU SERVO ADJUST ====
        if state == "normal" and not avoidance_mode:
            motor_speed = NORMAL_SPEED
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

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
                        color_angle = LEFT_NEAR
                        target_angle = color_angle + int(yaw * 2)
                        blue_hold_count = 0
                    elif chosen_color == "Green":
                        color_angle = RIGHT_NEAR
                        target_angle = color_angle - int(yaw * 2)
                        blue_hold_count = 0
                    elif chosen_color == "Orange":
                        target_angle = CENTER_ANGLE
                        blue_hold_count = 0
                    elif chosen_color == "Blue":
                        x1, y1, x2, y2 = box_coords
                        cx = (x1 + x2) // 2
                        bottom = y2
                        width = (x2 - x1)
                        height = (y2 - y1)

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
                            blue_turn_dir = BLUE_TURN_DIRECTION
                            with yaw_lock:
                                yaw = 0.0
                            blue_turn_start_yaw = 0.0
                            blue_turn_start_time = time.time()
                            state = "blue_turn"
                            state_start = time.time()
                        else:
                            target_angle = LEFT_NEAR if BLUE_TURN_DIRECTION == "left" else RIGHT_NEAR
                else:
                    blue_hold_count = 0

            else:
                if current_yaw > CENTER_DEADZONE:
                    target_angle = LEFT_ANGLE
                elif current_yaw < -CENTER_DEADZONE:
                    target_angle = RIGHT_ANGLE
                else:
                    target_angle = CENTER_ANGLE
                set_motor_speed(MOTOR_FWD, MOTOR_REV, IMU_CORRECT_SPEED)

            target_angle = max(60, min(130, target_angle))

        # ==== SERVO SMOOTHING ====
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if avoidance_mode or state == "blue_turn" else STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            current_servo_angle = max(60, min(130, current_servo_angle))
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        # ==== YAW TURN DETECTION (not used for blue turn) ====
        if state != "blue_turn" and abs(current_yaw) >= 90:
            direction = 'LEFT' if current_yaw > 0 else 'RIGHT'
            print(f"Turn completed: {direction}")
            with yaw_lock:
                yaw = 0

        # ==== DISPLAY ====
        # Draw the wall floor line for debugging
        img_contours = img.copy()
        img_contours = cv2.line(img_contours, (0, wall_floor_y + WALL_UNDER_MARGIN), (img.shape[1]-1, wall_floor_y + WALL_UNDER_MARGIN), (0, 255, 255), 2)
        cv2.imshow('Contours', img_contours)

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
              f"BlueBottom={last_blue_bottom} | Absent={blue_absent_count} | WallFloorY={wall_floor_y} | "
              f"CAL={CAL_MODE} | SHOW_MASKS={SHOW_MASKS}")

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
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    print("Stopped and reset.")

SERVO_CHANNEL = 0
CENTER_ANGLE = 90  # servo center
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

# ==== BLUE-BOX BACKWARD LOGIC ====
blue_backward_start = None
in_blue_backward = False
BLUE_BACK_DURATION = 1.5  # updated to 1.5 seconds
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
        return int(RIGHT_FAR + closeness * (RIGHT_NEAR - RIGHT_FAR))
    else:
        return int(LEFT_FAR - closeness * (LEFT_FAR - LEFT_NEAR))

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

# ==== STABILITY VARIABLES ====
last_color = None
frame_count = 0
last_update_time = time.time()

# ==== STATE VARIABLES ====
state = "normal"
state_start = time.time()
HOLD_DURATION = 1.0

def get_largest_contour_corners(contours):
    if len(contours) == 0:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    return ( (x, y), (x + w, y), (x, y + h), (x + w, y + h), largest_contour )

try:
    # ==== INITIALIZATION ====
    motors_started = False
    avoidance_mode = False
    avoid_direction = None
    avoid_start_time = None
    FAST_SERVO_STEP = 4

    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    current_servo_angle = CENTER_ANGLE
    last_update_time = time.time()

    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    # ==== MAIN LOOP ====
    while True:

        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        scale = 1080 / h_img
        img_small = cv2.resize(img, (int(w_img * scale), 1080))

        # Convert directly from RGB to HSV
        imgHSV = cv2.cvtColor(img_small, cv2.COLOR_RGB2HSV)

        # ==== MASKS ====
        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_green = cv2.inRange(imgHSV, np.array([35,60,60]), np.array([95,255,255]))

        img_contours = img_small.copy()
        boxes = []

        # Red detection
        red_data = get_largest_contour(mask_red)
        if red_data:
            _, top_left, bottom_right, area = red_data
            cv2.rectangle(img_contours, top_left, bottom_right, (0,0,255), 2)
            cv2.putText(img_contours, "Red Box", (top_left[0], top_left[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            boxes.append(("Red", area, (*top_left, *bottom_right)))

        # Green detection
        green_data = get_largest_contour(mask_green)
        if green_data:
            _, top_left, bottom_right, area = green_data
            cv2.rectangle(img_contours, top_left, bottom_right, (0,255,0), 2)
            cv2.putText(img_contours, "Green Box", (top_left[0], top_left[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            boxes.append(("Green", area, (*top_left, *bottom_right)))

        # ==== CAR BOX ====
        center_x = img_contours.shape[1] // 2
        car_width, car_height = 350, 100
        bottom_y = img_contours.shape[0] - 10
        car_top_left = (center_x - car_width // 2, bottom_y - car_height)
        car_bottom_right = (center_x + car_width // 2, bottom_y)
        car_box = (*car_top_left, *car_bottom_right)
        cv2.rectangle(img_contours, car_top_left, car_bottom_right, (255,0,0), -1)

        target_angle = CENTER_ANGLE
        motor_speed = 0

        # ==== BACKWARD LOGIC ====
        if not in_blue_backward:
            # <<< MODIFIED: when backing for colors
            if red_data and boxes_intersect(car_box, red_data[1]+red_data[2]):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = RIGHT_NEAR  # Red → go back AND right
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
            elif green_data and boxes_intersect(car_box, green_data[1]+green_data[2]):
                in_blue_backward = True
                blue_backward_start = time.time()
                target_angle = LEFT_NEAR   # Green → go back AND left
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
            # <<< END MODIFIED SECTION

        if in_blue_backward:
            step = FAST_SERVO_STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)

            if time.time() - blue_backward_start >= BLUE_BACK_DURATION:
                in_blue_backward = False
                motor_speed = 15
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                cv2.imshow('Contours', img_contours)
                if cv2.waitKey(1) in [27, ord('q')]:
                    break
                continue  # skip rest of loop while backing

        # ==== AVOIDANCE LOGIC ====
        if motors_started and not avoidance_mode:
            for color, _, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode = True
                    avoid_direction = "left" if color == "Green" else "right"
                    avoid_start_time = time.time()
                    motor_speed = 20
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -motor_speed)
                    target_angle = LEFT_FAR if avoid_direction == "left" else RIGHT_FAR
                    state = "avoid"
                    state_start = time.time()
                    break

        if avoidance_mode:
            elapsed = time.time() - avoid_start_time
            if elapsed < AVOID_BACK_DURATION and avoid_direction == "left":
                set_servo_angle(SERVO_CHANNEL, LEFT_NEAR)
                motor_speed = 20
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -motor_speed)
            elif elapsed < AVOID_BACK_DURATION and avoid_direction == "right":
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR)
                motor_speed = 20
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -motor_speed)
            else:
                motor_speed = 20
                set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
                target_angle = LEFT_FAR if avoid_direction == "left" else RIGHT_FAR
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    state = "normal"

        # ==== NORMAL LINE-FOLLOWING ====
        if not avoidance_mode and state == "normal":
            motor_speed = 30
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

            if boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]

                # Stability filter
                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    if chosen_color == "Red":
                        box_center_x = (box_coords[0]+box_coords[2])//2
                        target_angle = LEFT_NEAR if box_center_x > center_x else 110
                    elif chosen_color == "Green":
                        box_center_x = (box_coords[0]+box_coords[2])//2
                        target_angle = RIGHT_NEAR if box_center_x < center_x else 70
                    else:
                        target_angle = compute_servo_angle(chosen_color, chosen_area)
                
                if frame_count >= COLOR_HOLD_FRAMES:
                    box_center_x = (box_coords[0] + box_coords[2]) // 2  # center of detected box

                    if chosen_color == "Red":
                        if box_center_x > center_x:
                            target_angle = LEFT_NEAR
                        else:
                            target_angle = RIGHT_NEAR

                    elif chosen_color == "Green":
                        if box_center_x < center_x:
                            target_angle = RIGHT_NEAR

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

        # ==== SHOW RESULTS ====
        cv2.imshow('Contours', img_contours)
        if cv2.waitKey(1) in [27, ord('q')]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, 90)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
