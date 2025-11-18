#-----------------------------------------------------------------------------------------------------------------------
# 2nd Mission WRO 2025 FE - VivaLaVida
# Final Version
#-----------------------------------------------------------------------------------------------------------------------

# v1.1: add state print, add turn/lap count and stop after 3 laps, variables cleanup
#       add ultrasonic for side checks during cruise
# v1.2: add backwards drive after turning, unpark in both directions
# v1.3: give priority to object that is closer [line or obstacle]
# v1.4-v1.5: did not work
# v1.6: restructuring with FSM, lock object logic, unpark alternstive in open space

# =========================
# IMPORTS
# =========================

import os, sys

VENV_PY = "/home/stem/env/bin/python"  # exact path to Python in your venv

if sys.executable != VENV_PY and os.path.exists(VENV_PY):
    print(f"[INFO] Relaunching under virtual environment: {VENV_PY}", flush=True)
    os.execv(VENV_PY, [VENV_PY] + sys.argv)

# --- your normal robot code below ---
print(f"[INFO] Now running inside: {sys.executable}")

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
from gpiozero import Device, Button, DistanceSensor, LED
from gpiozero.pins.lgpio import LGPIOFactory   # Uses /dev/gpiochip*
Device.pin_factory = LGPIOFactory()

# =========================
# CONFIGURABLE PARAMETERS
# =========================

# ---- Speed settings ----
NORMAL_SPEED          = 13   # Base forward cruising speed during normal driving
AVOID_SPEED           = 13   # Reverse speed when doing obstacle-avoidance backup
TURN_MOTOR_SPEED      = 15   # Motor speed while performing a 90° line-based turn
UNPARK_STRAIGHT_SPEED = 18   # Speed used during smart unpark phases
STOP_SPEED            = 0    # Zero-speed (motors off)
POST_TURN_BACK_SPEED  = 17     # Reverse speed used right after each turn

# ---- Motor / servo basics ----
MOTOR_FWD       = 1         # PCA9685 motor channel for forward direction
MOTOR_REV       = 2         # PCA9685 motor channel for reverse direction
SERVO_CHANNEL   = 0         # PCA9685 channel used by steering servo
CENTER_ANGLE    = 90        # Servo angle for going straight
LEFT_FAR        = 105       # Steering angle for a far red box (mild left)
LEFT_NEAR       = 120       # Steering angle for a near red box (strong left)
RIGHT_FAR       = 75        # Steering angle for a far green box (mild right)
RIGHT_NEAR      = 60        # Steering angle for a near green box (strong right)
LEFT_COLLIDE_ANGLE = 120    # Steering angle on left side collision correctio
RIGHT_COLLIDE_ANGLE = 60    # Steering angle on right side collision correctio

# ---- Obstacle detection (vision) ----
MIN_AREA          = 1500    # Minimum contour area to accept as a box
MAX_AREA          = 18000   # Area at which box is considered "very close"
COLOR_HOLD_FRAMES = 2       # Frames the same color must persist to be “locked”
OBSTACLE_CLEAR_FRAMES = 10     # Frames without red/green to "forget" obstacle

# ---- ToF thresholds (general) ----
SIDE_COLLIDE_CM       = 30.0  # If side < this, we steer away to avoid collision

# ---- Post-turn backward reposition ----
POST_TURN_BACK_CLEAR_CM  = 20.0   # Back ToF distance target after a normal line-based turn
POST_TURN_BACK_TIMEOUT_S = 2.0    # Safety timeout so we don't reverse forever
POST_TURN_LINE_IGNORE_S  = 4.0    # Time to ignore blue/orange lines after backing (sec)

# ---- Obstacle avoidance ----
AVOID_BACK_DURATION = 0.7  # Reverse duration in normal avoidance (seconds)

# ---- Line-turn (shape / Hough based) ----
TURN_LEFT_SERVO       = 60    # Servo angle for a hard left line-turn
TURN_COOLDOWN_S       = 0.7   # Minimal cooldown after a line detection
TURN_MIN_INTERVAL_S   = 5.0   # Minimum time between two turns
CANNY_LO, CANNY_HI    = 60, 160  # Canny edge thresholds for line band pre-processing
BLUR_KSIZE            = 5     # Gaussian blur kernel size for edge preprocessing
HOUGH_THRESHOLD       = 60    # HoughLinesP threshold
HOUGH_MIN_LENGTH      = 120   # Min line length to accept from HoughLinesP
HOUGH_MAX_GAP         = 20    # Max allowed gap in HoughLinesP segments
LINE_DETECT_CONSEC_FRAMES = 3 # Frames of consistent line detection to confirm
LINE_ORIENT_MIN_DEG   = 25    # Minimum angle (deg) to accept a line as “diagonal”
LINE_ORIENT_MAX_DEG   = 65    # Maximum angle (deg) to accept a line as “diagonal”
LINE_MASK_THICKNESS   = 9     # Thickness of mask drawn over detected line band

# ---- Turn-related constants ----
LINE_CENTER_BLUE_Y_MIN     = 550  # Minimal Y for blue line to be valid for turn
LINE_CENTER_ORANGE_Y_MIN   = 600  # Minimal Y for orange line to be valid for turn
TURN_RIGHT_SERVO           = 120  # Servo angle for a hard right line-turn
TURN_FAILSAFE_MAX_DEG      = 80.0 # Failsafe yaw to stop turn even without box

YAW_RESET_AFTER_LEFT  = -10.0     # Yaw offset after finishing a left turn
YAW_RESET_AFTER_RIGHT = 10.0      # Yaw offset after finishing a right turn

TURN_COOLDOWN_SEC = 6.0           # Cooldown after each completed turn (no new turns)

# ---- Obstacle vs line priority ----
OBSTACLE_LINE_MARGIN_PX = 25  # How many pixels "closer" something must be to win priority

# HSV thresholds (tweak for venue lighting)
RED1_LO    = np.array([0,   220, 110], dtype=np.uint8)  # Red lower hue range 1
RED1_HI    = np.array([5,  255, 170], dtype=np.uint8)  # Red upper hue range 1
RED2_LO    = np.array([0, 120, 110], dtype=np.uint8)  # Red lower hue range 2
RED2_HI    = np.array([5, 255, 255], dtype=np.uint8)  # Red upper hue range 2
GREEN_LO   = np.array([70, 145, 70],  dtype=np.uint8)   # Green lower HSV bound
GREEN_HI   = np.array([80, 200, 160], dtype=np.uint8)   # Green upper HSV bound
ORANGE_LO  = np.array([6,  170, 170], dtype=np.uint8)   # Orange lower HSV bound
ORANGE_HI  = np.array([14, 210, 205], dtype=np.uint8)   # Orange upper HSV bound
BLUE_LO    = np.array([110, 100, 110], dtype=np.uint8)  # Blue lower HSV bound
BLUE_HI    = np.array([120, 211, 150], dtype=np.uint8)  # Blue upper HSV bound

# ---- Dynamic yaw / line trigger tuning ----
BLUE_MIN_LEN_PX     = 70    # Minimal Hough line length (blue/orange) to trigger turn

# ---- Post-reverse / settling behavior ----
SETTLE_DURATION      = 0.0  # Seconds to force CENTER_ANGLE after certain events
settle_until_ts      = 0.0  # Timestamp until which settle is active

# ---- Extra IMU gain near boxes ----
BOX_YAW_GAIN_MIN = 1.5  # Minimum yaw gain when box just appears
BOX_YAW_GAIN_MAX = 3.0  # Maximum yaw gain when box very close

# ---- Drift control ----
DRIFT_GZ_THRESH       = 0.8   # Max |Gz| to consider for drift/bias update
BIAS_ALPHA            = 0.002 # Smoothing factor for gyro bias update
STRAIGHT_SERVO_WINDOW = 8     # Servo must be within this of CENTER to update bias

# ---- Stability gates ----
YAW_CLAMP_DEG     = 120.0 # Limit |yaw| to avoid runaway integration
SOFT_DECAY_RATE   = 0.6   # Softer yaw decay factor used during bias update

# ---- IMU keep-straight gains ----
YAW_KP_BASE             = 1.2  # Base proportional gain for yaw correction
SERVO_CORR_LIMIT_BASE   = 25   # Max correction (deg) from base yaw controller
YAW_DEADBAND_DEG_BASE   = 4.0  # Deadband for base yaw correction (small error ignored)
YAW_DEADBAND_DEG_STRONG = 6.0  # Deadband when using stronger yaw correction
YAW_KP_STRONG           = 1.3  # Stronger proportional gain when no boxes
SERVO_CORR_LIMIT_STRONG = 24   # Max correction (deg) for strong yaw controller

# ---- Smart Unpark configuration ----
UNPARK_CENTER_ANGLE     = CENTER_ANGLE # Servo angle for going straight during unpark
UNPARK_LEFT_TURN_ANGLE  = 55           # Initial unpark steering angle if turning left
UNPARK_RIGHT_TURN_ANGLE = 125          # Initial unpark steering angle if turning right

# ---- Turn / lap counting ----
TURNS_PER_LAP = 4   # Number of line-based turns per lap
TOTAL_LAPS    = 3   # Number of laps to complete
TOTAL_TURNS   = TURNS_PER_LAP * TOTAL_LAPS  # Convenience (total planned turns)

# ---- Line / ToF gates for turning ----
TURN_FRONT_MIN_CM = 18.0   # Min front distance to allow starting a line-based turn
TURN_FRONT_MAX_CM = 140.0  # Max front distance to allow starting a line-based turn

# =========================
# HARDWARE SETUP
# =========================

# ---- Status LEDs (gpiozero) ----
RED_LED_PIN   = 13
GREEN_LED_PIN = 19

red_led   = LED(RED_LED_PIN)
green_led = LED(GREEN_LED_PIN)

# Program start indication: red ON, green OFF
red_led.on()
green_led.off()

# ---- IMU SETUP ----
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

# ---- YAW TRACKING -----
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

def imu_center_servo(current_yaw_deg: float, deadband: float, kp: float, limit: float) -> int:
    if abs(current_yaw_deg) <= deadband:
        return CENTER_ANGLE
    corr = kp * current_yaw_deg
    corr = max(-limit, min(limit, corr))
    return int(CENTER_ANGLE + corr)

# ----- SERVO SETUP -----
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ---- CAMERA SETUP ----
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1280, 480)})) #(main={"size": (640, 480)}))
picam2.set_controls({"FrameRate": 15})
picam2.start()
time.sleep(1.0)
#picam2.set_controls({"AwbEnable": False})

# ---- ToF SETUP ----
i2c = busio.I2C(board.SCL, board.SDA)
xshut_pins = {
    "left":    board.D16,
    "right":   board.D25,
    "front":   board.D26,
    "back":    board.D8,
    "front_l": board.D7,
    "front_r": board.D24
}
addresses = {
    "left":    0x30,
    "right":   0x31,
    "front":   0x32,
    "back":    0x33,
    "front_l": 0x34,
    "front_r": 0x35
}

xshuts = {}
for name, pin in xshut_pins.items():
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False
    xshuts[name] = x
time.sleep(0.1)

sensors = {}
for name in ["left", "right", "front", "back", "front_l", "front_r"]:
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

# ==== ULTRASONIC PINS (FRONT, LEFT, RIGHT) ====
TRIG_FRONT, ECHO_FRONT = 22, 23  # GPIO pins for front sensor
TRIG_LEFT,  ECHO_LEFT  = 27, 17  # GPIO pins for left sensor
TRIG_RIGHT, ECHO_RIGHT = 5,  6   # GPIO pins for right sensor

# ==== ULTRASONIC SENSORS (gpiozero DistanceSensor) ====
front_ultra = DistanceSensor(echo=ECHO_FRONT, trigger=TRIG_FRONT, max_distance=2.4, queue_len=3)
left_ultra  = DistanceSensor(echo=ECHO_LEFT,  trigger=TRIG_LEFT,  max_distance=1.2, queue_len=3)
right_ultra = DistanceSensor(echo=ECHO_RIGHT, trigger=TRIG_RIGHT, max_distance=1.2, queue_len=3)

def ultra_cm(sensor, max_cm=200.0):
    d_m = sensor.distance  # in meters, between 0 and max_distance
    if d_m is None:
        return 999
    d_cm = d_m * 100.0
    if d_cm <= 0 or d_cm > max_cm:
        return 999
    return d_cm

def get_front_ultra_cm():
    return ultra_cm(front_ultra)

def get_left_ultra_cm():
    return ultra_cm(left_ultra)

def get_right_ultra_cm():
    return ultra_cm(right_ultra)

# ==== HELPERS ====

# ==== STATE PRINTING ====
run_state = None

def set_run_state(new_state: str):
    global run_state
    if new_state != run_state:
        run_state = new_state
        print(f"[STATE] {new_state}", flush=True)

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

def max_line_len(lines):
    if lines is None:
        return 0
    m = 0
    for x1, y1, x2, y2 in lines[:, 0]:
        L = int(np.hypot(x2 - x1, y2 - y1))
        if L > m:
            m = L
    return m

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

def y_at_center(lines, center_x):
    if lines is None:
        return -1
    ys = []
    for x1,y1,x2,y2 in lines[:,0]:
        if x1 != x2:
            m = (y2 - y1) / float(x2 - x1)
            y = m * (center_x - x1) + y1
            ys.append(y)
        else:
            if x1 == center_x:
                ys.append(max(y1, y2))
    return max(ys) if ys else -1

# ===============================
# SMART UNPARK (two-phase; faster; left-case phase 2 = 65° + extra ~60° left if direction == "left")
# ===============================
start_button = Button(20)

print("\n=== SMART UNPARK START ===")
red_led.off()
green_led.blink(on_time=0.3, off_time=0.3, background=True)

start_button.wait_for_press()
print("Button pressed! tarting sequence...")
green_led.on()
time.sleep(1)

left_dist = tof_cm(sensors["left"])
right_dist = tof_cm(sensors["right"])
print(f"Left={left_dist:.1f}cm | Right={right_dist:.1f}cm")

def UnPark_L():
    # Phase 1: ~45�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    first_angle = UNPARK_LEFT_TURN_ANGLE
    set_servo_angle(SERVO_CHANNEL, first_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= 45.0:
            break
    
    # Phase 2: ~40�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    second_angle = UNPARK_RIGHT_TURN_ANGLE; target_abs_yaw = 40.0
    set_servo_angle(SERVO_CHANNEL, second_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= target_abs_yaw:
            break
    
    # Straighten & reset yaw
    set_servo_angle(SERVO_CHANNEL, UNPARK_CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    with yaw_lock:
        yaw = 0.0
    print("[DONE] Unpark sequence complete. Entering vision/avoid loop...")
    set_run_state("cruise")

def UnPark_R():
        # Phase 1: ~45�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    first_angle = UNPARK_RIGHT_TURN_ANGLE
    set_servo_angle(SERVO_CHANNEL, first_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= 45.0:
            break
    
    # Phase 2: ~40�
    with yaw_lock:
        yaw = 0.0
    last_time = time.time()
    second_angle = UNPARK_LEFT_TURN_ANGLE;  target_abs_yaw = 40.0
    set_servo_angle(SERVO_CHANNEL, second_angle)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt; current_yaw = yaw
        if abs(current_yaw) >= target_abs_yaw:
            break
    
    # Straighten & reset yaw
    set_servo_angle(SERVO_CHANNEL, UNPARK_CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_STRAIGHT_SPEED)
    with yaw_lock:
        yaw = 0.0
    print("[DONE] Unpark sequence complete. Entering vision/avoid loop...")
    set_run_state("cruise")

if left_dist > right_dist:
    direction = "left"
    UnPark_L()
    print("? Choosing LEFT (more open).")
else:
    direction = "right"
    UnPark_R()
    print("? Choosing RIGHT (more open).")

# ==== FSM STATES & RUNTIME VARIABLES ====

STATE_CRUISE    = "cruise"
STATE_TURN      = "turn"
STATE_POST_TURN = "post_turn"
STATE_AVOID     = "avoid_obstacle"

fsm_state        = STATE_CRUISE
state_start_time = time.time()

# Obstacle lock & avoidance phases
obstacle_lock_color      = None   # "Red" or "Green" when in AVOID
obstacle_lock_last_area  = 0
obstacle_lock_last_box   = None   # (x1, y1, x2, y2)
obstacle_clear_streak    = 0      # consecutive frames without the locked obstacle
avoid_phase              = None   # "back_off" or "forward"
avoid_phase_start        = 0.0
avoid_direction          = None   # "left" or "right" during reverse

# Turn / line detection & gating
blue_gate_streak         = 0
last_turn_end_time       = -1.0
next_turn_allowed_time   = 0.0
post_turn_line_ignore_until = 0.0

turn_dir                 = None   # "Left" or "Right"
post_turn_start_time     = 0.0

# Lap & turn counters
turn_count = 0
lap_count  = 1

# Color streak for obstacle locking
last_color         = None
color_hold_streak  = 0

# Servo state
current_servo_angle = CENTER_ANGLE
last_time           = time.time()
settle_until_ts     = 0.0  # already defined above, we just make sure it exists

# ---- Simple debug helper ----
def dbg(msg: str):
    print(f"[DBG {time.time():.2f}] {msg}", flush=True)

try:
    # initial “safe” motor state
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    time.sleep(0.2)

    dbg("FSM start -> CRUISE")

    while True:
        now = time.time()
        dt  = now - last_time
        last_time = now

        # ===== IMU / YAW UPDATE =====
        raw_gz_dps = read_raw_data(GYRO_ZOUT_H) / 131.0
        Gz         = raw_gz_dps - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            if yaw > YAW_CLAMP_DEG:
                yaw = YAW_CLAMP_DEG
            elif yaw < -YAW_CLAMP_DEG:
                yaw = -YAW_CLAMP_DEG
            current_yaw = yaw

        # Drift / bias update only in cruise
        if fsm_state == STATE_CRUISE:
            near_center = abs(current_servo_angle - CENTER_ANGLE) <= STRAIGHT_SERVO_WINDOW
            if near_center and abs(raw_gz_dps) < DRIFT_GZ_THRESH:
                gyro_z_bias = (1.0 - BIAS_ALPHA)*gyro_z_bias + BIAS_ALPHA*raw_gz_dps
                yaw *= (1.0 - min(1.0, SOFT_DECAY_RATE * dt))

        # Default target angle (will be overwritten by each state)
        target_angle = current_servo_angle

        # ===== VISION: IMAGE, LINES, BOXES =====
        img     = picam2.capture_array()
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        imgHSV  = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        h_img, w_img = img_bgr.shape[:2]
        center_x = w_img // 2

        # --- Diagonal line mask (for excluding from red/green) ---
        edges_full = preprocess_edges(img_bgr)
        _, _, line_mask_diag = detect_line_and_mask(edges_full, h_img, w_img)

        # --- ORANGE & BLUE line detection (Hough) ---
        mask_orange  = cv2.inRange(imgHSV, ORANGE_LO, ORANGE_HI)
        edges_orange = cv2.Canny(mask_orange, 50, 150)

        mask_blue    = cv2.inRange(imgHSV, BLUE_LO, BLUE_HI)
        edges_blue   = cv2.Canny(mask_blue, 50, 150)

        lines_orange = cv2.HoughLinesP(
            edges_orange, 1, np.pi/180,
            threshold=50, minLineLength=50, maxLineGap=10
        )
        lines_blue = cv2.HoughLinesP(
            edges_blue, 1, np.pi/180,
            threshold=50, minLineLength=50, maxLineGap=10
        )

        blue_len_max   = max_line_len(lines_blue)
        orange_len_max = max_line_len(lines_orange)

        orange_y = y_at_center(lines_orange, center_x)
        blue_y   = y_at_center(lines_blue, center_x)

        blue_trigger   = (blue_y   >= LINE_CENTER_BLUE_Y_MIN   and blue_len_max   >= BLUE_MIN_LEN_PX)
        orange_trigger = (orange_y >= LINE_CENTER_ORANGE_Y_MIN and orange_len_max >= BLUE_MIN_LEN_PX)
        line_trigger_raw = blue_trigger or orange_trigger

        # Choose the “closest” line Y for comparison with obstacle
        valid_ys = []
        if blue_trigger and blue_y >= 0:
            valid_ys.append(blue_y)
        if orange_trigger and orange_y >= 0:
            valid_ys.append(orange_y)
        line_y_for_turn = max(valid_ys) if valid_ys else -1

        # --- RED / GREEN obstacle masks ---
        # PINK to exclude from red
        mask_pink = cv2.inRange(
            imgHSV,
            np.array([140, 60, 120], dtype=np.uint8),
            np.array([170, 255, 255], dtype=np.uint8),
        )

        mask_red1 = cv2.inRange(imgHSV, RED1_LO, RED1_HI)
        mask_red2 = cv2.inRange(imgHSV, RED2_LO, RED2_HI)
        mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
        mask_red  = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_orange))
        mask_red  = cv2.bitwise_and(mask_red, cv2.bitwise_not(mask_pink))
        if line_mask_diag is not None:
            mask_red = cv2.bitwise_and(mask_red, cv2.bitwise_not(line_mask_diag))

        mask_green = cv2.inRange(imgHSV, GREEN_LO, GREEN_HI)
        if line_mask_diag is not None:
            mask_green = cv2.bitwise_and(mask_green, cv2.bitwise_not(line_mask_diag))

        # Morphological cleanup
        k3 = np.ones((3,3), np.uint8)
        k5 = np.ones((5,5), np.uint8)
        def morph(m):
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k3)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k5)
            return m

        mask_red   = morph(mask_red)
        mask_green = morph(mask_green)

        boxes = []

        red_data = thin_shape_reject(get_largest_contour(mask_red, min_area=MIN_AREA))
        if red_data:
            _, tl, br, area = red_data
            boxes.append(("Red", area, (*tl, *br)))

        green_data = thin_shape_reject(get_largest_contour(mask_green, min_area=MIN_AREA))
        if green_data:
            _, tl, br, area = green_data
            boxes.append(("Green", area, (*tl, *br)))

        if boxes:
            boxes.sort(key=lambda b: b[1], reverse=True)
            chosen_color, chosen_area, chosen_box = boxes[0]
        else:
            chosen_color = None
            chosen_area  = 0
            chosen_box   = None

        # Update color streak used for locking obstacle
        if chosen_color is not None:
            if chosen_color == last_color:
                color_hold_streak += 1
            else:
                last_color = chosen_color
                color_hold_streak = 1
        else:
            color_hold_streak = 0

        # Which is “closer”: line or obstacle?
        closest_obstacle_y = chosen_box[3] if chosen_box is not None else -1
        obstacle_closer = False
        line_closer     = False

        if chosen_area > 6000:
            obstacle_closer = True
            line_closer     = False
        else:
            if line_trigger_raw and line_y_for_turn >= 0 and closest_obstacle_y >= 0:
                if closest_obstacle_y > line_y_for_turn + OBSTACLE_LINE_MARGIN_PX:
                    obstacle_closer = True
                elif line_y_for_turn > closest_obstacle_y + OBSTACLE_LINE_MARGIN_PX:
                    line_closer = True
                else:
                    line_closer = True
            elif line_trigger_raw and closest_obstacle_y < 0:
                line_closer = True
            elif (not line_trigger_raw) and closest_obstacle_y >= 0:
                obstacle_closer = True

        # ===== DISTANCES (ToF + ultrasonic) =====
        f_cm = tof_cm(sensors["front"])
        b_cm = tof_cm(sensors["back"])
        l_cm = tof_cm(sensors["left"])
        r_cm = tof_cm(sensors["right"])

        f_ultra = get_front_ultra_cm()
        l_ultra = get_left_ultra_cm()
        r_ultra = get_right_ultra_cm()

        # Gate for starting a turn (based on front ultrasonic)
        tof_line_turn_gate = (TURN_FRONT_MIN_CM <= f_ultra <= TURN_FRONT_MAX_CM)

        # Turn cooldown logic
        time_since_last_turn = now - last_turn_end_time if last_turn_end_time > 0 else 1e9
        out_of_cooldown     = time_since_last_turn >= TURN_COOLDOWN_S
        out_of_min_interval = time_since_last_turn >= TURN_MIN_INTERVAL_S

        # Consecutive frames condition for line confirmation
        if (fsm_state == STATE_CRUISE and
            line_trigger_raw and
            tof_line_turn_gate and
            now >= post_turn_line_ignore_until and
            out_of_cooldown and out_of_min_interval and
            not obstacle_closer):

            blue_gate_streak += 1
        else:
            blue_gate_streak = 0

        line_confirmed = (blue_gate_streak >= LINE_DETECT_CONSEC_FRAMES)

        # ====== FSM ======
        if fsm_state == STATE_CRUISE:
            # ---- CRUISE: go straight with wall safety ----
            set_run_state("cruise")
            set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

            # Base IMU keep-straight (strong gains – no obstacle locked)
            target_angle = imu_center_servo(
                current_yaw,
                YAW_DEADBAND_DEG_STRONG,
                YAW_KP_STRONG,
                SERVO_CORR_LIMIT_STRONG
            )

            # Wall collision correction using ultrasonic (fallback to ToF)
            l_side = l_ultra if l_ultra < 999 else l_cm
            r_side = r_ultra if r_ultra < 999 else r_cm

            if l_side < SIDE_COLLIDE_CM and r_side >= SIDE_COLLIDE_CM:
                target_angle = LEFT_COLLIDE_ANGLE
            elif r_side < SIDE_COLLIDE_CM and l_side >= SIDE_COLLIDE_CM:
                target_angle = RIGHT_COLLIDE_ANGLE

            # ---- Priority: lock obstacle -> AVOID state ----
            if (chosen_color is not None and
                color_hold_streak >= COLOR_HOLD_FRAMES and
                not line_closer):

                obstacle_lock_color     = chosen_color
                obstacle_lock_last_area = chosen_area
                obstacle_lock_last_box  = chosen_box
                obstacle_clear_streak   = 0
                avoid_phase             = "back_off"
                avoid_phase_start       = now
                # For red we want to finally pass it on the right,
                # for green on the left -> reverse in the opposite direction first.
                avoid_direction         = "left" if obstacle_lock_color == "Red" else "right"

                dbg(f"FSM CRUISE -> AVOID (lock {obstacle_lock_color}, area={chosen_area}, box={chosen_box}, dir={avoid_direction})")
                fsm_state        = STATE_AVOID
                state_start_time = now
                set_run_state(f"avoid lock – {obstacle_lock_color.lower()}")

            # ---- Otherwise: line-based turn (if allowed) ----
            elif (line_confirmed and
                  (turn_count < TOTAL_TURNS) and
                  not obstacle_closer):

                fsm_state        = STATE_TURN
                state_start_time = now
                turn_dir = "Left" if direction == "left" else "Right"

                trig_color = "blue" if blue_trigger and not orange_trigger else \
                             "orange" if orange_trigger and not blue_trigger else \
                             "both"
                dbg(f"FSM CRUISE -> TURN (dir={turn_dir}, trig={trig_color}, f_ultra={f_ultra:.1f})")

                if blue_trigger and not orange_trigger:
                    set_run_state(f"blue line – turn {turn_dir.lower()}")
                elif orange_trigger and not blue_trigger:
                    set_run_state(f"orange line – turn {turn_dir.lower()}")
                else:
                    set_run_state(f"line – turn {turn_dir.lower()}")

                # Reset yaw for the turn
                with yaw_lock:
                    yaw = 0.0

        elif fsm_state == STATE_TURN:
            # ---- TURN: 90° line-based turn ----
            if turn_dir == "Left":
                target_angle = TURN_LEFT_SERVO
            else:
                target_angle = TURN_RIGHT_SERVO

            set_motor_speed(MOTOR_FWD, MOTOR_REV, TURN_MOTOR_SPEED)

            # Stop rule: yaw-based (failsafe)
            if abs(current_yaw) >= TURN_FAILSAFE_MAX_DEG:
                # Set yaw to a small offset after turn
                with yaw_lock:
                    if turn_dir == "Left":
                        yaw = YAW_RESET_AFTER_LEFT
                    else:
                        yaw = YAW_RESET_AFTER_RIGHT
                    current_yaw = yaw

                # Turn / lap counting
                turn_count += 1
                turn_in_lap = ((turn_count - 1) % TURNS_PER_LAP) + 1
                lap_count   = (turn_count - 1) // TURNS_PER_LAP + 1
                dbg(f"TURN end: dir={turn_dir}, yaw={current_yaw:.1f}, turn_count={turn_count}, lap={lap_count}")
                print(f"[LAP] turn {turn_in_lap} / lap {lap_count}", flush=True)

                # If all laps done -> final straight and stop
                if turn_count >= TOTAL_TURNS:
                    print("[LAP] All laps completed. Driving straight then stopping.", flush=True)
                    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                    time.sleep(3.5)
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
                    break

                # Otherwise go into post-turn reverse state
                fsm_state              = STATE_POST_TURN
                state_start_time       = now
                post_turn_start_time   = now
                last_turn_end_time     = now
                next_turn_allowed_time = now + TURN_COOLDOWN_SEC
                post_turn_line_ignore_until = now + POST_TURN_LINE_IGNORE_S
                settle_until_ts        = time.time() + SETTLE_DURATION
                dbg("FSM TURN -> POST_TURN (start reverse)")
                set_run_state("post-turn reverse")

                target_angle = CENTER_ANGLE
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -POST_TURN_BACK_SPEED)

        elif fsm_state == STATE_POST_TURN:
            # ---- POST_TURN: back up straight a bit ----
            target_angle = CENTER_ANGLE
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -POST_TURN_BACK_SPEED)

            back_cm      = tof_cm(sensors["back"])
            elapsed_back = now - post_turn_start_time

            if back_cm >= POST_TURN_BACK_CLEAR_CM or elapsed_back >= POST_TURN_BACK_TIMEOUT_S:
                dbg(f"POST_TURN done: back_cm={back_cm:.1f}, elapsed={elapsed_back:.2f}")
                # Done backing up – resume cruise
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                fsm_state        = STATE_CRUISE
                state_start_time = now
                settle_until_ts  = time.time() + SETTLE_DURATION
                dbg("FSM POST_TURN -> CRUISE")
                set_run_state("cruise")

        elif fsm_state == STATE_AVOID:
            # ---- AVOID: obstacle avoidance has priority ----
            locked_seen = (chosen_color == obstacle_lock_color)

            if avoid_phase == "back_off":
                # Short reverse with steering away, to create clearance
                elapsed = now - avoid_phase_start
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                target_angle = LEFT_NEAR if avoid_direction == "left" else RIGHT_NEAR

                if elapsed >= AVOID_BACK_DURATION:
                    avoid_phase       = "forward"
                    avoid_phase_start = now
                    dbg(f"AVOID phase switch: back_off -> forward (color={obstacle_lock_color})")
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                    obstacle_clear_streak = 0

            elif avoid_phase == "forward":
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)

                if locked_seen and chosen_box is not None:
                    # Obstacle still visible – follow around it
                    area       = chosen_area
                    base_angle = compute_servo_angle(obstacle_lock_color, area)

                    norm_area  = max(MIN_AREA, min(MAX_AREA, area))
                    closeness  = (norm_area - MIN_AREA) / float(MAX_AREA - MIN_AREA + 1e-6)
                    yaw_gain   = BOX_YAW_GAIN_MIN + closeness * (BOX_YAW_GAIN_MAX - BOX_YAW_GAIN_MIN)

                    if obstacle_lock_color == "Red":
                        # Red: obstacle should end up on the RIGHT of the robot
                        target_angle = max(60, min(120, int(base_angle + current_yaw * yaw_gain)))
                    else:
                        # Green: obstacle should end up on the LEFT of the robot
                        target_angle = max(60, min(120, int(base_angle - current_yaw * yaw_gain)))
                else:
                    # Locked obstacle not seen this frame – drive roughly straight
                    target_angle = imu_center_servo(
                        current_yaw,
                        YAW_DEADBAND_DEG_BASE,
                        YAW_KP_BASE,
                        SERVO_CORR_LIMIT_BASE
                    )

                # Hysteresis: we only exit AVOID when the locked color is
                # missing for OBSTACLE_CLEAR_FRAMES consecutive frames.
                if locked_seen:
                    obstacle_clear_streak = 0
                else:
                    obstacle_clear_streak += 1

                if obstacle_clear_streak >= OBSTACLE_CLEAR_FRAMES:
                    dbg(f"AVOID done: color={obstacle_lock_color}, clear_streak={obstacle_clear_streak}")
                    obstacle_lock_color   = None
                    obstacle_clear_streak = 0
                    avoid_phase           = None
                    fsm_state             = STATE_CRUISE
                    state_start_time      = now
                    settle_until_ts       = time.time() + SETTLE_DURATION
                    dbg("FSM AVOID -> CRUISE (obstacle cleared)")
                    set_run_state("cruise")

        # ==== SERVO OUTPUT (direct angle with optional settle) ====
        if (time.time() < settle_until_ts and
            fsm_state not in (STATE_TURN, STATE_AVOID)):
            desired = CENTER_ANGLE
        else:
            desired = target_angle

        current_servo_angle = int(max(60, min(120, desired)))
        set_servo_angle(SERVO_CHANNEL, current_servo_angle)

        # Optional: if you still want ESC to stop the program:
        if cv2.waitKey(1) in [27, ord('q')]:
            dbg("ESC pressed, exiting main loop")
            break

    time.sleep(0.1)

finally:
    dbg("Shutting down: stopping camera and motors")
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, STOP_SPEED)
    front_ultra.close()
    left_ultra.close()
    right_ultra.close()
    red_led.off()
    green_led.off()


