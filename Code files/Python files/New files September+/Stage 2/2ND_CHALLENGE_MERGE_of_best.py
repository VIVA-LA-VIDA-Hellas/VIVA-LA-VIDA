# ===============================================
# VIVA LaVida – FE 2025 Combined Navigation Script
# - BOX AVOIDANCE (camera red/green)
# - TURN DETECTION (blue/orange line) with "keep turning" rule
# - SMART UNPARK (ToF + button)
# - DEBUG LEDs: red (GPIO13), green (GPIO19), blue (GPIO11)
# - Area-based steering: farther box -> less turn, closest -> full turn
# Raspberry Pi 5 + Picamera2 + PCA9685 + MPU6050 + VL53L0X
# ===============================================

# ---------- Imports ----------
import time
import threading
import cv2
import numpy as np
import smbus2
import RPi.GPIO as GPIO
from gpiozero import Button
from picamera2 import Picamera2
from pca9685_control import set_servo_angle, set_motor_speed

# ToF deps (for Smart Unpark)
import board
import busio
import digitalio
import adafruit_vl53l0x

# ---------- CONSTANTS (tune here) ----------
# --- General ---
LOOP_DT_TARGET = 0.02            # main loop sleep (sec) for ~50 Hz
SHOW_WINDOWS = False             # set True to see video windows (slower)

# --- Motor channels / servo channel on PCA9685 ---
MOTOR_FWD = 1                    # motor forward channel on PCA9685
MOTOR_REV = 2                    # motor reverse channel on PCA9685
SERVO_CHANNEL = 0                # steering servo channel on PCA9685

# --- Steering geometry ---
CENTER_ANGLE = 90                # neutral servo angle
LEFT_NEAR  = 130                 # hard left when very close to red
RIGHT_NEAR = 60                  # hard right when very close to green
LEFT_FAR   = 110                 # soft left (legacy; not used in new area mapping)
RIGHT_FAR  = 70                  # soft right (legacy; not used in new area mapping)
SERVO_MIN  = 50                  # clamp lower bound
SERVO_MAX  = 130                 # clamp upper bound
STEP = 3                         # normal smoothing step (deg/update)
FAST_SERVO_STEP = 6              # faster step in reversal/turning
SERVO_UPDATE_DELAY = 0.02        # min time between servo updates (sec)

# --- Speeds ---
NORMAL_SPEED = 20                # forward cruise
AVOID_SPEED  = 20                # reverse speed during avoidance
BLUE_BACK_SPEED = 15             # reverse when intersecting with box

# --- "Blue-backward" (box intersect immediate reverse) ---
BLUE_BACK_DURATION = 1.0         # seconds to reverse when intersecting

# --- Avoidance (proximity via bounding-box intersect) ---
AVOID_BACK_DURATION = 1.0        # reverse time for avoidance (sec)

# --- Box detection areas (distance proxy) ---
MIN_AREA = 2000                  # min contour area to consider a box
MAX_AREA = 20000                 # area mapped to "very close"
COLOR_HOLD_FRAMES = 5            # frames to confirm chosen color

# >>> area response shaping <<<
AREA_RESPONSE_GAMMA = 1.6        # >1 = gentler when far, ramps up close-in

# --- Camera / Car footprint in image ---
CAM_SIZE = (640, 480)            # camera preview size (WxH)
CAR_BOX_WIDTH  = 350             # bounding box width for our car footprint
CAR_BOX_HEIGHT = 100             # bounding box height
CAR_BOX_BOTTOM_MARGIN = 10       # px from bottom

# --- TURN detection (blue/orange Hough lines) ---
LINE_CENTER_Y_MIN = 400          # min y (from top) needed to consider "near"
TURN_LEFT_SERVO  = 60            # steering angle while left turn is active
TURN_RIGHT_SERVO = 120           # steering angle while right turn is active
TURN_MIN_YAW_DEG = 65.0          # MUST reach at least this yaw before stopping turn
TURN_FAILSAFE_MAX_DEG = 90.0     # stop turning even if no box is seen (safety)
TURN_MOTOR_SPEED = 20            # speed while turning

# Post-turn yaw reset/bias (helps stabilise straightening)
YAW_RESET_AFTER_LEFT  = -12.0    # set yaw to this after left turn finishes
YAW_RESET_AFTER_RIGHT =  12.0    # set yaw to this after right turn finishes

# --- IMU (MPU6050) ---
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
GYRO_ZOUT_H  = 0x47
YAW_CLAMP_DEG = 120.0            # sanity clamp for yaw
# IMU straighten (proportional)
YAW_KP            = 1.0          # proportional gain (deg->servo delta)
YAW_DEADBAND_DEG  = 3.0          # ignore small drift
SERVO_CORR_LIMIT  = 22           # max correction from yaw (deg)

# --- HSV thresholds (tune for venue lighting) ---
RED1_LO = np.array([0,   100, 80], dtype=np.uint8)
RED1_HI = np.array([10,  255, 255], dtype=np.uint8)
RED2_LO = np.array([170, 100, 80], dtype=np.uint8)
RED2_HI = np.array([180, 255, 255], dtype=np.uint8)
GREEN_LO = np.array([35, 60, 60], dtype=np.uint8)
GREEN_HI = np.array([95, 255, 255], dtype=np.uint8)
ORANGE_LO = np.array([0,  70, 150], dtype=np.uint8)
ORANGE_HI = np.array([20, 200, 230], dtype=np.uint8)
BLUE_LO   = np.array([70,  0, 130], dtype=np.uint8)
BLUE_HI   = np.array([125, 170, 170], dtype=np.uint8)

# --- LEDs (BCM pins) ---
LED_BLUE_PIN  = 11               # ON while turning
LED_RED_PIN   = 13               # ON when red box seen / avoiding red
LED_GREEN_PIN = 19               # ON when green box seen / avoiding green

# --- Smart Unpark (kept from your code) ---
START_BUTTON_PIN = 20            # button to start unpark
UNPARK_SPEED = 17                # speed during unpark
UNPARK_LEFT_TURN_ANGLE  = 55     # phase steering if going left first
UNPARK_RIGHT_TURN_ANGLE = 120    # phase steering if going right first
UNPARK_PHASE1_TARGET_DEG = 45.0  # first yaw segment
UNPARK_PHASE2_TARGET_DEG = 40.0  # second yaw segment
UNPARK_EXTRA_LEFT_DEG    = 90.0  # extra spin if initial direction is left

# ToF XSHUT pins (match your wiring)
XSHUT_PINS = {
    "left":  board.D16,
    "right": board.D25,
    "front": board.D26,
    "back":  board.D24
}
# Readdressed ToF I2C addresses
TOF_ADDR = {
    "left":  0x30,
    "right": 0x31,
    "front":  0x32,
    "back":   0x33
}
TOF_RANGE_BAD = 999.0            # sentinel cm value when reading fails
TOF_VALID_MAX = 150.0            # drop out-of-range returns

# ---------- Helpers / Setup ----------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

# ### area-based steering (distance proxy = contour area)
def _area_closeness(area):
    # 0.0 at/below MIN_AREA (far) -> 1.0 at/above MAX_AREA (very close)
    a = clamp(area, MIN_AREA, MAX_AREA)
    frac = (a - MIN_AREA) / float(MAX_AREA - MIN_AREA + 1e-6)
    # gamma shaping: >1 makes it gentle when far, aggressive close-in
    return pow(clamp(frac, 0.0, 1.0), AREA_RESPONSE_GAMMA)

def compute_servo_angle(color, area):
    """
    NEW mapping:
      - Far box  -> angle near CENTER (tiny turn)
      - Close box -> FULL to LEFT_NEAR / RIGHT_NEAR
    """
    w = _area_closeness(area)  # 0..1
    near = LEFT_NEAR if color == "Red" else RIGHT_NEAR
    # interpolate from CENTER toward NEAR by w
    return int(round(CENTER_ANGLE + w * (near - CENTER_ANGLE)))

def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    x,y,w,h = cv2.boundingRect(c)
    return c, (x,y), (x+w, y+h), w*h

def boxes_intersect(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return not (ax2 < bx1 or ax1 > bx2 or ay2 < by1 or ay1 > by2)

def imu_center_servo(yaw_deg):
    """Centering based on yaw (simple P)."""
    if abs(yaw_deg) <= YAW_DEADBAND_DEG:
        return CENTER_ANGLE
    corr = clamp(YAW_KP * yaw_deg, -SERVO_CORR_LIMIT, SERVO_CORR_LIMIT)
    return int(CENTER_ANGLE + corr)

def y_at_center(lines, center_x):
    """Return the maximum y at the image center from lines (for 'lower is closer')."""
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

def set_leds(red=False, green=False, blue=False):
    GPIO.output(LED_RED_PIN,   GPIO.HIGH if red else GPIO.LOW)
    GPIO.output(LED_GREEN_PIN, GPIO.HIGH if green else GPIO.LOW)
    GPIO.output(LED_BLUE_PIN,  GPIO.HIGH if blue else GPIO.LOW)

# ---------- Hardware Init ----------
GPIO.setmode(GPIO.BCM)
for p in [LED_BLUE_PIN, LED_RED_PIN, LED_GREEN_PIN]:
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

# Camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": CAM_SIZE}))
picam2.start()
time.sleep(1.5)

# Servo center
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE
last_servo_update = time.time()

# IMU
bus = smbus2.SMBus(1)
def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

def read_raw(addr):
    hi = bus.read_byte_data(MPU6050_ADDR, addr)
    lo = bus.read_byte_data(MPU6050_ADDR, addr+1)
    val = (hi << 8) | lo
    if val > 32767: val -= 65536
    return val

mpu6050_init()
print("Measuring gyro bias, keep sensor still...")
def get_gyro_z_bias(samples=200):
    tot = 0.0
    for _ in range(samples):
        tot += read_raw(GYRO_ZOUT_H) / 131.0
        time.sleep(0.005)
    return tot / samples
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

# ToF – readdress and continuous ranging (for Smart Unpark)
i2c = busio.I2C(board.SCL, board.SDA)
xshuts = {}
for name, pin in XSHUT_PINS.items():
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
    s.set_address(TOF_ADDR[name])
    s.start_continuous()
    sensors[name] = s
    print(f"[ToF] {name.upper()} @ {hex(TOF_ADDR[name])}")

def tof_cm(sensor):
    try:
        v = sensor.range / 10.0
        if v <= 0 or v > TOF_VALID_MAX: return TOF_RANGE_BAD
        return v
    except:
        return TOF_RANGE_BAD

# ---------- Smart Unpark (kept) ----------
start_button = Button(START_BUTTON_PIN)
print("\n=== SMART UNPARK READY ===")
print("Press the start button to begin...")
start_button.wait_for_press()
print("Button pressed! Starting unpark in 1s...")
time.sleep(1)

left_d  = tof_cm(sensors["left"])
right_d = tof_cm(sensors["right"])
print(f"[UNPARK] Left={left_d:.1f}cm | Right={right_d:.1f}cm")
direction = "left" if left_d > right_d else "right"
print(f"[UNPARK] Choosing {direction.upper()}")

# Phase 1 (~45°)
with yaw_lock: yaw = 0.0
last_time = time.time()
first_angle = UNPARK_LEFT_TURN_ANGLE if direction=="left" else UNPARK_RIGHT_TURN_ANGLE
set_servo_angle(SERVO_CHANNEL, first_angle)
set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_SPEED)
while True:
    now = time.time(); dt = now - last_time; last_time = now
    Gz = (read_raw(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
    with yaw_lock:
        yaw += Gz * dt
        if abs(yaw) >= UNPARK_PHASE1_TARGET_DEG: break

# Phase 2 (~40°)
with yaw_lock: yaw = 0.0
last_time = time.time()
second_angle = UNPARK_RIGHT_TURN_ANGLE if direction=="left" else UNPARK_LEFT_TURN_ANGLE
set_servo_angle(SERVO_CHANNEL, second_angle)
set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_SPEED)
while True:
    now = time.time(); dt = now - last_time; last_time = now
    Gz = (read_raw(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
    with yaw_lock:
        yaw += Gz * dt
        if abs(yaw) >= UNPARK_PHASE2_TARGET_DEG: break

# Extra ~90 only if initial left
if direction == "left":
    with yaw_lock: yaw = 0.0
    last_time = time.time()
    set_servo_angle(SERVO_CHANNEL, UNPARK_LEFT_TURN_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_SPEED)
    while True:
        now = time.time(); dt = now - last_time; last_time = now
        Gz = (read_raw(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            if abs(yaw) >= UNPARK_EXTRA_LEFT_DEG: break

# Straighten and reset yaw
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
set_motor_speed(MOTOR_FWD, MOTOR_REV, UNPARK_SPEED)
with yaw_lock: yaw = 0.0
print("[UNPARK] Complete. Entering main loop...")

# ---------- Main State ----------
last_color = None
frame_count = 0
in_blue_backward = False
blue_backward_start = None

avoidance_mode = False
avoid_start_time = 0.0
avoid_direction = None        # "left" or "right"
avoid_color = None            # "Red" or "Green" (for LEDs)
avoid_area_at_trigger = None  # ### area-based steering for avoidance

turn_active = False
turn_dir = None               # "Left" or "Right"
box_seen_while_turning = False

try:
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    time.sleep(0.2)

    while True:
        # ----- Time / IMU -----
        now = time.time()
        dt = now - last_time
        last_time = now

        Gz = (read_raw(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            yaw = clamp(yaw, -YAW_CLAMP_DEG, YAW_CLAMP_DEG)
            current_yaw = yaw

        # ----- Camera -----
        img = picam2.capture_array()        # RGB
        h, w = img.shape[:2]
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        center_x = w // 2

        # ----- Box masks -----
        mask_red   = cv2.bitwise_or(cv2.inRange(imgHSV, RED1_LO, RED1_HI),
                                    cv2.inRange(imgHSV, RED2_LO, RED2_HI))
        mask_green = cv2.inRange(imgHSV, GREEN_LO, GREEN_HI)

        red_data   = get_largest_contour(mask_red,   min_area=MIN_AREA)
        green_data = get_largest_contour(mask_green, min_area=MIN_AREA)

        boxes = []
        if red_data:
            _, tl, br, area = red_data
            boxes.append(("Red", area, (*tl, *br)))
        if green_data:
            _, tl, br, area = green_data
            boxes.append(("Green", area, (*tl, *br)))

        # ----- LEDs for "seeing" boxes (also set again during avoidance) -----
        red_seen = red_data is not None
        green_seen = green_data is not None

        # ----- Car footprint in image (for intersect checks) -----
        car_box = (center_x - CAR_BOX_WIDTH//2,
                   h - CAR_BOX_BOTTOM_MARGIN - CAR_BOX_HEIGHT,
                   center_x + CAR_BOX_WIDTH//2,
                   h - CAR_BOX_BOTTOM_MARGIN)

        # ----- TURN detection via orange/blue lines -----
        mask_orange = cv2.inRange(imgHSV, ORANGE_LO, ORANGE_HI)
        mask_blue   = cv2.inRange(imgHSV, BLUE_LO, BLUE_HI)
        edges_orange = cv2.Canny(mask_orange, 50, 150)
        edges_blue   = cv2.Canny(mask_blue,   50, 150)
        lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi/180,
                                       threshold=50, minLineLength=50, maxLineGap=10)
        lines_blue   = cv2.HoughLinesP(edges_blue,   1, np.pi/180,
                                       threshold=50, minLineLength=50, maxLineGap=10)

        orange_y = y_at_center(lines_orange, center_x)
        blue_y   = y_at_center(lines_blue,   center_x)

        # Decide requested turn
        Turn = "No"
        if orange_y > blue_y and orange_y > LINE_CENTER_Y_MIN:
            Turn = "Right"
        elif blue_y > orange_y and blue_y > LINE_CENTER_Y_MIN:
            Turn = "Left"

        # ----- TURN state machine -----
        if (Turn in ("Left","Right")) and not turn_active and not in_blue_backward and not avoidance_mode:
            turn_active = True
            turn_dir = Turn
            box_seen_while_turning = False
            with yaw_lock: yaw = 0.0
            last_time = time.time()
            print(f"[TURN] Start {turn_dir}")

        # LED: blue while turning
        if turn_active:
            set_leds(blue=True, red=red_seen, green=green_seen)
        else:
            set_leds(blue=False, red=red_seen, green=green_seen)

        # Motor/steer while turning
        if turn_active:
            target_angle = TURN_LEFT_SERVO if turn_dir=="Left" else TURN_RIGHT_SERVO
            set_servo_angle(SERVO_CHANNEL, target_angle)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, TURN_MOTOR_SPEED)

            # Track if any box is in view
            if red_seen or green_seen:
                box_seen_while_turning = True

            # Stop turning ONLY when a box is seen AND minimum yaw is achieved
            stop_for_box = box_seen_while_turning and (abs(current_yaw) >= TURN_MIN_YAW_DEG)
            failsafe_yaw = abs(current_yaw) >= TURN_FAILSAFE_MAX_DEG

            if stop_for_box or failsafe_yaw:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                if turn_dir == "Left":
                    with yaw_lock: yaw = YAW_RESET_AFTER_LEFT
                else:
                    with yaw_lock: yaw = YAW_RESET_AFTER_RIGHT
                current_servo_angle = CENTER_ANGLE
                print(f"[TURN] End {turn_dir} | yaw={current_yaw:.1f}° | box_seen={box_seen_while_turning}")
                turn_active = False
                turn_dir = None
            time.sleep(LOOP_DT_TARGET)
            continue  # skip to next frame; avoidance disabled while turning

        # ----- Immediate "blue-backward" if intersecting a box -----
        if not in_blue_backward:
            if red_data and boxes_intersect(car_box, (*red_data[1], *red_data[2])):
                in_blue_backward = True
                blue_backward_start = time.time()
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                print("[BACK] Intersect Red -> reversing")
            elif green_data and boxes_intersect(car_box, (*green_data[1], *green_data[2])):
                in_blue_backward = True
                blue_backward_start = time.time()
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                print("[BACK] Intersect Green -> reversing")

        if in_blue_backward:
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
            if (time.time() - blue_backward_start) >= BLUE_BACK_DURATION:
                in_blue_backward = False
                with yaw_lock: yaw = 0.0
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                print("[BACK] Done")
            time.sleep(LOOP_DT_TARGET)
            continue

        # ----- Box-avoidance trigger (reverse steer away) -----
        if not avoidance_mode:
            for color, area, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode = True
                    avoid_direction = "right" if color == "Green" else "left"
                    avoid_color = color
                    avoid_area_at_trigger = area      # ### remember area at trigger
                    avoid_start_time = time.time()
                    print(f"[AVOID] {color} intersects -> reversing {avoid_direction.upper()}")
                    break

        # LEDs also reflect avoidance color
        if avoidance_mode and avoid_color == "Red":
            GPIO.output(LED_RED_PIN, GPIO.HIGH)
        if avoidance_mode and avoid_color == "Green":
            GPIO.output(LED_GREEN_PIN, GPIO.HIGH)

        # ----- Avoidance state (area-based steering while reversing) -----
        if avoidance_mode:
            elapsed = time.time() - avoid_start_time
            if elapsed < AVOID_BACK_DURATION:
                # pick freshest area for that color if available
                if avoid_color == "Red":
                    cur_area = (red_data[3] if red_data else avoid_area_at_trigger)
                else:
                    cur_area = (green_data[3] if green_data else avoid_area_at_trigger)
                steer = compute_servo_angle(avoid_color, cur_area)  # ### area-based
                set_servo_angle(SERVO_CHANNEL, steer)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
            else:
                # go forward and re-center with IMU
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                target_angle = imu_center_servo(current_yaw)
                set_servo_angle(SERVO_CHANNEL, clamp(target_angle, SERVO_MIN, SERVO_MAX))
                # clear once clear of current boxes
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    avoid_direction = None
                    avoid_color = None
                    avoid_area_at_trigger = None
                    with yaw_lock: yaw = 0.0
                    print("[AVOID] Done")
            time.sleep(LOOP_DT_TARGET)
            continue

        # ----- Normal forward + IMU straighten + colored guidance (area-based) -----
        set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
        target_angle = imu_center_servo(current_yaw)

        if boxes:
            boxes.sort(key=lambda b: b[1], reverse=True)
            chosen_color, chosen_area, _ = boxes[0]
            if last_color == chosen_color:
                frame_count += 1
            else:
                frame_count = 0
                last_color = chosen_color

            if frame_count >= COLOR_HOLD_FRAMES:
                # ### area-based steering toward that color + yaw assist
                area_angle = compute_servo_angle(chosen_color, chosen_area)
                if chosen_color == "Red":
                    target_angle = clamp(int(area_angle + int(current_yaw * 2)), SERVO_MIN, SERVO_MAX)
                elif chosen_color == "Green":
                    target_angle = clamp(int(area_angle - int(current_yaw * 2)), SERVO_MIN, SERVO_MAX)

        # ----- Servo smoothing / output -----
        if (time.time() - last_servo_update) >= SERVO_UPDATE_DELAY:
            step = STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            current_servo_angle = clamp(current_servo_angle, SERVO_MIN, SERVO_MAX)
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_servo_update = time.time()

        time.sleep(LOOP_DT_TARGET)

finally:
    # Safe stop/cleanup
    try:
        set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
        set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    except Exception:
        pass
    set_leds(False, False, False)
    GPIO.cleanup()
    try:
        picam2.stop()
        cv2.destroyAllWindows()
    except Exception:
        pass
    print("Stopped & cleaned up.")
