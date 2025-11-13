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
from gpiozero import Button

# ===============================
# BUTTON
# ===============================
start_button = Button(20)

# ===============================
# MOTOR / SERVO CHANNELS
# ===============================
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0

CENTER_ANGLE = 88          # keep your center
LEFT_TURN_ANGLE = 55       # unparking left
RIGHT_TURN_ANGLE = 120     # unparking right
STRAIGHT_SPEED = 17

# ==== AVOID / STEERING CONSTANTS ====
LEFT_FAR = 95
LEFT_NEAR = 130
RIGHT_FAR = 85
RIGHT_NEAR = 50

# smoother servo
STEP = 2                   # was 3 – smoother
FAST_SERVO_STEP = 4        # was 6 – smoother in avoidance
SERVO_UPDATE_DELAY = 0.01  # faster updates

MIN_AREA = 2000
MAX_AREA = 20000
COLOR_HOLD_FRAMES = 5

# ==== BLUE-BOX BACKWARD LOGIC ====
blue_backward_start = None
in_blue_backward = False
BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 15

# ==== AVOIDANCE LOGIC ====
AVOID_BACK_DURATION = 0.4   # NEW: shorter back step -> reacts quicker
AVOID_SPEED = 15

# ==== NORMAL LINE FOLLOWING SPEED ====
NORMAL_SPEED = 13
SLOW_SPEED = 9              # NEW: when box is very close

# ===============================
# I2C + ToF Setup
# ===============================
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

# ===============================
# MPU6050 IMU Setup (ONLY ONCE)
# ===============================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
GYRO_ZOUT_H  = 0x47
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

print("Initializing IMU...")
mpu6050_init()
print("Measuring gyro bias, keep sensor still...")
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

# ==== SERVO SETUP ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA SETUP ====
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ==== VISION HELPERS ====
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

# ===============================
# MAIN SEQUENCE
# ===============================
print("\n=== SMART UNPARK START ===")
start_button.wait_for_press()
print("Button pressed! Starting sequence...")

# ==== 1️⃣ Read ToF sensors and choose direction ====
left_dist = tof_cm(sensors["left"])
right_dist = tof_cm(sensors["right"])
print(f"Left={left_dist:.1f}cm | Right={right_dist:.1f}cm")

if left_dist > right_dist:
    direction = "left"
    print("➡ Choosing LEFT (more open).")
    target_angle_turn = LEFT_TURN_ANGLE
else:
    direction = "right"
    print("➡ Choosing RIGHT (more open).")
    target_angle_turn = RIGHT_TURN_ANGLE

# ==== 2️⃣ Start unparking ====
print("[STEP 1] Moving servo to 55° and driving forward...")
set_servo_angle(SERVO_CHANNEL, 50)
set_motor_speed(MOTOR_FWD, MOTOR_REV, STRAIGHT_SPEED)
with yaw_lock:
    yaw = 0.0
last_time = time.time()

# ==== Move until yaw reaches ~15° (you had 45 in print, 15 in code) ====
while True:
    now = time.time()
    dt = now - last_time
    last_time = now
    Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
    with yaw_lock:
        yaw += Gz * dt
        current_yaw = yaw
    print(f"Yaw={current_yaw:.2f}° (target 15°)")
    if abs(current_yaw) >= 15.0:
        break
    time.sleep(0.02)

print("Reached ~15° yaw → switching to turn phase.")
set_motor_speed(MOTOR_FWD, MOTOR_REV, STRAIGHT_SPEED)
with yaw_lock:
    yaw = 0.0
last_time = time.time()

# ==== 3️⃣ Turn (left/right based on direction) until yaw = 25° ====
print(f"[STEP 2] Turning {direction.upper()} until yaw=25°...")
set_servo_angle(SERVO_CHANNEL, target_angle_turn)
while True:
    now = time.time()
    dt = now - last_time
    last_time = now
    Gz = (read_raw_data(GYRO_ZOUT_H)/131.0) - gyro_z_bias
    with yaw_lock:
        yaw += Gz * dt
        current_yaw = yaw
    print(f"Yaw={current_yaw:.2f}° (target 25°)")
    if abs(current_yaw) >= 25.0:
        break
    time.sleep(0.02)

# ==== 4️⃣ Straighten and continue ====
print("[STEP 3] Straightening wheels and moving forward...")
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
set_motor_speed(MOTOR_FWD, MOTOR_REV, STRAIGHT_SPEED)

# ==== MAIN NAVIGATION LOOP ====
try:
    motors_started = True
    avoidance_mode = False
    avoid_direction = None
    avoid_start_time = None

    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    current_servo_angle = CENTER_ANGLE
    last_update_time = time.time()

    # brief pause
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

        # ==== BLUE-BACKWARD LOGIC (KEEP) ====
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
            for color, area, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode = True
                    avoid_direction = "right" if color == "Green" else "left"
                    avoid_start_time = time.time()

                    # NEW: immediate reaction – small back then steer
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
                    target_angle = RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR
                    state = "avoid"
                    state_start = time.time()
                    break

        if avoidance_mode:
            elapsed = time.time() - avoid_start_time
            if elapsed < AVOID_BACK_DURATION:
                # short back, strong steering
                set_servo_angle(SERVO_CHANNEL, RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -AVOID_SPEED)
            else:
                # then go forward while steering away
                set_motor_speed(MOTOR_FWD, MOTOR_REV, NORMAL_SPEED)
                target_angle = RIGHT_NEAR if avoid_direction == "right" else LEFT_NEAR

                # if no more intersection, go back to normal
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    state = "normal"
                    # reset yaw a bit when finished avoiding to reduce drift
                    with yaw_lock:
                        yaw = 0.0

        # ==== NORMAL LINE-FOLLOWING & IMU CORRECTION ====
        if not avoidance_mode and state == "normal":
            # speed depends on proximity of biggest box
            motor_speed = NORMAL_SPEED
            if boxes:
                # choose closest box (biggest area)
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, box_coords = boxes[0]

                # NEW: slow down when very close
                if chosen_area > 0.7 * MAX_AREA:
                    motor_speed = SLOW_SPEED

            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

            target_angle = CENTER_ANGLE

            if boxes:
                chosen_color, chosen_area, box_coords = boxes[0]

                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count = 0
                    last_color = chosen_color

                if frame_count >= COLOR_HOLD_FRAMES:
                    # base color angle
                    if chosen_color == "Red":
                        color_angle = LEFT_NEAR
                        # NEW: weaker IMU correction (was yaw * 2)
                        target_angle = color_angle + int(yaw * 0.8)
                    elif chosen_color == "Green":
                        color_angle = RIGHT_NEAR
                        target_angle = color_angle - int(yaw * 0.8)
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
