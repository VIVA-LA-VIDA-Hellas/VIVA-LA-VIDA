import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import board
import busio
import adafruit_vl53l0x
import smbus  # MPU6050

# ==== I2C + TOF SENSOR ====
i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

# ==== MPU6050 SETUP ====
bus = smbus.SMBus(1)
MPU_ADDR = 0x68
bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # wake up
ACCEL_XOUT_H, GYRO_ZOUT_H = 0x3B, 0x47

def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr+1)
    val = (high << 8) | low
    if val > 32768: val -= 65536
    return val

gyro_reference = None

# ==== MOTOR + SERVO CONSTANTS ====
MOTOR_FWD, MOTOR_REV = 1, 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR, LEFT_NEAR = 70, 75   # stronger left
RIGHT_FAR, RIGHT_NEAR = 110, 95
LEFT_CENTER, RIGHT_CENTER = 65, 115

STEP, FAST_SERVO_STEP = 4, 4
SERVO_UPDATE_DELAY = 0.04
MIN_AREA, MAX_AREA = 2000, 20000
COLOR_HOLD_FRAMES = 5

# ==== SERVO INIT ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA ====
picam2 = Picamera2()
capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
picam2.configure(capture_config)
picam2.start()
time.sleep(2)

# ==== DETECTION HELPERS ====
def get_largest_contour(mask, min_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area: return None
    x, y, w, h = cv2.boundingRect(largest)
    return largest, (x, y), (x + w, y + h), w*h

def compute_servo_angle(color, area):
    # Make red → proportional right turn, green → strong left turn
    if color == "Red":
        norm_area = max(MIN_AREA, min(MAX_AREA, area))
        closeness = (norm_area - MIN_AREA) / (MAX_AREA - MIN_AREA)
        return int(RIGHT_FAR + closeness*(RIGHT_NEAR-RIGHT_FAR))
    elif color == "Green":
        return LEFT_FAR  # force stronger left
    return CENTER_ANGLE

def boxes_intersect(box1, box2):
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    return not (x1_max < x2_min or x1_min > x2_max or y1_max < y2_min or y1_min > y2_max)

# ==== STATES ====
last_color, frame_count = None, 0
last_update_time = time.time()
state, state_start = "normal", time.time()
in_blue_backward, blue_backward_start = False, None
avoidance_mode, avoid_direction, avoid_start_time = False, None, None

BLUE_BACK_DURATION, BLUE_BACK_SPEED = 1.5, 20
AVOID_BACK_DURATION = 1.5

# ==== MAIN LOOP ====
try:
    motors_started = False
    while True:
        # ==== Read MPU6050 ====
        gyro_z = read_raw_data(GYRO_ZOUT_H)/131.0
        if gyro_reference is None:
            gyro_reference = gyro_z

        # ==== Camera frame ====
        img = picam2.capture_array()
        h_img, w_img = img.shape[:2]
        scale = 1080/h_img
        img_small = cv2.resize(img, (int(w_img*scale), 1080))
        imgHSV = cv2.cvtColor(img_small, cv2.COLOR_RGB2HSV)

        # Red mask (two ranges)
        mask_red1 = cv2.inRange(imgHSV, np.array([0,100,80]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(imgHSV, np.array([170,100,80]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # Green mask (tighter for better detection)
        mask_green = cv2.inRange(imgHSV, np.array([40,80,80]), np.array([85,255,255]))

        red_data = get_largest_contour(mask_red)
        green_data = get_largest_contour(mask_green)

        boxes = []
        if red_data: _, tl, br, area = red_data; boxes.append(("Red", area, (*tl,*br)))
        if green_data: _, tl, br, area = green_data; boxes.append(("Green", area, (*tl,*br)))

        # Start motors after detection
        if boxes and not motors_started:
            motors_started = True
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 25)

        # Car virtual box
        cx = img_small.shape[1]//2
        car_box = (cx-400, img_small.shape[0]-225, cx+400, img_small.shape[0]-10)

        target_angle = CENTER_ANGLE

        # ==== BLUE-BACKWARD LOGIC ====
        if not in_blue_backward and any(boxes_intersect(car_box, b[2]) for b in boxes):
            in_blue_backward, blue_backward_start = True, time.time()
            set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)

        if in_blue_backward:
            target_angle = CENTER_ANGLE
            if time.time()-blue_backward_start >= BLUE_BACK_DURATION:
                in_blue_backward = False
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 30)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -BLUE_BACK_SPEED)
                continue

        # ==== AVOIDANCE ====
        if motors_started and not avoidance_mode:
            for color, _, box_coords in boxes:
                if boxes_intersect(car_box, box_coords):
                    avoidance_mode, avoid_direction, avoid_start_time = True, ("left" if color=="Green" else "right"), time.time()
                    set_motor_speed(MOTOR_FWD, MOTOR_REV, -20)
                    target_angle = LEFT_FAR if avoid_direction=="left" else RIGHT_FAR
                    state, state_start = "avoid", time.time()
                    break

        if avoidance_mode:
            if time.time()-avoid_start_time < AVOID_BACK_DURATION:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -20)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 30)
                target_angle = LEFT_FAR if avoid_direction=="left" else RIGHT_FAR
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode, state = False, "normal"

        # ==== NORMAL FOLLOW ====
        if not avoidance_mode and state=="normal":
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 30)
            if boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, _ = boxes[0]
                if last_color == chosen_color: frame_count += 1
                else: frame_count, last_color = 0, chosen_color
                if frame_count >= COLOR_HOLD_FRAMES:
                    target_angle = compute_servo_angle(chosen_color, chosen_area)

        # ==== SERVO SMOOTHING ====
        if time.time()-last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if avoidance_mode else STEP
            if abs(current_servo_angle-target_angle) > step:
                current_servo_angle += step if current_servo_angle<target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        cv2.imshow("Contours", img_small)
        if cv2.waitKey(1) & 0xFF in [27, ord("q")]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, 120)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
