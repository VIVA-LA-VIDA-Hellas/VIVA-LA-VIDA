
import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import board
import busio
import adafruit_vl53l0x

# ==== I2C + SENSOR ====
i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

# ==== MOTOR + SERVO CONSTANTS ====
MOTOR_FWD = 1
MOTOR_REV = 2
SERVO_CHANNEL = 0
CENTER_ANGLE = 90
LEFT_FAR, LEFT_NEAR = 80, 75
RIGHT_FAR, RIGHT_NEAR = 95, 110
LEFT_CENTER, RIGHT_CENTER = 65, 115

KP = 0.5
STEP = 4
SERVO_UPDATE_DELAY = 0.04

MIN_AREA, MAX_AREA = 2000, 20000
COLOR_HOLD_FRAMES = 5

BLUE_BACK_DURATION = 1.5
BLUE_BACK_SPEED = 20

# ==== SERVO INIT ====
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
current_servo_angle = CENTER_ANGLE

# ==== CAMERA INIT ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (1920, 1080)}))
picam2.start()
time.sleep(2)

# ==== COLOR MASKS ====
def get_masks(imgHSV):
    red_lower1 = np.array([0, 150, 120])
    red_upper1 = np.array([20, 255, 255])
    mask_red = cv2.inRange(imgHSV, red_lower1, red_upper1)

    green_lower = np.array([60, 70, 80])
    green_upper = np.array([80, 210, 140])
    mask_green = cv2.inRange(imgHSV, green_lower, green_upper)

    return mask_red, mask_green

# ==== DETECTION ====
def get_largest_contour_corners(mask, min_area=500, min_width=100, min_height=100):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    # NEW: filter small detections by width & height (100x100)
    if w < min_width or h < min_height:
        return None
    top_left, bottom_right = (x, y), (x + w, y + h)
    return largest, top_left, bottom_right, cv2.contourArea(largest)

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

# ==== STATE VARS ====
last_color, frame_count = None, 0
last_update_time = time.time()
state, state_start = "normal", time.time()
HOLD_DURATION = 1.0

motors_started = False
avoidance_mode = False
avoid_direction = None
avoid_start_time = None
in_blue_backward = False
blue_backward_start = None
FAST_SERVO_STEP = 4

# ==== MAIN LOOP ====
try:
    while True:
        img = picam2.capture_array()
        imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask_red, mask_green = get_masks(imgHSV)
        red_data = get_largest_contour_corners(mask_red)
        green_data = get_largest_contour_corners(mask_green)

        img_contours = img.copy()
        boxes = []

        if red_data:
            contour, tl, br, area = red_data
            cv2.rectangle(img_contours, tl, br, (0, 0, 255), 2)
            boxes.append(("Red", area, (*tl, *br)))
        if green_data:
            contour, tl, br, area = green_data
            cv2.rectangle(img_contours, tl, br, (0, 255, 0), 2)
            boxes.append(("Green", area, (*tl, *br)))

        # ==== Car virtual box ====
        center_x = img_contours.shape[1] // 2
        car_width, car_height = 750, 200
        bottom_y = img_contours.shape[0] - 10
        car_box = (center_x - car_width // 2, bottom_y - car_height,
                   center_x + car_width // 2, bottom_y)
        cv2.rectangle(img_contours, (car_box[0], car_box[1]), (car_box[2], car_box[3]), (255, 0, 0), 2)

        # ==== Start motors after first detection ====
        if boxes and not motors_started:
            motors_started = True
            motor_speed = 25
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)

        target_angle, motor_speed = CENTER_ANGLE, 0

        # ==== AVOIDANCE LOGIC ====
        if motors_started:
            if not avoidance_mode:
                for color, _, box_coords in boxes:
                    if boxes_intersect(car_box, box_coords):
                        avoidance_mode = True
                        avoid_direction = "left" if color == "Green" else "right"
                        avoid_start_time = time.time()
                        set_motor_speed(MOTOR_FWD, MOTOR_REV, -20)
                        target_angle = LEFT_FAR if avoid_direction == "left" else RIGHT_FAR
                        state = "avoid"
                        state_start = time.time()
                        break

        if avoidance_mode:
            elapsed = time.time() - avoid_start_time
            if elapsed < 1.5:
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                set_motor_speed(MOTOR_FWD, MOTOR_REV, -20)
            else:
                set_motor_speed(MOTOR_FWD, MOTOR_REV, 30)
                target_angle = LEFT_FAR if avoid_direction == "left" else RIGHT_FAR
                if not any(boxes_intersect(car_box, b[2]) for b in boxes):
                    avoidance_mode = False
                    state = "normal"

        # ==== NORMAL FOLLOW ====
        if not avoidance_mode and state == "normal":
            motor_speed = 30
            set_motor_speed(MOTOR_FWD, MOTOR_REV, motor_speed)
            if boxes:
                boxes.sort(key=lambda b: b[1], reverse=True)
                chosen_color, chosen_area, _ = boxes[0]
                if last_color == chosen_color:
                    frame_count += 1
                else:
                    frame_count, last_color = 0, chosen_color
                if frame_count >= COLOR_HOLD_FRAMES:
                    target_angle = compute_servo_angle(chosen_color, chosen_area)

        # ==== STATE HOLD ====
        if state == "hold":
            target_angle = CENTER_ANGLE
            if time.time() - state_start > HOLD_DURATION:
                state = "normal"

        # ==== SERVO SMOOTHING ====
        if time.time() - last_update_time > SERVO_UPDATE_DELAY:
            step = FAST_SERVO_STEP if avoidance_mode else STEP
            if abs(current_servo_angle - target_angle) > step:
                current_servo_angle += step if current_servo_angle < target_angle else -step
            else:
                current_servo_angle = target_angle
            set_servo_angle(SERVO_CHANNEL, current_servo_angle)
            last_update_time = time.time()

        cv2.imshow("Contours", img_contours)
        key = cv2.waitKey(1) & 0xFF
        if key in [27, ord("q")]:
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    set_servo_angle(SERVO_CHANNEL, 90)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
