import cv2
import numpy as np
from picamera2 import Picamera2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import board
import busio


# ==== CONSTANTS ====
MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2  # reverse channel
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
