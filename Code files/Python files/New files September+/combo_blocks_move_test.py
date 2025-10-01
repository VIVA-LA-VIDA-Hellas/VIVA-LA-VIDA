#!/usr/bin/env python3
"""
Combined Program:
Wall-following robot (Ultrasonic + PCA9685) + Color detection (PiCamera2 + OpenCV).
- Base: Wall-following using ultrasonic sensors.
- Override: Red object = turn right, Green object = turn left.
"""

import time
import board
import busio
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685

# ==================== PCA9685 / Motor / Servo Config ====================
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50 Hz for servos

SERVO_CHANNEL = 0
MOTOR_FWD = 1
MOTOR_REV = 2

SERVO_MIN_ANGLE = 50
SERVO_MAX_ANGLE = 130
SERVO_CENTER = 90

PULSE_MIN_US = 1000
PULSE_MAX_US = 2000
SERVO_PERIOD_US = 20000

def speed_to_duty(speed_percent: float) -> int:
    s = max(0.0, min(100.0, speed_percent))
    return int(s / 100.0 * 65535)

def rotate_motor_forward(speed_percent: float = 85.0):
    duty = speed_to_duty(speed_percent)
    pca.channels[MOTOR_FWD].duty_cycle = duty
    pca.channels[MOTOR_REV].duty_cycle = 0

def stop_motor():
    pca.channels[MOTOR_FWD].duty_cycle = 0
    pca.channels[MOTOR_REV].duty_cycle = 0

def set_servo_angle(channel: int, angle_deg: float):
    a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle_deg))
    pulse_us = int(PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) *
                   ((a - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)))
    duty = int(pulse_us * 65535 / SERVO_PERIOD_US)
    pca.channels[channel].duty_cycle = duty

set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)

# ==================== Ultrasonic Sensor Setup ====================
TRIG_FRONT, ECHO_FRONT = 22, 23
TRIG_LEFT, ECHO_LEFT   = 27, 17
TRIG_RIGHT, ECHO_RIGHT = 5, 6

GPIO.setmode(GPIO.BCM)
for trig in (TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT):
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, GPIO.LOW)
for echo in (ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT):
    GPIO.setup(echo, GPIO.IN)

time.sleep(0.1)

TRIG_PULSE = 0.00001
ECHO_TIMEOUT = 0.02

def get_distance(trigger_pin: int, echo_pin: int):
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(TRIG_PULSE)
    GPIO.output(trigger_pin, GPIO.LOW)

    start_wait = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        if time.time() - start_wait > ECHO_TIMEOUT:
            return None
    pulse_start = time.time()

    start_wait = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        if time.time() - start_wait > ECHO_TIMEOUT:
            return None
    pulse_end = time.time()

    return (pulse_end - pulse_start) * 17150  # cm

# ==================== PID Config ====================
TARGET_DISTANCE = 18.0
KP, KI, KD = 0.8, 0.0, 2.0
integral, last_error = 0.0, 0.0

# ==================== Camera Setup ====================
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

def get_largest_contour_corners(contours):
    if len(contours) == 0:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    return ( (x, y), (x + w, y), (x, y + h), (x + w, y + h), largest_contour )

# ==================== Main Loop ====================
try:
    while True:
        # --- Camera capture ---
        image = picam2.capture_array()
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Masks for red and green
        red_lower1, red_upper1 = np.array([0, 150, 120]), np.array([20, 255, 255])
        mask_red = cv2.inRange(imgHSV, red_lower1, red_upper1)

        green_lower, green_upper = np.array([60, 70, 80]), np.array([80, 210, 140])
        mask_green = cv2.inRange(imgHSV, green_lower, green_upper)

        # Contours
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        red_data = get_largest_contour_corners(contours_red)
        green_data = get_largest_contour_corners(contours_green)

        red_area = cv2.contourArea(red_data[-1]) if red_data else 0
        green_area = cv2.contourArea(green_data[-1]) if green_data else 0

        # --- Ultrasonic distances ---
        dist_front = get_distance(TRIG_FRONT, ECHO_FRONT)
        dist_left = get_distance(TRIG_LEFT, ECHO_LEFT)
        dist_right = get_distance(TRIG_RIGHT, ECHO_RIGHT)

        # --- Color detection overrides ---
        if green_area > red_area and green_area > 200:
            print("Green detected -> FORCE LEFT TURN")
            set_servo_angle(SERVO_CHANNEL, SERVO_MIN_ANGLE)
            rotate_motor_forward(60.0)
            time.sleep(0.2)
            continue
        elif red_area > green_area and red_area > 200:
            print("Red detected -> FORCE RIGHT TURN")
            set_servo_angle(SERVO_CHANNEL, SERVO_MAX_ANGLE)
            rotate_motor_forward(60.0)
            time.sleep(0.2)
            continue

        # --- Normal wall following ---
        if dist_left and dist_right:
            side_dist = dist_left  # follow left wall
            error = TARGET_DISTANCE - side_dist
            integral += error
            derivative = error - last_error
            output = KP * error + KD * derivative
            new_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, SERVO_CENTER + output))
            set_servo_angle(SERVO_CHANNEL, new_angle)
            rotate_motor_forward(80.0)
            print(f"Wall follow | Left: {dist_left:.1f} | Right: {dist_right:.1f} | Servo: {new_angle:.1f}")
            last_error = error
        else:
            print("No valid side distance - going straight")
            set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)
            rotate_motor_forward(70.0)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping robot...")
finally:
    stop_motor()
    set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)
    pca.deinit()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
