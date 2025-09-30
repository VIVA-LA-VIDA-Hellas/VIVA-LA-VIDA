#!/usr/bin/env python3
"""
Wall-following robot using ultrasonic sensors + PCA9685 for servo + motor control.
Simplified version without any color-based detection.
"""

import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685

# --- Configuration / Constants ---
# I2C / PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # 50 Hz for hobby servos

SERVO_CHANNEL = 0  # PCA9685 channel used for steering servo
MOTOR_FWD = 1      # PCA9685 channel used for forward motor
MOTOR_REV = 2      # PCA9685 channel used for reverse motor

# Ultrasonic pins (BCM)
TRIG_FRONT, ECHO_FRONT = 22,23  # front sensor
TRIG_LEFT, ECHO_LEFT   = 27, 17   # left sensor
TRIG_RIGHT, ECHO_RIGHT = 5,  6    # right sensor

# PID constants
TARGET_DISTANCE = 18.0  # cm target from wall
KP = 0.8
KI = 0.0
KD = 2.0

# Servo angle limits
SERVO_MIN_ANGLE = 50
SERVO_MAX_ANGLE = 130
SERVO_CENTER = 90

# Pulse mapping for PCA9685
PULSE_MIN_US = 1000   # 1ms
PULSE_MAX_US = 2000   # 2ms
SERVO_PERIOD_US = 20000  # 20ms (50Hz)

# Motor speed mapping
def speed_to_duty(speed_percent: float) -> int:
    s = max(0.0, min(100.0, speed_percent))
    return int(s / 100.0 * 65535)

TRIG_PULSE = 0.00001  # 10 µs
ECHO_TIMEOUT = 0.02   # 20 ms

# --- Global state ---
turns_completed = 0
direction = "undefined"
integral = 0.0
last_error = 0.0

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
for trig in (TRIG_FRONT, TRIG_LEFT, TRIG_RIGHT):
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, GPIO.LOW)
for echo in (ECHO_FRONT, ECHO_LEFT, ECHO_RIGHT):
    GPIO.setup(echo, GPIO.IN)

time.sleep(0.1)

# --- Motor control ---
def rotate_motor_forward(speed_percent: float = 85.0):
    duty = speed_to_duty(speed_percent)
    pca.channels[MOTOR_FWD].duty_cycle = duty
    pca.channels[MOTOR_REV].duty_cycle = 0

def rotate_motor_backward(speed_percent: float = 90.0):
    duty = speed_to_duty(speed_percent)
    pca.channels[MOTOR_FWD].duty_cycle = 0
    pca.channels[MOTOR_REV].duty_cycle = duty

def stop_motor():
    pca.channels[MOTOR_FWD].duty_cycle = 0
    pca.channels[MOTOR_REV].duty_cycle = 0

# --- Servo control ---
def set_servo_angle(channel: int, angle_deg: float):
    a = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle_deg))
    pulse_us = int(PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) *
                   ((a - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)))
    duty = int(pulse_us * 65535 / SERVO_PERIOD_US)
    pca.channels[channel].duty_cycle = duty

set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)

# --- Distance measurement ---
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

    pulse_duration = pulse_end - pulse_start
    return (pulse_duration * 34300) / 2  # cm

# --- Initial direction detection ---
def detect_direction(timeout_seconds: float = 5.0) -> str:
    global direction
    print("Detecting initial direction...")
    start_time = time.time()
    rotate_motor_forward(35.0)

    try:
        while True:
            if time.time() - start_time > timeout_seconds:
                print("Timeout — defaulting to LEFT.")
                direction = "left"
                return direction

            left_d = get_distance(TRIG_LEFT, ECHO_LEFT)
            right_d = get_distance(TRIG_RIGHT, ECHO_RIGHT)

            if left_d is not None and left_d > 150:
                direction = "left"
                print("Initial direction: LEFT")
                return "left"
            if right_d is not None and right_d > 150:
                direction = "right"
                print("Initial direction: RIGHT")
                return "right"

            time.sleep(0.08)
    finally:
        stop_motor()

# --- Main loop ---
def main_loop():
    global integral, last_error, turns_completed, direction

    direction = detect_direction()
    print("Following wall on:", direction)

    try:
        while True:
            dist_front = get_distance(TRIG_FRONT, ECHO_FRONT)
            dist_left = get_distance(TRIG_LEFT, ECHO_LEFT)
            dist_right = get_distance(TRIG_RIGHT, ECHO_RIGHT)

            if direction == "left":
                side_dist = dist_left
                other_side = dist_right
            elif direction == "right":
                side_dist = dist_right
                other_side = dist_left
            else:
                print("Direction undefined — exiting.")
                break

            rotate_motor_forward(85.0)

            if side_dist is None:
                print("Side sensor error. Skipping cycle.")
                time.sleep(0.1)
                continue

            if side_dist > 90 and dist_front is not None and dist_front < 100:
                if direction == "left":
                    print("Wall lost! Sharp LEFT turn.")
                    set_servo_angle(SERVO_CHANNEL, 40)
                    if other_side is not None and other_side > 100:
                        set_servo_angle(SERVO_CHANNEL, 120)
                else:
                    print("Wall lost! Sharp RIGHT turn.")
                    set_servo_angle(SERVO_CHANNEL, 135)
                    if other_side is not None and other_side > 100:
                        set_servo_angle(SERVO_CHANNEL, 60)

                turns_completed += 1
                print(f"Turns completed: {turns_completed}")
                time.sleep(0.12)
                continue

            error = TARGET_DISTANCE - side_dist
            integral += error
            derivative = error - last_error
            output = KP * error + KI * integral + KD * derivative

            if direction == "left":
                new_angle = SERVO_CENTER + output
            else:
                new_angle = SERVO_CENTER - output

            new_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, new_angle))
            set_servo_angle(SERVO_CHANNEL, new_angle)

            print(f"Side: {side_dist:.1f} cm | Error: {error:.2f} | Servo: {new_angle:.1f} | Front: {dist_front if dist_front else 'N/A'}")
            print(f"Turns completed: {turns_completed}")

            last_error = error
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt — stopping.")
    finally:
        print("Stopping safely.")
        stop_motor()
        set_servo_angle(SERVO_CHANNEL, SERVO_CENTER)
        pca.deinit()
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()
