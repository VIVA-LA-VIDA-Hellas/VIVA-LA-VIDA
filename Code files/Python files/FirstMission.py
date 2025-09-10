import board
import busio
import time
import RPi.GPIO as GPIO
from gpiozero import Button

# Import motor + servo functions from your module
from pca9685_control import set_motor_speed, set_servo_angle

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
TRIG_1 = 22
ECHO_1 = 23
TRIG_2 = 5
ECHO_2 = 6
TRIG_3 = 27
ECHO_3 = 17

GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIG_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)

# === Motor Pins ===
MOTOR_FWD = 1
MOTOR_REV = 0

# === Servo Channel ===
STEERING_SERVO = 0
set_servo_angle(STEERING_SERVO, 90)  # neutral steering

# === Speed Settings ===
BASE_SPEED = 80
TURN_SPEED = 85
MIN_TURN_DURATION = 0.5

# === PID Constants ===
TARGET_DISTANCE = 20.0
KP = 0.5
KI = 0.0
KD = 0.5

integral = 0
last_error = 0

# === Button Setup ===
button = Button(21)
program_started = False
stop_requested = False

# === Motor Functions ===
def slow_start():
    print("Starting motor slowly")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 35)

def rotate_motor_forward(speed): 
    print(f"Rotating motor forward at {speed}%")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, speed)

def rotate_motor_backward(speed):
    print(f"Rotating motor backward at {speed}%")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -abs(speed))

def stop_motor():
    print("Stopping motor")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)

# === Ultrasonic Functions ===
def get_distance(trigger_pin, echo_pin, timeout=0.1):
    GPIO.output(trigger_pin, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, GPIO.LOW)

    start_time = time.monotonic()
    while GPIO.input(echo_pin) == 0:
        if time.monotonic() - start_time > timeout:
            return None
    pulse_start = time.monotonic()

    while GPIO.input(echo_pin) == 1:
        if time.monotonic() - pulse_start > timeout:
            return None
    pulse_end = time.monotonic()

    duration = pulse_end - pulse_start
    distance = (duration * 34300) / 2
    return round(distance, 2)

def get_distance_filtered(trigger_pin, echo_pin, samples=5):
    distances = []
    for _ in range(samples):
        d = get_distance(trigger_pin, echo_pin)
        if d is not None:
            distances.append(d)
        time.sleep(0.01)
    if distances:
        distances.sort()
        return distances[len(distances)//2]
    return None

# === Direction Detection ===
def detect_direction():
    print("Detecting wall for initial direction...")
    while True:
        right = get_distance_filtered(TRIG_2, ECHO_2)
        left = get_distance_filtered(TRIG_3, ECHO_3)
        if left and left > 120:
            print("Initial direction: LEFT")
            return "left"
        if right and right > 120:
            print("Initial direction: RIGHT")
            return "right"
        rotate_motor_forward(70)
        if stop_requested:
            stop_motor()
            return None

# === Button Handling ===
def toggle_program():
    global program_started, stop_requested
    if not program_started:
        print("Button pressed. Starting robot...")
        program_started = True
    else:
        print("Button pressed again. Stopping...")
        stop_requested = True
        stop_motor()
        GPIO.cleanup()
        exit(0)

button.when_pressed = toggle_program

# === Main Wait ===
print("Waiting for button press to start...")
while not program_started:
    time.sleep(0.1)

# === Main Program ===
direction = detect_direction()
if direction is None:
    exit(0)

turns_completed = 0
last_turn_time = time.monotonic()
LOOP_DELAY = 0.05

while True:
    if stop_requested:
        stop_motor()
        GPIO.cleanup()
        break

    start_time = time.monotonic()
    front = get_distance_filtered(TRIG_1, ECHO_1)
    right = get_distance_filtered(TRIG_2, ECHO_2)
    left = get_distance_filtered(TRIG_3, ECHO_3)

    if direction == "left":
        distance = left
        other = right
    else:
        distance = right
        other = left

    if distance is None or other is None or front is None:
        continue

    # Sharp turn
    if distance > 100 and other < 100 and front < 100:
        if direction == "left":
            set_servo_angle(STEERING_SERVO, 130)
        else:
            set_servo_angle(STEERING_SERVO, 50)
        rotate_motor_forward(TURN_SPEED)
        last_turn_time = time.monotonic()
        turns_completed += 1

    # Recover from oversteering
    elif time.monotonic() - last_turn_time < MIN_TURN_DURATION:
        if direction == "left":
            set_servo_angle(STEERING_SERVO, 70)
        else:
            set_servo_angle(STEERING_SERVO, 110)
        rotate_motor_forward(TURN_SPEED)

    else:
        # PID control
        error = TARGET_DISTANCE - distance
        integral += error
        derivative = error - last_error
        last_error = error

        output = KP * error + KI * integral + KD * derivative

        if direction == "left":
            angle = max(50, min(130, 90 + output))
        else:
            angle = max(50, min(130, 90 - output))

        set_servo_angle(STEERING_SERVO, angle)
        rotate_motor_forward(BASE_SPEED)

    print(f"Front: {front}, L: {left}, R: {right}, Angle: {angle:.1f}, Turns: {turns_completed}")

    elapsed = time.monotonic() - start_time
    if elapsed < LOOP_DELAY:
        time.sleep(LOOP_DELAY - elapsed)

