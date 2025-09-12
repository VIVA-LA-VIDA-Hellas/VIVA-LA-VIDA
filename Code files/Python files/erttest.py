import time
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from pca9685_control import set_motor_speed, set_servo_angle

# --- Global State ---
turns_completed = 0
direction = "left"  # default direction

# --- PCA9685 Setup ---
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Default for servos

# --- PID Constants ---
TARGET_DISTANCE = 20.0  # cm
KP = 0.8
KI = 0.0
KD = 2.3

integral = 0
last_error = 0
SERVO_CHANNEL = 0

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)

TRIG_1, ECHO_1 = 22, 23   # Front sensor
TRIG_2, ECHO_2 = 5, 6     # Left sensor
TRIG_3, ECHO_3 = 27, 17   # Right sensor

for trig in [TRIG_1, TRIG_2, TRIG_3]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, GPIO.LOW)

for echo in [ECHO_1, ECHO_2, ECHO_3]:
    GPIO.setup(echo, GPIO.IN)

# --- Motor Channels ---
MOTOR_FWD = 1
MOTOR_REV = 0

# --- Motor Control ---
def rotate_motor_forward():
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 80)

def rotate_motor_backward():
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -50)

def stop_motor():
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)

# --- Distance Measurement ---
def get_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10 µs pulse
    GPIO.output(trigger_pin, GPIO.LOW)

    pulse_start, pulse_end = None, None

    # Wait for echo to start
    timeout = time.time() + 0.02
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    # Wait for echo to end
    timeout = time.time() + 0.02
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    if pulse_start and pulse_end:
        pulse_duration = pulse_end - pulse_start
        return (pulse_duration * 34300) / 2  # cm

    return None

# --- Main Loop ---
try:
    while True:
        distance_front = get_distance(TRIG_1, ECHO_1)
        distance_left = get_distance(TRIG_2, ECHO_2)
        distance_right = get_distance(TRIG_3, ECHO_3)

        if direction == "left":
            side_sensor = distance_left
        else:
            side_sensor = distance_right

        rotate_motor_forward()

        if side_sensor is None:
            print("Sensor error, skipping cycle.")
            continue

        # Detect wall loss
        if side_sensor > 100 and distance_front and distance_front < 120:
            if direction == "left":
                print("Wall lost! Sharp LEFT turn")
                set_servo_angle(SERVO_CHANNEL, 40)
                if distance_right and distance_right > 100:
                    set_servo_angle(SERVO_CHANNEL, 110)
            else:
                print("Wall lost! Sharp RIGHT turn")
                set_servo_angle(SERVO_CHANNEL, 135)

            turns_completed += 1
            print(f"Turns completed: {turns_completed}")
            time.sleep(0.1)
            continue

        # --- PID Control ---
        error = TARGET_DISTANCE - side_sensor
        integral += error
        derivative = error - last_error
        output = KP * error + KI * integral + KD * derivative

        if direction == "left":
            new_angle = max(60, min(120, 90 + output))
        else:
            new_angle = max(60, min(120, 90 - output))  # mirrored for right wall

        set_servo_angle(SERVO_CHANNEL, new_angle)

        print(f"Side: {side_sensor:.1f} cm | Error: {error:.2f} | Servo: {new_angle:.1f}")
        print(f"Turns completed: {turns_completed}")

        last_error = error
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping safely...")
    stop_motor()
    GPIO.cleanup()
