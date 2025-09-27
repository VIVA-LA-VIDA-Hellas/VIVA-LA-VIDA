import time
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

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
    speed = 80
    duty_cycle = int(min(max(abs(speed), 0), 100) / 100 * 65535)
    pca.channels[MOTOR_FWD].duty_cycle = duty_cycle
    pca.channels[MOTOR_REV].duty_cycle = 0

def rotate_motor_backward():
    speed = -50
    duty_cycle = int(min(max(abs(speed), 0), 100) / 100 * 65535)
    pca.channels[MOTOR_FWD].duty_cycle = 0
    pca.channels[MOTOR_REV].duty_cycle = duty_cycle

def stop_motor():
    pca.channels[MOTOR_FWD].duty_cycle = 0
    pca.channels[MOTOR_REV].duty_cycle = 0

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
                # Move servo to 40 degrees
                angle = max(50, min(130, 40))
                pulse_min = 1000
                pulse_max = 2000
                pulse = int(pulse_min + (pulse_max - pulse_min) * ((angle - 50) / (130 - 50)))
                pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / 20000)
                if distance_right and distance_right > 100:
                    angle = max(50, min(130, 110))
                    pulse = int(pulse_min + (pulse_max - pulse_min) * ((angle - 50) / (130 - 50)))
                    pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / 20000)
            else:
                print("Wall lost! Sharp RIGHT turn")
                angle = max(50, min(130, 135))
                pulse_min = 1000
                pulse_max = 2000
                pulse = int(pulse_min + (pulse_max - pulse_min) * ((angle - 50) / (130 - 50)))
                pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / 20000)

            turns_completed += 1
            print(f"Turns completed: {turns_completed}")
            time.sleep(0.1)
            continue

        # --- PID Control ---
        error = TARGET_DISTANCE - side_sensor
        integral += error
        derivative = error - last_error
        output = KP * error + KI * integral + KD * derivative

        # Use servo min/max 50/130
        if direction == "left":
            new_angle = max(50, min(130, 90 + output))
        else:
            new_angle = max(50, min(130, 90 - output))  # mirrored for right wall

        # Direct servo control using adafruit_pca9685
        min_angle = 50
        max_angle = 130
        angle = max(min_angle, min(max_angle, new_angle))
        pulse_min = 1000  # 1ms
        pulse_max = 2000  # 2ms
        pulse = int(pulse_min + (pulse_max - pulse_min) * ((angle - min_angle) / (max_angle - min_angle)))
        pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / 20000)  # 20ms period

        print(f"Side: {side_sensor:.1f} cm | Error: {error:.2f} | Servo: {new_angle:.1f}")
        print(f"Turns completed: {turns_completed}")

        last_error = error
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping safely...")
    stop_motor()
    GPIO.cleanup()