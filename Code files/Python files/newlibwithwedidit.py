'''original working detection direction with distance sensors, using pid for wall follow'''
import board
import busio
import time
#import adafruit_tcs34725
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
from pca9685_control import set_motor_speed
from pca9685_control import set_servo_angle
import RPi.GPIO as GPIO


turns_completed = 0
direction = "undefined"
# --- Initialize I2C ---
i2c = busio.I2C(board.SCL, board.SDA)

# PCA setup
# Initialize I2C and PCA9685 once
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)

def set_servo_angle(channel, angle, min_us=500, max_us=2500, frequency=50):
    if angle < 0: angle = 0
    if angle > 180: angle = 180

    pca.frequency = frequency
    pulse_length = 1000000 / pca.frequency / 4096  # microseconds per tick
    pulse = min_us + (angle / 180.0) * (max_us - min_us)
    ticks = int(pulse / pulse_length)  # 0–4095

    # Scale 12-bit ticks (0–4095) to 16-bit duty cycle (0–65535)
    pca.channels[channel].duty_cycle = int(ticks / 4096 * 0xFFFF)



def set_motor_speed(channel_forward, channel_reverse, speed, frequency=1000):
    if speed > 100: speed = 100
    if speed < -100: speed = -100

    pca.frequency = frequency
    duty_cycle = int(abs(speed) / 100.0 * 0xFFFF)

    if speed > 0:
        pca.channels[channel_forward].duty_cycle = duty_cycle
        pca.channels[channel_reverse].duty_cycle = 0
    elif speed < 0:
        pca.channels[channel_forward].duty_cycle = 0
        pca.channels[channel_reverse].duty_cycle = duty_cycle
    else:
        pca.channels[channel_forward].duty_cycle = 0
        pca.channels[channel_reverse].duty_cycle = 0
# --- Color Sensor Setup ---
# sensor_color = adafruit_tcs34725.TCS34725(i2c)
# sensor_color.gain = 16
# sensor_color.integration_time = 2.4

# --- Servo Driver Setup ---

MOTOR_FWD = 11   # forward channel
MOTOR_REV = 7   # reverse channel

# --- Color Classification ---
def classify_color(r, g, b):
    if b < 150 and b > r + 50 and b > g + 50:
        return "Blue"
    if 60 < r < 75 and 55 < g < 75 and 80 < b < 95:
        return "Orange"
    return "Unknown"

# --- PID Constants ---
TARGET_DISTANCE = 20.0  # cm
KP = 0.8
KI = 0.0
KD = 2.3

integral = 0
last_error = 0
SERVO_CHANNEL = 0

GPIO.setmode(GPIO.BCM)
TRIG_1 = 22
ECHO_1 = 23
TRIG_2 = 5
ECHO_2 = 6
TRIG_3 = 27
ECHO_3 = 17

# Set up Trigger pins as output and Echo pins as input
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIG_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)


# Ensure all Triggers are low initially
GPIO.output(TRIG_1, GPIO.LOW)
GPIO.output(TRIG_2, GPIO.LOW)
GPIO.output(TRIG_3, GPIO.LOW) 	

MOTOR_FWD = 1
MOTOR_REV = 0

# Motor control functions
def slow_start():
    print("Starting motor slowly")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 30)

def rotate_motor_forward(): 
    print("Rotating motor forward")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 80)

def rotate_motor_backward():
    print("Rotating motor backward")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, -50)

def stop_motor():
    print("Stopping motor")
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)


# def detect_direction():
#     print("Detecting color for initial direction...")
#     while True:
#         r, g, b, c = sensor_color.color_raw
#         color = classify_color(r, g, b)
#         print(f"Raw RGB: R={r}, G={g}, B={b} → Detected Color: {color}")
# 
#         if color == "Blue":
#             direction == "left"
#             print("Initial direction set: LEFT")
#             return "left"
#         if color == "Orange":
#             direction == "right"
#             print("Initial direction set: RIGHT")
#             return "right"
#         else:
#             slow_start()  # Keep moving forward until color is detected
# 
#         time.sleep(0.1)


def get_distance(trigger_pin, echo_pin):
    # Send a pulse to the trigger pin
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.007)  # Pulse duration (10 microseconds)
    GPIO.output(trigger_pin, GPIO.LOW)

    # Wait for the Echo pin to go HIGH and record the start time
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()

    # Wait for the Echo pin to go LOW and record the end time
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the time difference
    pulse_duration = pulse_end - pulse_start

    # Calculate the distance (speed of sound is ~34300 cm/s)
    distance = pulse_duration * 34300 / 2  # Divide by 2 for the round trip

    return distance

#move forward slowly, get and print colour value to determine direction
#direction = detect_direction()
#print("Using distance sensors with direction:", direction)

front = get_distance(TRIG_1, ECHO_1)
left_sensor = get_distance(TRIG_2, ECHO_2)
right_sensor = get_distance(TRIG_3, ECHO_3)
direction = "left" 

if direction == "left":
    side_sensor = left_sensor
elif direction == "right":
    side_sensor = right_sensor
else:
    print("Direction not set correctly. Exiting.")
    exit()

while True:
    distance = get_distance(TRIG_3, ECHO_3)
    distance_right = get_distance(TRIG_2,ECHO_2)
    distance_front = get_distance(TRIG_1,ECHO_1)

    rotate_motor_forward()
    if distance is not None:
        if distance > 100 and distance_front < 120:
            if direction == "left":
                print("Wall lost! Making sharp left turn...")
                set_servo_angle(SERVO_CHANNEL, 40)   # Sharp left
                if distance_right>100:
                    set_servo_angle(SERVO_CHANNEL, 110) 
            else:  # direction == "right"
                print("Wall lost! Making sharp right turn...")
                set_servo_angle(SERVO_CHANNEL, 135) 
            time.sleep(0.1)
            turns_completed += 1
            print(f"Turns completed: {turns_completed}")
            continue  # Skip PID for this cycle

        error = TARGET_DISTANCE - distance
        integral += error
        derivative = error - last_error

        # PID output
        output = KP * error + KI * integral + KD * derivative

        # Clamp servo angle between 60 and 120 degrees (adjust as needed)
        if direction == "left":
            new_angle = max(60, min(120, 90 + output))
        else:  # direction == "right"
            new_angle = max(60, min(120, 90 - output))  # Mirror for right wall
        set_servo_angle(SERVO_CHANNEL, new_angle) 

        print(f"Distance: {distance} cm | Error: {error:.2f} | Servo: {new_angle:.1f}")
        print(f"Turns completed: {turns_completed}")
        last_error = error
    else:
        print("Sensor error, skipping this cycle.")

    time.sleep(0.1)
