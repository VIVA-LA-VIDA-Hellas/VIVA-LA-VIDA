import board
import busio
import time
import adafruit_tcs34725
from adafruit_servokit import ServoKit
from adafruit_motorkit import MotorKit
import adafruit_hcsr04

turns_completed = 0
direction = "undefined"
# --- Initialize I2C ---
i2c = busio.I2C(board.SCL, board.SDA)

# --- Color Sensor Setup ---
sensor_color = adafruit_tcs34725.TCS34725(i2c)
sensor_color.gain = 16
sensor_color.integration_time = 2.4

# --- Servo Driver Setup ---
kit = ServoKit(channels=8, i2c=i2c, address=0x40)
kit.servo[0].angle = 90

# --- Ultrasonic Sensors Setup ---
front_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D17, echo_pin=board.D27)
left_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)
right_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D22, echo_pin=board.D23)

# --- Color Classification ---
def classify_color(r, g, b):
    if b < 150 and b > r + 50 and b > g + 50:
        return "Blue"
    if 60 < r < 75 and 55 < g < 75 and 80 < b < 95:
        return "Orange"
    return "Unknown"

# --- PID Constants ---
TARGET_DISTANCE = 20.0  # cm
KP = 1.0
KI = 0.0
KD = 2.0

integral = 0
last_error = 0

kit_motor = MotorKit()

# Motor control functions
def slow_start():
    print("Starting motor slowly")
    kit_motor.motor3.throttle = 0.7  # Slow start

def rotate_motor_forward(): 
    print("Rotating motor forward")
    kit_motor.motor3.throttle = 0.9#speed forward

def rotate_motor_backward():
    print("Rotating motor backward")
    kit_motor.motor3.throttle = -0.9#speed backward

def stop_motor():
    print("Stopping motor")
    kit_motor.motor3.throttle = 0.0  # Stop the motor


def detect_direction():
    print("Detecting color for initial direction...")
    while True:
        r, g, b, c = sensor_color.color_raw
        color = classify_color(r, g, b)
        print(f"Raw RGB: R={r}, G={g}, B={b} → Detected Color: {color}")

        if color == "Blue":
            direction == "left"
            print("Initial direction set: LEFT")
            return "left"
        if color == "Orange":
            direction == "right"
            print("Initial direction set: RIGHT")
            return "right"
        else:
            slow_start()  # Keep moving forward until color is detected

        time.sleep(0.1)
    
def get_distance(sensor):
    try:
        return round(sensor.distance, 2)
    except RuntimeError:
        return None

#move forward slowly, get and print colour value to determine direction
direction = detect_direction()
print("Using distance sensors with direction:", direction)

if direction == "left":
    side_sensor = left_sensor
elif direction == "right":
    side_sensor = right_sensor
else:
    print("Direction not set correctly. Exiting.")
    exit()

while True:
    distance = get_distance(side_sensor)
    rotate_motor_forward()
    if distance is not None:
        if distance > 100:
            if direction == "left":
                print("Wall lost! Making sharp left turn...")
                kit.servo[0].angle = 45  # Sharp left
            else:  # direction == "right"
                print("Wall lost! Making sharp right turn...")
                kit.servo[0].angle = 135  # Sharp right (mirror)
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
        kit.servo[0].angle = new_angle

        print(f"Distance: {distance} cm | Error: {error:.2f} | Servo: {new_angle:.1f}")
        print(f"Turns completed: {turns_completed}")
        last_error = error
    else:
        print("Sensor error, skipping this cycle.")

    time.sleep(0.1)
