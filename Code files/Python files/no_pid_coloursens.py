import board
import busio
import time
import adafruit_tcs34725
from adafruit_servokit import ServoKit
import adafruit_hcsr04

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
front_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D17, echo_pin=board.D18)
side_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D22, echo_pin=board.D23)

# --- Color Classification ---
def classify_color(r, g, b):
    if b < 150 and b > r + 50 and b > g + 50:
        return "Blue"
    if 60 < r < 75 and 55 < g < 75 and 80 < b < 95:
        return "Orange"
    return "Unknown"

# --- Detect initial direction ---
def detect_direction():
    print("Detecting color for initial direction...")
    while True:
        r, g, b, c = sensor_color.color_raw
        color = classify_color(r, g, b)
        print(f"Raw RGB: R={r}, G={g}, B={b} → Detected Color: {color}")

        if color == "Blue":
            print("Initial direction set: LEFT")
            return "left"
        elif color == "Orange":
            print("Initial direction set: RIGHT")
            return "right"

        time.sleep(0.1)

# --- Get distance from sensor with error handling ---
def get_distance(sensor):
    try:
        return round(sensor.distance, 2)
    except RuntimeError:
        return None

# --- Main logic ---
direction = detect_direction()
print("Using distance sensors with direction:", direction)

# --- Setup: Track current servo angle to avoid redundant commands ---
current_angle = 90
kit.servo[0].angle = current_angle

# --- Main Loop ---
try:
    while True:
        front_dist = get_distance(front_sensor)
        side_dist = get_distance(side_sensor)

        print(f"[{time.strftime('%H:%M:%S')}] Front: {front_dist} cm, Side: {side_dist} cm")

        desired_angle = 90  # default: center

        if front_dist is not None and front_dist < 80:
            # Obstacle ahead → full turn
            if direction == "left":
                print("Obstacle detected! Turning HARD LEFT")
                desired_angle = 25
            else:
                print("Obstacle detected! Turning HARD RIGHT")
                desired_angle = 160  # Adjusted to match typical servo range (0-180)

        elif side_dist is not None:
            # Side-based course correction
            if direction == "left":
                if side_dist > 15:
                    print(f"Side distance {side_dist} cm > 15 → slight LEFT")
                    desired_angle = 70
                else:
                    print("Left-side OK → Centering")
                    desired_angle = 100
            else:  # direction == "right"
                if side_dist < 50:
                    print(f"Side distance {side_dist} cm < 50 → slight RIGHT")
                    desired_angle = 110
                else:
                    print("Right-side OK → Centering")
                    desired_angle = 80
        else:
            print("Side sensor reading failed → Centering")
            desired_angle = 90

        # --- Update servo only if angle changes ---
        if desired_angle != current_angle:
            print(f"Setting servo angle: {desired_angle}")
            kit.servo[0].angle = desired_angle
            current_angle = desired_angle

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")
    kit.servo[0].angle = 90  # Always center on exit

