import board
import busio
import time
import adafruit_tcs34725
from adafruit_servokit import ServoKit
import adafruit_hcsr04

turns_completed = 0
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


'''def detect_direction():
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

        time.sleep(0.1)'''

def get_distance(sensor):
    try:
        return round(sensor.distance, 2)
    except RuntimeError:
        return None

# --- Main logic ---
'''direction = detect_direction()
print("Using distance sensors with direction:", direction)'''

# --- PID Constants ---
TARGET_DISTANCE = 15.0  # cm
KP = 0.5
KI = 0.0
KD = 2.0

integral = 0
last_error = 0

while True:
    distance = get_distance(side_sensor)
    if distance is not None:
        if distance > 100:
            # Wall lost: sharp left turn until wall is found again
            print("Wall lost! Making sharp left turn...")
            kit.servo[0].angle = 45  # Sharp left
            time.sleep(0.1)
            turns_completed += 1
            print(f"Turns completed: {turns_completed}")
            if turns_completed >= 12:
                print("12 turns completed. Stopping all motors and exiting.")
                kit.servo[0].angle = 90  # Center/stop steering
                # Add code to stop drive motors here if needed
                break
            continue  # Skip PID for this cycle

        error = TARGET_DISTANCE - distance
        integral += error
        derivative = error - last_error

        # PID output
        output = KP * error + KI * integral + KD * derivative

        # Clamp servo angle between 60 and 120 degrees (adjust as needed)
        new_angle = max(60, min(120, 90 + output))
        kit.servo[0].angle = new_angle

        print(f"Distance: {distance} cm | Error: {error:.2f} | Servo: {new_angle:.1f}")
        print(f"Turns completed: {turns_completed}")
        last_error = error
    else:
        print("Sensor error, skipping this cycle.")

    #if turns_completed >= 12:
     #   print("12 turns completed. Stopping all motors and exiting.")
     #   kit.servo[0].angle = 90  # Center/stop steering
      #  # Add code to stop drive motors here if needed
      #  break

    time.sleep(0.1)
