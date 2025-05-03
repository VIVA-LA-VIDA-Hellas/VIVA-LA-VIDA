import time
from adafruit_servokit import ServoKit
import board
import busio
import digitalio
import adafruit_vl53l0x
from adafruit_as7341 import AS7341

# ---------------- PID Controller ------------------
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

# ---------------- Sensor & Hardware Setup ------------------
i2c = busio.I2C(board.SCL, board.SDA)

# Setup TOF sensor shutdown pins
xshut_left = digitalio.DigitalInOut(board.D17)
xshut_right = digitalio.DigitalInOut(board.D27)
xshut_left.direction = digitalio.Direction.OUTPUT
xshut_right.direction = digitalio.Direction.OUTPUT

xshut_left.value = False
xshut_right.value = False
time.sleep(0.1)

xshut_left.value = True
time.sleep(0.1)
sensor_left = adafruit_vl53l0x.VL53L0X(i2c)
sensor_left.set_address(0x30)

xshut_right.value = True
time.sleep(0.1)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c)
sensor_right.set_address(0x31)

sensor_left = adafruit_vl53l0x.VL53L0X(i2c, address=0x30)
sensor_right = adafruit_vl53l0x.VL53L0X(i2c, address=0x31)

# Color sensor
color_sensor = AS7341(i2c)
if color_sensor:
    print("Color sensor initialized successfully.")
else:
    print("Color sensor initialization failed!")

color_sensor.led_current = 0
color_sensor.led = True

# Motor controller
kit = ServoKit(channels=8, i2c=i2c, address=0x40)

# Bar graph for display
def bar_graph(read_value):
    scaled = int(read_value / 1000)
    return "[%5d] " % read_value + (scaled * "*")

# PID controller setup
pid = PIDController(Kp=2.0, Ki=0.0, Kd=1.0)

# ---------------- MAIN LOOP ------------------
try:
    time.sleep(1)

    left = sensor_left.range / 10
    right = sensor_right.range / 10

    reference_side = "left" if left > right else "right"
    waiting_for = "right" if reference_side == "left" else "left"
    print(f"Reference: {reference_side}, waiting for '{waiting_for}' to reach 5 cm.")

    at_target_position = False
    locked_color = None
    total_detections = 0
    blue_count = 0
    orange_count = 0

    blue_threshold = 900
    orange_threshold = 900
    color_margin = 100

    while total_detections < 12:
        left_distance = sensor_left.range / 10
        right_distance = sensor_right.range / 10
        distance_diff = left_distance - right_distance

        blue = color_sensor.channel_480nm or 0
        orange = color_sensor.channel_630nm or 0

        print(f"Color Sensor Readings → 480nm (Blue): {blue} | 630nm (Orange): {orange}")
        print("F3 - 480nm//Blue   %s" % bar_graph(blue))
        print("F7 - 630nm/Orange  %s" % bar_graph(orange))

        turn_decision = None
        color_detected = None

        print("\n--- Iteration Report --------------------------------------")
        print(f"TOF Distances → Left: {left_distance:.1f} cm | Right: {right_distance:.1f} cm")
        print(f"Distance Difference: {abs(distance_diff):.1f} cm")

        if blue > orange and blue > blue_threshold and (blue - orange > color_margin):
            if locked_color != "blue":
                locked_color = "blue"
                print("🔵 Color Lock Updated → BLUE")
            color_detected = "blue"
            print("Action: Blue detected! Turning RIGHT (PID adjusted).")
            angle = 130
            total_detections += 1
            blue_count += 1

        elif orange > blue and orange > orange_threshold and (orange - blue > color_margin):
            if locked_color != "orange":
                locked_color = "orange"
                print("Color Lock Updated → ORANGE")
            color_detected = "orange"
            print("Action: Orange detected! Turning LEFT (PID adjusted).")
            angle = 70
            total_detections += 1
            orange_count += 1

        else:
            pid_output = pid.compute(distance_diff)
            center_angle = 100
            angle = max(70, min(130, center_angle + pid_output))
            print(f"No color lock. Using PID → Output: {pid_output:.2f} → Servo Angle: {angle:.2f}")

        # Apply servo angle and move forward briefly
        kit.servo[0].angle = angle
        kit.continuous_servo[1].throttle = 0.5
        time.sleep(0.5)
        kit.continuous_servo[1].throttle = 0

        if not at_target_position:
            print(f"Target Check → Waiting for '{waiting_for}' to reach ≤ 5 cm.")
            if (waiting_for == "left" and left_distance <= 5) or (waiting_for == "right" and right_distance <= 5):
                print("Target position reached!")
                kit.continuous_servo[1].throttle = 0
                at_target_position = True
            else:
                print("Moving forward toward target...")
        else:
            print("Holding position. Monitoring for color...")

        print(f"Servo Angle: {kit.servo[0].angle if kit.servo[0].angle else 'N/A'}")
        print(f"Motor Throttle: {kit.continuous_servo[1].throttle}")
        print(f"Status → Blue: {blue_count}, Orange: {orange_count}, Total: {total_detections}")
        print("--------------------------------------------------------------")
        time.sleep(1)

    print("12 detections complete. Shutting down.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    kit.continuous_servo[1].throttle = 0
    print("Motors off. Program ended.")

