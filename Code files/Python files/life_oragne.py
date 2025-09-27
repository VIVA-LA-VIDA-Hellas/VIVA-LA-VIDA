'''???'''
import board
import busio
import time
import digitalio
import adafruit_tcs34725
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

# Ultrasonic Sensor Setup
TRIG = 17
ECHO = 18

GPIO.setmode(GPIO.BCM) 
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Color Sensor Setup
i2c = busio.I2C(board.SCL, board.SDA)
sensor_color = adafruit_tcs34725.TCS34725(i2c)
sensor_color.gain = 16
sensor_color.integration_time = 2.4

# Servo Driver Setup
kit = ServoKit(channels=8, i2c=i2c, address=0x40)

# Classify Color from Raw RGB
def classify_color(r, g, b):
    if b > 150 and b > r + 50 and b > g + 50:
        return "Blue"
    if r > 200 and 50 < g < 180 and b < 100:
        return "Orange"
    return "Unknown"

# Measure Distance Function
def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance + 1.15, 2)

# Detect Color Once
print("Detecting color...")
while True:
    r, g, b, c = sensor_color.color_raw
    color = classify_color(r, g, b)
    print(f"Raw RGB: R={r}, G={g}, B={b}")
    print(f"Detected Color: {color}")

    if color in ["Orange", "Blue"]:
        break

    time.sleep(1)

# Act based on Distance and Detected Color
print(f"Detected Color: {color}")
print("Waiting for object within 80cm...")

try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance} cm")

        if distance < 80:
            if color == "Orange":
                print("Object close! Turning RIGHT")
                kit.servo[0].angle = 145
            elif color == "Blue":
                print("Object close! Turning LEFT")
                kit.servo[0].angle = 45
            time.sleep(1)
            print("Returning servo to center")
            kit.servo[0].angle = 90
            time.sleep(1)

        time.sleep(0.3)

except KeyboardInterrupt:
    print("Stopped by user")
    GPIO.cleanup()
