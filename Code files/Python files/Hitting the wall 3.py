'''no pid, no servo coms, detect direction with camera (not working, unfinished) -updated version, has more targeted readings'''
import board
import busio
import digitalio
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

def empty(x):
    pass

# Pin setup
TRIG = 22
ECHO = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Camera and servo setup
picam2 = Picamera2()
picam2.start()
time.sleep(2)

i2c = busio.I2C(board.SCL, board.SDA)
kit = ServoKit(channels=16, i2c=i2c, address=0x40)

direction = "null"

# --- 1️⃣ ONE-TIME Blue/Orange Detection ---
img = picam2.capture_array()
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# Ignore the bottom third of the image
height, width = imgHSV.shape[:2]
bottom_third = height * 2 // 3
imgHSV[bottom_third:, :] = 0


# Define color ranges
orange_lower = np.array([5, 140, 100])
orange_upper = np.array([15, 235, 255])

blue_lower = np.array([80, 80, 100])
blue_upper = np.array([170, 135, 255])


# Create masks
mask_orange = cv2.inRange(imgHSV, orange_lower, orange_upper)
edges_orange = cv2.Canny(mask_orange, 100, 100)

mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper) 
edges_blue = cv2.Canny(mask_blue, 100, 100)

# Find contours
contours_orange, _ = cv2.findContours(edges_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_blue, _ = cv2.findContours(edges_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw rectangles
img_contours = img.copy()
for contour in contours_orange:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 165, 255), 2)

for contour in contours_blue:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(img_contours, (x, y), (x + w, y + h), (255, 0, 0), 2)

# Determine middle zone
height, width = img.shape[:2]
middle_start = int(width * 0.4)
middle_end = int(width * 0.6)

def filter_middle_contours(contours, middle_start, middle_end):
    return [
        c for c in contours
        if middle_start <= (cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2] // 2) <= middle_end
    ]

middle_orange = filter_middle_contours(contours_orange, middle_start, middle_end)
middle_blue = filter_middle_contours(contours_blue, middle_start, middle_end)

lowest_orange = max([cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in middle_orange], default=0)
lowest_blue = max([cv2.boundingRect(c)[1] + cv2.boundingRect(c)[3] for c in middle_blue], default=0)

if lowest_orange > lowest_blue:
    print("Orange first")
    direction = "Right"
elif lowest_blue > lowest_orange:
    print("Blue first")
    direction = "Left"
else:
    print("No colours detected")
    direction = "null"

print("Direction:", direction)

# --- 2️⃣ MAIN LOOP ---
while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Lane detection masks
    black_lower = np.array([0, 0, 0])
    black_upper = np.array([255, 255, 40])
    white_lower = np.array([0, 0, 40])
    white_upper = np.array([255, 100, 255])
    mask_black = cv2.inRange(imgHSV, black_lower, black_upper)
    mask_white = cv2.inRange(imgHSV, white_lower, white_upper)

    edges_black = cv2.Canny(mask_black, 100, 100)
    height, width = edges_black.shape
    left_edges = []
    right_edges = []

    # --- Ultrasonic Distance Measurement ---
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()

    try:
        pulse_duration = pulse_end - pulse_start
        distance = round(pulse_duration * 17150 + 1.15, 2)
    except:
        distance = 999  # Timeout fallback

    if distance <= 100:
        print("Distance:", distance, "cm — Hitting the wall")
        kit.servo[0].angle = 45
    else:
        print("Distance:", distance, "cm — Moving forward")

    # --- Lane Tracking ---
    for i in range(height):
        row = edges_black[i, :]

        left = np.where(row[:width // 2] > 0)[0]
        right = np.where(row[width // 2:] > 0)[0]

        left_edges.append(left[-1] if len(left) > 0 else None)
        right_edges.append(right[0] + width // 2 if len(right) > 0 else None)

    # Draw path line
    for i in range(height):
        left_avg = left_edges[i]
        right_avg = right_edges[i]

        if left_avg is not None and right_avg is not None:
            middle_line = (left_avg + right_avg) // 2
            cv2.line(edges_black, (middle_line, i), (middle_line, i), (255, 255, 255), 3)

    # --- Show Windows ---
    cv2.imshow("Input", img)
    cv2.imshow("Contours (one-time)", img_contours)
    cv2.imshow("Path on Edges", edges_black)

    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
        break

# Cleanup
picam2.stop()
GPIO.cleanup()
cv2.destroyAllWindows()
