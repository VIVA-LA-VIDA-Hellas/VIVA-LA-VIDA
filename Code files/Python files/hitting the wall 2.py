import board
import busio
import digitalio
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
from smbus2 import SMBus    

bus = SMBus(0)

def empty(x):
    pass

TRIG = 17
ECHO = 18
i=0

GPIO.setmode(GPIO.BCM) 
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

# Initial values for HSV trackbars
 
h_min, h_max = 3, 18
s_min, s_max = 220, 255
v_min, v_max = 0, 255

# Initialize Picamera2 for capturing frames
picam2 = Picamera2()
picam2.start()

kit = ServoKit(channels=8, i2c=i2c, address=0x40)

# Give the camera a moment to adjust settings
time.sleep(2)

while True:
    # Capture frame using picamera2q
    img = picam2.capture_array()

    # Convert the image from RGB to BGR for OpenCV compatibility
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Convert the image to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask based on the HSV range values
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)

    # Create masks for White and Black
    white_lower = np.array([0, 0, 40])
    white_upper = np.array([255, 100, 255])
    mask_white = cv2.inRange(imgHSV, white_lower, white_upper)

    black_lower = np.array([0, 0, 0])
    black_upper = np.array([255, 255, 40])
    mask_black = cv2.inRange(imgHSV, black_lower, black_upper)

    orange_lower = np.array([5, 140, 200])
    orange_upper = np.array([15, 235, 255])
    mask_orange= cv2.inRange(imgHSV, orange_lower, orange_upper)

    blue_lower = np.array([100, 160, 110])
    blue_upper = np.array([170, 255, 160])
    mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper)
    # Edge detection on masks
    edges_white = cv2.Canny(mask_white, 100, 100)
    edges_black = cv2.Canny(mask_black, 100, 100)
    edges_orange = cv2.Canny(mask_orange, 100, 100)
    edges_blue = cv2.Canny(mask_blue, 100, 100) 

    contours_orange, _ = cv2.findContours(edges_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(edges_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw bounding rectangles around contours on the original image
    img_contours = img.copy()
    for contour in contours_orange:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img_contours, (x, y), (x + w, y + h), (0, 165, 255), 2)  # Orange rectangle

    for contour in contours_blue:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img_contours, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue rectangle

    if contours_orange(x) > contours_blue(x):
        print("Orange detected first")
    elif contours_blue(x) > contours_orange(x):
        print("Blue detected first")
    else:
        print("No colours detected")

    # Initialize variables for lane tracking
    height, width = edges_black.shape
    left_edges = []
    right_edges = []

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance+1.15, 2)

    if distance<=100:
        print("distance:",distance,"cm", "Hitting the wall")
        kit.servo[0].angle = 45
        i=1
        
    else:
        print("moving forward")
        i=0
    

    # Loop through each row to find edges
    for i in range(height):
        row = edges_black[i, :]

        # Detect leftmost edge in the left half
        left = np.where(row[:width // 2] > 0)[0]
        if len(left) > 0:
            left_edges.append(left[-1])  # Take the last detected edge on the left side
        else:
            left_edges.append(None)  # No edge detected

        # Detect rightmost edge in the right half
        right = np.where(row[width // 2:] > 0)[0]
        if len(right) > 0:
            right_edges.append(right[0] + (width // 2))  # Adjust index for right side
        else:
            right_edges.append(None)  # No edge detected

    # Draw lines based on detected edges
    for i in range(height):
        left_avg = left_edges[i]
        right_avg = right_edges[i]

        if left_avg is not None and right_avg is not None:
            # Calculate the midpoint and draw the path
            middle_line = (left_avg + right_avg) // 2

            # Draw the path line (white) with increased thickness
            cv2.line(edges_black, (middle_line, i), (middle_line, i), (255, 255, 255), 3)  # Increased thickness

            # Check for sharp turns (if needed in future)
            if (right_avg - left_avg) < 50:  # Threshold for sharp turn indication
                pass

    # Show edges with path lines drawn
    cv2.imshow("Path on Edges", edges_black)

    # Show the input image (no turn message)
    cv2.imshow("Input", img)

    # Break the loop when 'q' or 'Esc' is pressed
    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
        break

# Release the video capture object and close windows
picam2.stop()
cv2.destroyAllWindows()
