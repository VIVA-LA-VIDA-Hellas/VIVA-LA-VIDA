'''
This code uses the output from R_G_cannyoutline_WORKING and adds finds the coordinates of the corners
from the largest green and red object detected from the feed
It then creats a line connecting the top-bottom (left for green, 
right for red) corners so the vehicle can use them to then choose how to steer.
'''
                                                 
import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Initial values for HSV trackbars
h_min, h_max = 3, 18
s_min, s_max = 220, 255
v_min, v_max = 0, 255

# Capture video from file or webcam
# ==== CAMERA CALIBRATION PARAMETERS FOR FISHEYE CORRECTION ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== HSV RANGE FOR BLACK ====
# You can add white or specific lane colors if needed
black_lower = np.array([0, 0, 0])
black_upper = np.array([255, 255, 40])

# ==== START CAMERA ====
picam2 = Picamera2()
picam2.start()
time.sleep(2)

# Get undistort map once


# Function to find the largest contour and return its corner coordinates
def get_largest_contour_corners(contours):
    if len(contours) == 0:
        return None  # No contours found

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Corner coordinates based on bounding rectangle
    top_left = (x, y)
    top_right = (x + w, y)
    bottom_left = (x, y + h)
    bottom_right = (x + w, y + h)

    return top_left, top_right, bottom_left, bottom_right, largest_contour

while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    DIM = img.shape[1], img.shape[0]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2
    )

    # Convert the image to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red color mask (two ranges for the hue spectrum of red)
    red_lower1 = np.array([0, 200, 120])
    red_upper1 = np.array([10, 255, 200])
    mask_red = cv2.inRange(imgHSV, red_lower1, red_upper1)
    #red_lower2 = np.array([170, 120, 70])
    #red_upper2 = np.array([180, 255, 255])
    #mask_red2 = cv2.inRange(imgHSV, red_lower2, red_upper2)
    #mask_red = mask_red1 | mask_red2

    # Green color mask
    green_lower = np.array([50, 160, 20])
    green_upper = np.array([100, 255, 155])
    mask_green = cv2.inRange(imgHSV, green_lower, green_upper)

    # Edge detection on masks
    edges_red = cv2.Canny(mask_red, 100, 100)
    edges_green = cv2.Canny(mask_green, 100, 100)

    # Find contours for red and green edges
    contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(edges_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get the corner coordinates and largest contours for red and green objects
    red_data = get_largest_contour_corners(contours_red)
    green_data = get_largest_contour_corners(contours_green)

    # Copy the original image to draw contours
    img_contours = img.copy()

    # Draw red contour, bounding box, and connecting line (yellow)
    if red_data:
        corners_red, largest_red_contour = red_data[:-1], red_data[-1]
        cv2.drawContours(img_contours, [largest_red_contour], -1, (0, 0, 255), 2)  # Red outline
        cv2.rectangle(img_contours, corners_red[0], corners_red[3], (0, 0, 255), 2)  # Red bounding box
        cv2.line(img_contours, corners_red[1], corners_red[3], (0, 255, 255), 2)  # Yellow line (top-right to bottom-right)
        print("Red object corners:", corners_red)

    # Draw green contour, bounding box, and connecting line (blue)
    if green_data:
        corners_green, largest_green_contour = green_data[:-1], green_data[-1]
        cv2.drawContours(img_contours, [largest_green_contour], -1, (0, 255, 0), 2)  # Green outline
        cv2.rectangle(img_contours, corners_green[0], corners_green[3], (0, 255, 0), 2)  # Green bounding box
        cv2.line(img_contours, corners_green[0], corners_green[2], (255, 0, 0), 2)  # Blue line (top-left to bottom-left)
        print("Green object corners:", corners_green)

    # Display image with contours and lines
    cv2.imshow('Contours', img_contours)

    # Break the loop when 'q' or 'Esc' is pressed
    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
        break

# Release the video capture object and close windows
cv2.destroyAllWindows()
