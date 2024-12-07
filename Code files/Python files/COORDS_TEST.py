import cv2
import numpy as np

# Initial values for HSV trackbars
h_min, h_max = 3, 18
s_min, s_max = 220, 255
v_min, v_max = 0, 255

# Capture video from file or webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

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
    ret, img = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the image to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red color mask (two ranges for the hue spectrum of red)
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(imgHSV, red_lower1, red_upper1)
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(imgHSV, red_lower2, red_upper2)
    mask_red = mask_red1 | mask_red2

    # Green color mask
    green_lower = np.array([45, 200, 0])
    green_upper = np.array([180, 255, 150])
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
cap.release()
cv2.destroyAllWindows()
