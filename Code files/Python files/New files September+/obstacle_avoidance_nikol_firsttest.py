import cv2
import numpy as np
from picamera2 import Picamera2

# ==== CAMERA CALIBRATION PARAMETERS FOR FISHEYE CORRECTION ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== HSV RANGE FOR BLACK (Not used here but can be added if needed) ====
black_lower = np.array([0, 0, 0])
black_upper = np.array([255, 255, 40])

# ==== START CAMERA (Picamera2) ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Function to find the largest contour and return its corner coordinates
def get_largest_contour_corners(contours):
    if len(contours) == 0:
        return None  # No contours found

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Corner coordinates
    top_left = (x, y)
    top_right = (x + w, y)
    bottom_left = (x, y + h)
    bottom_right = (x + w, y + h)

    return top_left, top_right, bottom_left, bottom_right, largest_contour

while True:
    # Capture frame
    imageog = picam2.capture_array()
    img = cv2.cvtColor( imageog, cv2.COLOR_BGR2RGB )

    # Fisheye correction (optional)
    DIM = img.shape[1], img.shape[0]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2
    )
    # img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)  # Uncomment if needed

    # Convert to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Red mask
    red_lower1 = np.array([0, 150, 120])
    red_upper1 = np.array([20, 255, 255])
    mask_red = cv2.inRange(imgHSV, red_lower1, red_upper1)

    # Green mask
    green_lower = np.array([60, 70, 80])
    green_upper = np.array([80, 210, 140])
    mask_green = cv2.inRange(imgHSV, green_lower, green_upper)

    # Edges
    edges_red = cv2.Canny(mask_red, 100, 100)
    edges_green = cv2.Canny(mask_green, 100, 100)

    # Contours
    contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(edges_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Largest contours
    red_data = get_largest_contour_corners(contours_red)
    green_data = get_largest_contour_corners(contours_green)

    img_contours = img.copy()

    red_area = 0
    green_area = 0
  
    if red_data:
        corners_red, largest_red_contour = red_data[:-1], red_data[-1]
        cv2.drawContours(img_contours, [largest_red_contour], -1, (0, 0, 255), 2)
        cv2.rectangle(img_contours, corners_red[0], corners_red[3], (0, 0, 255), 2)
        cv2.line(img_contours, corners_red[1], corners_red[3], (0, 255, 255), 2)
        print("Red object corners:", corners_red)
        red_area = cv2.contourArea(largest_red_contour)

    if green_data:
        corners_green, largest_green_contour = green_data[:-1], green_data[-1]
        cv2.drawContours(img_contours, [largest_green_contour], -1, (0, 255, 0), 2)
        cv2.rectangle(img_contours, corners_green[0], corners_green[3], (0, 255, 0), 2)
        cv2.line(img_contours, corners_green[0], corners_green[2], (255, 0, 0), 2)
        print("Green object corners:", corners_green)
        green_area = cv2.contourArea(largest_green_contour)
      
    if green_area > red_area and green_area > 20:
        print("turn left")
    elif red_area > green_area and red_area > 20:
        print("turn right")
    else:
        print("No object detected")

    cv2.imshow('Contours', img_contours)
    

    key = cv2.waitKey(5) & 0xFF
    if key == ord('q') or key == 27:
        break

picam2.stop()
cv2.destroyAllWindows()
