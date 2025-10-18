import cv2
import numpy as np
from picamera2 import Picamera2

# ==== CAMERA CALIBRATION PARAMETERS FOR FISHEYE CORRECTION ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== START CAMERA ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()  

Turn = "No"

while True:
    # Capture frame
    imageog = picam2.capture_array()
    img = imageog.copy()  # Already RGB from Picamera2

    # Fisheye correction (optional)
    DIM = img.shape[1], img.shape[0]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2
    )
    # img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Convert to HSV
    imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Orange mask
    orange_lower = np.array([0, 110, 110])
    orange_upper = np.array([20, 160, 230])
    mask_orange = cv2.inRange(imgHSV, orange_lower, orange_upper)

    # Blue mask
    blue_lower = np.array([100, 60, 80])
    blue_upper = np.array([120, 120, 170])
    mask_blue = cv2.inRange(imgHSV, blue_lower, blue_upper)

    # Edge detection
    edges_orange = cv2.Canny(mask_orange, 50, 150)
    edges_blue = cv2.Canny(mask_blue, 50, 150)

    # Detect lines using Hough Transform
    lines_orange = cv2.HoughLinesP(edges_orange, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    lines_blue = cv2.HoughLinesP(edges_blue, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    img_lines = img.copy()

    # Draw lines for visualization
    if lines_orange is not None:
        for x1, y1, x2, y2 in lines_orange[:, 0]:
            cv2.line(img_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)

    if lines_blue is not None:
        for x1, y1, x2, y2 in lines_blue[:, 0]:
            cv2.line(img_lines, (x1, y1), (x2, y2), (0, 165, 255), 2)

    # ---- Decide based on which line is lower at the center x ----
    center_x = img.shape[1] // 2

    def y_at_center(lines):
        if lines is None:
            return -1
        y_values = []
        for x1, y1, x2, y2 in lines[:, 0]:
            if x1 != x2:  # avoid vertical divide-by-zero
                slope = (y2 - y1) / (x2 - x1)
                y = slope * (center_x - x1) + y1
                y_values.append(y)
            else:
                # vertical line, take min/max depending on which side of center
                if x1 == center_x:
                    y_values.append(max(y1, y2))
        return max(y_values) if y_values else -1

    orange_y = y_at_center(lines_orange)
    blue_y = y_at_center(lines_blue)

    if orange_y > blue_y:
        print("Turn right (orange line lower at center)")
        Turn = "Right"
    elif blue_y > orange_y:
        print("Turn left (blue line lower at center)")
        Turn = "Left"
    else:
        print("No line detected at center")
        Turn = "No"

    cv2.imshow('Lines', img_lines)

    key = cv2.waitKey(5) & 0xFF
    if key == ord('q') or key == 27:
        break

picam2.stop()
cv2.destroyAllWindows()
