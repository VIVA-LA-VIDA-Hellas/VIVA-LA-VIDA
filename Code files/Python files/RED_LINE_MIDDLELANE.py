import cv2
import numpy as np
from picamera2 import Picamera2
import time
import scipy.ndimage

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
img = picam2.capture_array()
img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
DIM = img.shape[1], img.shape[0]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, DIM, cv2.CV_16SC2
)

lane_width_guess = 120  # Adjust based on your track

while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    img = cv2.flip(img, -1)  # Flip vertically and horizontally

    # Convert to HSV and mask black areas (lanes)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_black = cv2.inRange(imgHSV, black_lower, black_upper)
    edges_black = cv2.Canny(mask_black, 100, 100)

    height, width = edges_black.shape

    # === Detect Horizontal Lines ===
    lines = cv2.HoughLinesP(edges_black, 1, np.pi / 180, threshold=100, minLineLength=60, maxLineGap=10)
    prominent_y = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if abs(y2 - y1) < 10:
                prominent_y = max(prominent_y, y1)

    # Start row = lower of (bottom 2/3) and prominent line
    min_row = max(height // 3, prominent_y)

    path_points = []

    for i in range(min_row, height):
        row = edges_black[i, :]
        left = np.where(row[:width // 2] > 0)[0]
        right = np.where(row[width // 2:] > 0)[0]

        l = left[-1] if len(left) > 0 else None
        r = right[0] + width // 2 if len(right) > 0 else None

        if l is not None and r is not None:
            mid = (l + r) // 2
        elif l is not None:
            mid = l + lane_width_guess
        elif r is not None:
            mid = r - lane_width_guess
        else:
            continue

        mid = max(0, min(mid, width - 1))
        path_points.append((mid, i))

    # === SMOOTH & DRAW PATH ===
    if len(path_points) > 5:
        smoothed = np.array(path_points, dtype=np.int32)
        smoothed[:, 0] = scipy.ndimage.gaussian_filter1d(smoothed[:, 0], sigma=2)
        smoothed = smoothed.astype(int)

        for i in range(1, len(smoothed)):
            cv2.line(edges_black, tuple(smoothed[i - 1]), tuple(smoothed[i]), (255, 255, 255), 2)

        # Find the average x-coordinate of the path
        avg_x = int(np.mean(smoothed[:, 0]))
        
        # Draw the red line on the existing img (de-warped input)
        cv2.line(img, (avg_x, 0), (avg_x, height), (0, 0, 255), 2)

    # Optional: Draw detected horizontal line
    if prominent_y > 0:
        cv2.line(edges_black, (0, prominent_y), (width, prominent_y), (100, 100, 100), 1)

    # Show the de-warped input with the red vertical line and lane path
    cv2.imshow("De-warped Input", img)
    cv2.imshow("Lane Path", edges_black)

    key = cv2.waitKey(5)
    if key == ord('q') or key == 27:
        break

# Cleanup
picam2.stop()
cv2.destroyAllWindows()
