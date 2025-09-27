'''
This code detects black and white using a hsv mask and img canny
It then outputs a line in the middle of the walls it detects on
each side of the vehicle, showing the path it needs to follow.
'''
import cv2
import numpy as np


def empty(x):
    pass


# Initial values for HSV trackbars
h_min, h_max = 3, 18
s_min, s_max = 220, 255
v_min, v_max = 0, 255

# Capture video from file or webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while True:
    ret, img = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

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

    # Edge detection on masks
    edges_white = cv2.Canny(mask_white, 100, 100)
    edges_black = cv2.Canny(mask_black, 100, 100)

    # Initialize variables for lane tracking
    height, width = edges_black.shape
    left_edges = []
    right_edges = []

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
cap.release()
cv2.destroyAllWindows()
