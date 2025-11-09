import cv2
import numpy as np

# Constants for the color range (HSV)
RED1_LO = np.array([0, 100, 80], dtype=np.uint8)
RED1_HI = np.array([10, 255, 255], dtype=np.uint8)
RED2_LO = np.array([170, 100, 80], dtype=np.uint8)
RED2_HI = np.array([180, 255, 255], dtype=np.uint8)
GREEN_LO = np.array([35, 60, 60], dtype=np.uint8)
GREEN_HI = np.array([95, 255, 255], dtype=np.uint8)

# Minimum area for the obstacle to trigger bypass
BY_MIN_AREA = 800  # px^2

# Initialize the webcam (usually 0 for default camera)
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Read frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Convert the frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red and green color detection
    mask_red1 = cv2.inRange(hsv, RED1_LO, RED1_HI)
    mask_red2 = cv2.inRange(hsv, RED2_LO, RED2_HI)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, GREEN_LO, GREEN_HI)

    # Find contours for the red and green areas
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the detected red contours and check area
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > BY_MIN_AREA:
            # If the red obstacle is large enough, show that it's a valid obstacle
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(frame, f"Red Obstacle - Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Draw the detected green contours and check area
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > BY_MIN_AREA:
            # If the green obstacle is large enough, show that it's a valid obstacle
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"Green Obstacle - Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the live video feed with detected obstacles
    cv2.imshow("Obstacle Detection", frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close the window
cap.release()
cv2.destroyAllWindows()
