from HC-SR04_Controller import get_filtered_distance
import RPi.GPIO as GPIO
import time

TRIG_PIN = 13
ECHO_PIN = 15

try:
    while True:
        dist = get_filtered_distance(TRIG_PIN, ECHO_PIN)
        if dist is not None:
            print(f"Filtered distance: {dist:.2f} cm")
        else:
            print("No echo detected")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
