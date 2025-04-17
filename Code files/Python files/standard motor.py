import time
from adafruit_motorkit import MotorKit

# Initialize the motor kit (this automatically detects the connected I2C motor shield)
kit = MotorKit()

# Function to rotate the motor forward
def rotate_motor_forward():
    print("Rotating motor forward")
    kit.motor3.throttle = 1.0  # Full speed forward

# Function to rotate the motor backward
def rotate_motor_backward():
    print("Rotating motor backward")
    kit.motor3.throttle = -1.0  # Full speed backward

# Function to stop the motor
def stop_motor():
    print("Stopping motor")
    kit.motor3.throttle = 0.0  # Stop the motor

# Main program loop to test motor control
try:
    # Rotate the motor forward for 3 seconds
    rotate_motor_forward()
    time.sleep(3)

    # Rotate the motor backward for 3 seconds
    rotate_motor_backward()
    time.sleep(3)

    # Stop the motor
    stop_motor()

finally:
    stop_motor()  # Ensure motor stops when program ends

