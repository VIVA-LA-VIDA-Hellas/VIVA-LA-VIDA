import RPi.GPIO as GPIO
import time
import numpy as np

# Kalman Filter Initialization
x = np.matrix([[0.]])      # initial distance estimate
p = np.matrix([[1000.]])   # initial uncertainty

u = np.matrix([[0.]])
w = np.matrix([[0.]])
A = np.matrix([[1.]])
B = np.matrix([[0.]])
H = np.matrix([[1.]])
Q = np.matrix([[0.00001]])     # process noise
R = np.matrix([[0.10071589]])  # measurement noise

def kalman(x_, p_, x_measured):
    # prediction
    x_predicted = A*x_ + B*u + w
    p_predicted = A*p_*np.transpose(A) + Q

    # measurement update
    y = x_measured - (H*x_predicted)
    s = H*p_predicted*np.transpose(H) + R
    K = p_predicted*np.transpose(H) * np.linalg.inv(s)

    x_estimated = x_predicted + (K*y)
    p_estimated = (1 - (K*H)) * p_predicted

    return x_estimated, p_estimated

def get_filtered_distance(trig, echo, timeout=0.02):
    """Return Kalman-filtered distance in cm from an HC-SR04."""
    global x, p

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

    # Trigger the sensor
    GPIO.output(trig, False)
    time.sleep(0.05)  # let sensor settle

    GPIO.output(trig, True)
    time.sleep(0.00001)  # 10 µs pulse
    GPIO.output(trig, False)

    # Wait for echo start
    start_time = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - start_time > timeout:
            return None  # timeout: no echo

    # Wait for echo end
    start_time = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - start_time > timeout:
            return None  # timeout: echo too long

    pulse = pulse_end - pulse_start
    distance = pulse * 17150  # convert to cm

    # Kalman update
    x, p = kalman(x, p, distance)

    return float(x.item(0))  # return filtered value as float
