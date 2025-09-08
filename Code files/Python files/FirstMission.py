import time
import RPi.GPIO as GPIO

# Set up the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the three sensors
TRIG_1 = 22
ECHO_1 = 23
TRIG_2 = 5
ECHO_2 = 6
TRIG_3 = 27
ECHO_3 = 17

# Set up Trigger pins as output and Echo pins as input
GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)
GPIO.setup(TRIG_3, GPIO.OUT)
GPIO.setup(ECHO_3, GPIO.IN)

# Ensure all Triggers are low initially
GPIO.output(TRIG_1, GPIO.LOW)
GPIO.output(TRIG_2, GPIO.LOW)
GPIO.output(TRIG_3, GPIO.LOW)

# Give time for the sensors to initialize
time.sleep(0.2)

def measure_distance(trigger_pin, echo_pin):
    # Send a pulse to the trigger pin
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(0.00001)  # Pulse duration (10 microseconds)
    GPIO.output(trigger_pin, GPIO.LOW)

    # Wait for the Echo pin to go HIGH and record the start time
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()

    # Wait for the Echo pin to go LOW and record the end time
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the time difference
    pulse_duration = pulse_end - pulse_start

    # Calculate the distance (speed of sound is ~34300 cm/s)
    distance = pulse_duration * 34300 / 2  # Divide by 2 for the round trip

    return distance

try:
    while True:
        # Measure distance for each sensor
        distance_1 = measure_distance(TRIG_1, ECHO_1)
        time.sleep(0.00001)
        distance_2 = measure_distance(TRIG_2, ECHO_2)
        time.sleep(0.00001)
        distance_3 = measure_distance(TRIG_3, ECHO_3)

        # Print the distances for each sensor
        print(f"front Distance: {distance_1:.2f} cm")
        print(f"right Distance: {distance_2:.2f} cm")
        print(f"left Distance: {distance_3:.2f} cm\n")

        # Wait 1 second before the next reading
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Measurement stopped by user")
    GPIO.cleanup()

