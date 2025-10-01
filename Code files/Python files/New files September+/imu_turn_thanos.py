import smbus2
import time
import RPi.GPIO as GPIO
from pca9685_control import set_servo_angle, set_motor_speed

MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2  # reverse channel
SERVO_CHANNEL =0

# ================== GPIO Setup ==================
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the three sensors
TRIG_1, ECHO_1 = 22, 23  # Front
TRIG_2, ECHO_2 = 5, 6    # Right
TRIG_3, ECHO_3 = 27, 17  # Left

# Setup Trigger as output, Echo as input
for trig, echo in [(TRIG_1, ECHO_1), (TRIG_2, ECHO_2), (TRIG_3, ECHO_3)]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trig, GPIO.LOW)

# Give time for sensors to settle
time.sleep(0.2)

# ================== Distance Measurement ==================
def measure_distance(trigger_pin, echo_pin, timeout=0.03, retries=2):
    """Measure distance from one ultrasonic sensor. Returns None if no echo."""
    for attempt in range(retries):
        # Send a short pulse
        GPIO.output(trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(trigger_pin, GPIO.LOW)

        start_time = time.time()
        pulse_start = start_time

        # Wait for Echo to go HIGH
        while GPIO.input(echo_pin) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start - start_time > timeout:
                break  # Timeout
        else:
            # Echo went HIGH; now wait for it to go LOW
            start_time = time.time()
            pulse_end = start_time
            while GPIO.input(echo_pin) == GPIO.HIGH:
                pulse_end = time.time()
                if pulse_end - start_time > timeout:
                    break  # Timeout

            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 34300 / 2
            return distance  # in cm
        # Retry if this attempt failed
        time.sleep(0.01)
    return None  # All retries failed

# I2C address of MPU6050
MPU6050_ADDR = 0x68

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38

GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus2.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low  = bus.read_byte_data(MPU6050_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

mpu6050_init()
print("Tracking turns...")

yaw = 0.0          # accumulated yaw angle
turns = 0          # number of 90° turns detected
last_time = time.time()

try:
    set_servo_angle(SERVO_CHANNEL,90) 
    while True:

        # Measure distances for all sensors
        distance_front = measure_distance(TRIG_1, ECHO_1)
        distance_right = measure_distance(TRIG_2, ECHO_2)
        distance_left  = measure_distance(TRIG_3, ECHO_3)

        # Function for clean printing
        def format_distance(d):
            return 'N/A' if d is None else f'{d:.2f} cm'

        # Read gyro Z (yaw rate)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        Gz = gyro_z / 131.0  # scale to deg/sec (±250°/s range)

        now = time.time()
        dt = now - last_time
        last_time = now

        # Integrate gyro rate to get yaw angle
        yaw += Gz * dt

        # If robot turned at least ±90°, count it
        if abs(yaw) >= 90:
            turns += 1
            print(f"TURN DETECTED! Total turns: {turns}, Direction: {'LEFT' if yaw > 0 else 'RIGHT'}")
            yaw = 0  # reset after counting

        if distance_front >= 25:
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 45)
            set_servo_angle(SERVO_CHANNEL,90) 
        else:
            set_motor_speed(MOTOR_FWD, MOTOR_REV, -30)
            set_servo_angle(SERVO_CHANNEL,90)
