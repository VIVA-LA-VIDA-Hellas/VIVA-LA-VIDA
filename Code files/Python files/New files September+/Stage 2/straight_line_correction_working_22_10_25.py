import smbus2
import time
from pca9685_control import set_servo_angle, set_motor_speed
import threading

MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2  # reverse channel
SERVO_CHANNEL = 0
time.sleep(0.2)

# ================== MPU6050 Setup ==================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
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

# --- Measure gyro Z bias at startup ---
def get_gyro_z_bias(samples=200):
    total = 0.0
    for _ in range(samples):
        raw = read_raw_data(GYRO_ZOUT_H)
        total += raw / 131.0
        time.sleep(0.005)
    return total / samples

mpu6050_init()
print("Measuring gyro bias... Keep sensor still.")
gyro_z_bias = get_gyro_z_bias()
print(f"Gyro Z bias: {gyro_z_bias:.3f} deg/s")

# ================== Yaw Tracking ==================
yaw = 0.0          # cumulative rotation in degrees
yaw_lock = threading.Lock()  # thread-safe reset
last_time = time.time()

# Servo constants
CENTER_ANGLE = 90
LEFT_ANGLE = 105
RIGHT_ANGLE = 75
CENTER_DEADZONE = 5  # degrees ± around 0 for "center"

# Servo gradual movement
current_servo_angle = CENTER_ANGLE
SERVO_STEP = 6.0  # degrees per loop

# Center servo initially
set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
print("Tracking turns...")

# --- Optional: Thread to reset yaw ---
def reset_yaw_listener():
    global yaw
    while True:
        input("Press ENTER to reset yaw to 0°")
        with yaw_lock:
            yaw = 0.0
            print("Yaw reset to 0° (center)")

threading.Thread(target=reset_yaw_listener, daemon=True).start()

try:
    while True:
        set_motor_speed(MOTOR_FWD, MOTOR_REV, 20)
        now = time.time()
        dt = now - last_time
        last_time = now

        # Read gyro Z and subtract bias
        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias

        # Integrate angular velocity to get yaw
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # Decide target servo position based on yaw with deadzone
        if -CENTER_DEADZONE <= current_yaw <= CENTER_DEADZONE:
            target_servo = CENTER_ANGLE
            position_str = "CENTER"
        elif current_yaw > CENTER_DEADZONE:
            target_servo = LEFT_ANGLE
            position_str = "LEFT"
        else:
            target_servo = RIGHT_ANGLE
            position_str = "RIGHT"

        # Gradually move servo toward target
        if current_servo_angle < target_servo:
            current_servo_angle += SERVO_STEP
            if current_servo_angle > target_servo:
                current_servo_angle = target_servo
        elif current_servo_angle > target_servo:
            current_servo_angle -= SERVO_STEP
            if current_servo_angle < target_servo:
                current_servo_angle = target_servo

        set_servo_angle(SERVO_CHANNEL, current_servo_angle)

        # Print yaw and servo info
        print(f"Yaw={current_yaw:.2f}° | Servo={current_servo_angle:.1f} | Target={target_servo} | {position_str}")

        # Detect 90° turn
        if abs(current_yaw) >= 90:
            direction = 'LEFT' if current_yaw > 0 else 'RIGHT'
            print(f"Turn completed: {direction}")
            with yaw_lock:
                yaw = 0  # reset yaw after detected turn

        time.sleep(0.02)

except KeyboardInterrupt:
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    print("Stopped and reset.")
