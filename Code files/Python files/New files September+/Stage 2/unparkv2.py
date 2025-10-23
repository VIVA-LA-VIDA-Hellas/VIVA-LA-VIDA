import time
import board
import busio
import digitalio
from adafruit_pca9685 import PCA9685
import adafruit_vl53l0x

# ===============================
# SETUP I2C & ToF
# ===============================
i2c = busio.I2C(board.SCL, board.SDA)

# XSHUT pins
xshut_pins = {"left": board.D16, "right": board.D25, "front": board.D26, "back": board.D24}
addresses = {"left": 0x30, "right": 0x31, "front": 0x32, "back": 0x33}

xshuts = {}
for n, p in xshut_pins.items():
    x = digitalio.DigitalInOut(p)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False
    xshuts[n] = x

time.sleep(0.1)

sensors = {}
for name in ["left", "right", "front", "back"]:
    xshuts[name].value = True
    time.sleep(0.05)
    s = adafruit_vl53l0x.VL53L0X(i2c)
    s.set_address(addresses[name])
    s.start_continuous()
    sensors[name] = s
    print(f"{name.capitalize()} ToF ready: {hex(addresses[name])}")

# ===============================
# PCA9685 servo setup
# ===============================
pca = PCA9685(i2c)
pca.frequency = 50
SERVO_CHANNEL = 0
SERVO_MIN = 50
SERVO_MAX = 130
SERVO_PULSE_MIN = 1000
SERVO_PULSE_MAX = 2000
SERVO_PERIOD = 20000

def set_servo(angle):
    angle = max(SERVO_MIN, min(SERVO_MAX, angle))
    pulse = int(SERVO_PULSE_MIN + (SERVO_PULSE_MAX - SERVO_PULSE_MIN) *
                ((angle - SERVO_MIN) / (SERVO_MAX - SERVO_MIN)))
    pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / SERVO_PERIOD)

def tof_cm(sensor):
    try:
        d = sensor.range / 10.0
        return d
    except:
        return 999

# ===============================
# START LOGIC
# ===============================
time.sleep(0.5)  # wait for sensors
left_dist = tof_cm(sensors["left"])
right_dist = tof_cm(sensors["right"])
print(f"Initial Left: {left_dist:.2f} | Right: {right_dist:.2f}")

direction = "left" if left_dist >= right_dist else "right"
print(f"Chosen direction: {direction}")

set_servo(90)  # start centered
time.sleep(0.5)

# ===============================
# CONTROL LOOP
# ===============================
if direction == "left":
    print("Moving center until front < 2 cm...")
    set_servo(90)
    while tof_cm(sensors["front"]) > 2:
        print(f"Front: {tof_cm(sensors['front']):.2f}")
        time.sleep(0.1)

    print("Moving left until back < 3 cm...")
    set_servo(50)  # left
    while tof_cm(sensors["back"]) > 3:
        print(f"Back: {tof_cm(sensors['back']):.2f}")
        time.sleep(0.1)

    print("Turning right and going center...")
    set_servo(130)  # right turn
    time.sleep(0.5)
    set_servo(90)   # center

else:  # direction == "right"
    print("Moving center until front < 2 cm...")
    set_servo(90)
    while tof_cm(sensors["front"]) > 2:
        print(f"Front: {tof_cm(sensors['front']):.2f}")
        time.sleep(0.1)

    print("Moving right until back < 3 cm...")
    set_servo(130)  # right
    while tof_cm(sensors["back"]) > 3:
        print(f"Back: {tof_cm(sensors['back']):.2f}")
        time.sleep(0.1)

    print("Turning left and going center...")
    set_servo(50)   # left turn
    time.sleep(0.5)
    set_servo(90)   # center

print("Done sequence.")
