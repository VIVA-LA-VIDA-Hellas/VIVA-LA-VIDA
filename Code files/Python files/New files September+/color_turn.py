# ========= 4x VL53L0X + TCS34725 (no time.sleep, fast custom TCS, RGB detection + Blue boost) =========
# - ToFs via XSHUT, readdress to 0x30..0x33 (no time.sleep)
# - Waits (busy-loop) for you to plug/power the TCS34725 (0x29)
# - Custom fast TCS driver (no Adafruit TCS lib), RGB-based detection
# - "Blue boost" makes blue stand out vs white much more
# ================================================================================================

import time
import board
import busio
import digitalio
import adafruit_vl53l0x   # keep Adafruit driver ONLY for ToF
from pca9685_control import set_motor_speed, set_servo_angle

SERVO_CHANNEL = 0
set_servo_angle(SERVO_CHANNEL, 90) 
MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2   # reverse channel

# ---------------- I2C ----------------
i2c = busio.I2C(board.SCL, board.SDA)

def i2c_scan_contains(addr: int) -> bool:
    while not i2c.try_lock():
        pass
    try:
        return addr in i2c.scan()
    finally:
        i2c.unlock()

# ---------------- Fast custom TCS34725 driver ----------------
class TCS34725Fast:
    _ADDR = 0x29
    _CMD, _CMD_AUTOINC = 0x80, 0x20
    _ENABLE, _ATIME, _CONTROL, _ID, _STATUS, _CDATAL = 0x00, 0x01, 0x0F, 0x12, 0x13, 0x14
    _PON, _AEN = 0x01, 0x02
    GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X = 0x00, 0x01, 0x02, 0x03

    def __init__(self, i2c_bus, atime=0xFF, gain=GAIN_4X):
        self.i2c = i2c_bus
        self.addr = self._ADDR
        # optional ID read (fast)
        _ = self._read_u8(self._ID)
        self.set_integration_atime(atime)   # 0xFF=~2.4ms fastest
        self.set_gain(gain)
        # power on + enable RGBC (busy-wait ~3ms, no sleep)
        self._write_u8(self._ENABLE, self._PON)
        self._busy_wait_ms(3)
        self._write_u8(self._ENABLE, self._PON | self._AEN)

    # --- low-level ---
    def _busy_wait_ms(self, ms):
        t = time.monotonic() + ms/1000.0
        while time.monotonic() < t:
            pass

    def _write_u8(self, reg, val):
        buf = bytes([self._CMD | (reg & 0x1F), val & 0xFF])
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, buf)
        finally:
            self.i2c.unlock()

    def _read_u8(self, reg) -> int:
        w = bytes([self._CMD | (reg & 0x1F)])
        r = bytearray(1)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.addr, w, r)
        finally:
            self.i2c.unlock()
        return r[0]

    def _read_block(self, start_reg, nbytes) -> bytes:
        w = bytes([self._CMD | self._CMD_AUTOINC | (start_reg & 0x1F)])
        r = bytearray(nbytes)
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto_then_readfrom(self.addr, w, r)
        finally:
            self.i2c.unlock()
        return bytes(r)

    # --- public API ---
    def set_integration_atime(self, atime):
        # 0xFFβ‰2.4ms, 0xF6β‰24ms ... 0x00β‰614ms
        self._write_u8(self._ATIME, atime)

    def set_gain(self, gain_code):
        self._write_u8(self._CONTROL, gain_code & 0x03)

    def data_ready(self) -> bool:
        return (self._read_u8(self._STATUS) & 0x01) != 0  # AVALID

    def read_raw(self):
        b = self._read_block(self._CDATAL, 8)
        c = b[0] | (b[1] << 8)
        r = b[2] | (b[3] << 8)
        g = b[4] | (b[5] << 8)
        bl = b[6] | (b[7] << 8)
        return c, r, g, bl

    def read_rgb_bytes(self):
        c, r, g, bl = self.read_raw()
        if c == 0:
            return 0, 0, 0, 0
        rn = min(255, int((r * 255) / c))
        gn = min(255, int((g * 255) / c))
        bn = min(255, int((bl * 255) / c))
        return rn, gn, bn, c

# ------------- ToF XSHUT pins -------------
xshut_pins = {
    "left":  board.D16,
    "right": board.D25,
    "front": board.D26,
    "back":  board.D24,
}
addresses = {"left":0x30, "right":0x31, "front":0x32, "back":0x33}

xshuts = {}
for name, pin in xshut_pins.items():
    x = digitalio.DigitalInOut(pin)
    x.direction = digitalio.Direction.OUTPUT
    x.value = False   # hold all in reset
    xshuts[name] = x

sensors = {}

def wait_for_0x29(max_seconds=2.0) -> bool:
    t0 = time.monotonic()
    while (time.monotonic() - t0) < max_seconds:
        if i2c_scan_contains(0x29):
            return True
    return False

def init_one_tof(name: str):
    for n, x in xshuts.items():
        x.value = (n == name)
    if not wait_for_0x29(max_seconds=2.0):
        raise RuntimeError(f"[TOF:{name}] 0x29 did not appear (check XSHUT/wiring).")
    s = adafruit_vl53l0x.VL53L0X(i2c)   # default 0x29
    s.set_address(addresses[name])
    try:
        s.start_continuous()
    except AttributeError:
        pass
    sensors[name] = s
    print(f"[TOF] {name:6s} -> {hex(addresses[name])} ready")

# Bring up ToFs one-by-one
for name in ["left","right","front","back"]:
    init_one_tof(name)

print("\nβ… ToFs ready (0x30..0x33). Now plug/power the TCS34725 (0x29).")

def wait_for_tcs():
    while True:
        if i2c_scan_contains(0x29):
            return

wait_for_tcs()

# Fast TCS: keep shortest integration for speed, boost blue in software
tcs = TCS34725Fast(i2c, atime=0xFF, gain=TCS34725Fast.GAIN_4X)
print("[TCS34725] Fast driver initialized.\n")

# ---------------- Helpers ----------------
def tof_cm(sensor):
    try:
        cm = sensor.range / 10.0
        if cm <= 0 or cm > 1500:
            return 999.0
        return cm
    except Exception:
        return 999.0

# ===== RGB-only classifier with Blue boost =====
BLUE_GAIN = 4.0          # amplify blue difference (try 3β€“8)
BLUE_THRESH = 120         # detection threshold after boost
ORANGE_THRESH = 0       # how strong red dominance must be
MIN_CLEAR = 30           # ignore super dark frames

def classify_rgb_boost(r, g, b, clear):
    if clear < MIN_CLEAR:
        return "Other", 0, 0

    # Blue boost: how much B stands above the other channels
    blue_score = max(0, b - max(r, g))
    blue_boosted = int(min(255, blue_score * BLUE_GAIN))

    # Orange score: red dominance (red vs average of green+blue)
    orange_score = max(0, r - (g + b) / 2)

    if blue_boosted >= BLUE_THRESH and b > 127:
        return "Blue", blue_boosted, int(orange_score)
    if orange_score >= ORANGE_THRESH and r > 55:
        return "Orange", blue_boosted, int(orange_score)
    return "Other", blue_boosted, int(orange_score)

# Optional smoothing for display (no sleep)
alpha = 0.35
rf=gf=bf=0

print("=== Streaming (Ctrl+C to stop) ===")

# ================== BEGIN MERGED IMU + TURNING ADDITIONS FROM SECOND SCRIPT ==================
import smbus2
import threading
import RPi.GPIO as GPIO
from pca9685_control import set_servo_angle, set_motor_speed
# picamera2 import present in merged script later (kept for completeness)
from picamera2 import Picamera2
import numpy as np
import cv2

# ================== Common Constants ==================
MOTOR_FWD = 1   # forward channel
MOTOR_REV = 2   # reverse channel
SERVO_CHANNEL = 0

time.sleep(0.2)

# ==== CAMERA CALIBRATION PARAMETERS FOR FISHEYE CORRECTION ====
K = np.array([[320, 0, 320],
              [0, 320, 240],
              [0, 0, 1]], dtype=np.float32)
D = np.array([-0.28, 0.11, 0, 0], dtype=np.float32)

# ==== START CAMERA ====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()  

Turn = "No"

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
yaw = 0.0
yaw_lock = threading.Lock()
last_time = time.time()

CENTER_ANGLE = 90
TURN_ANGLE = 120
CENTER_DEADZONE = 5
current_servo_angle = CENTER_ANGLE
SERVO_STEP = 6.0

set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)

# ================== GPIO Setup ==================
GPIO.setmode(GPIO.BCM)
TRIG_FRONT, ECHO_FRONT = 22, 23
TRIG_LEFT, ECHO_LEFT = 27, 17

for trig, echo in [(TRIG_FRONT, ECHO_FRONT), (TRIG_LEFT, ECHO_LEFT)]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trig, GPIO.LOW)

time.sleep(0.2)

def measure_distance(trigger_pin, echo_pin, timeout=0.03, retries=2):
    for _ in range(retries):
        GPIO.output(trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trigger_pin, GPIO.LOW)

        start_time = time.time()
        pulse_start = start_time
        while GPIO.input(echo_pin) == GPIO.LOW:
            pulse_start = time.time()
            if pulse_start - start_time > timeout:
                break
        else:
            pulse_end = time.time()
            while GPIO.input(echo_pin) == GPIO.HIGH:
                if time.time() - pulse_end > timeout:
                    break
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 34300 / 2
            return distance
        time.sleep(0.01)
    return None

# ================== States ==================
STATE_FORWARD = 0
STATE_BACK_AND_TURN = 1
state = STATE_FORWARD

# ================== Helper Motor Functions ==================
def move_forward(speed=25):
    set_motor_speed(MOTOR_FWD, MOTOR_REV, speed)

def move_backward(speed=25):
    set_motor_speed(MOTOR_REV, MOTOR_FWD, speed)

def stop_motors():
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)


def y_at_center(lines):
    if lines is None:
        return -1
    y_values = []
    for x1, y1, x2, y2 in lines[:, 0]:
        if x1 != x2:  # avoid vertical divide-by-zero
            slope = (y2 - y1) / (x2 - x1)
            y = slope * (center_x - x1) + y1
            y_values.append(y)
        else:
            # vertical line, take min/max depending on which side of center
            if x1 == center_x:
                y_values.append(max(y1, y2))
    return max(y_values) if y_values else -1

# ================== MERGED TURN CONTROL VARIABLES ==================
turn_in_progress = False
turn_target_deg = 0.0
turn_direction = 1  # 1 = left (positive yaw), -1 = right (negative yaw)
turn_start_time = None
TURN_MOTOR_SPEED = 20  # speed used while pivoting
# ======================================================================================

# ================== Main Loop (merged) ==================
try:
    last_print = time.time()
    while True:
        # update yaw from MPU6050
        now = time.time()
        dt = now - last_time
        last_time = now

        Gz = (read_raw_data(GYRO_ZOUT_H) / 131.0) - gyro_z_bias
        with yaw_lock:
            yaw += Gz * dt
            current_yaw = yaw

        # ToF readings
        L = tof_cm(sensors["left"])
        R = tof_cm(sensors["right"])
        F = tof_cm(sensors["front"])
        Bk = tof_cm(sensors["back"])

        # Color when fresh
        color_text = " | RGB=?"
        if tcs.data_ready():
            r, g, b, clear = tcs.read_rgb_bytes()

            # simple EMA smoothing for nicer numbers (doesn't slow sampling)
            rf = int((1-alpha)*rf + alpha*r)
            gf = int((1-alpha)*gf + alpha*g)
            bf = int((1-alpha)*bf + alpha*b)

            label, blue_boosted, orange_score = classify_rgb_boost(rf, gf, bf, clear)
            color_text = (
                f" | R={rf:3d} G={gf:3d} B={bf:3d} C={clear:5d}"
                f" BlueBoost={blue_boosted:3d} OrangeScore={orange_score:3d}"
                f" -> {label}"
            )
        else:
            label = "Other"
            color_text = f" | -> {label}"

        # Print sensor summary occasionally
        if time.time() - last_print > 0.05:
            print(f"L={L:6.1f}  R={R:6.1f}  F={F:6.1f}  B={Bk:6.1f}{color_text} | Yaw={current_yaw:.2f}° | TurnInProg={turn_in_progress}")
            last_print = time.time()

        # If a turn is already in progress, ignore new color triggers until turn completes
        if turn_in_progress:
            set_servo_angle(SERVO_CHANNEL, 55)
            move_backward(TURN_MOTOR_SPEED)

            # check if desired yaw reached
            if abs(current_yaw) >= abs(turn_target_deg):
                # complete turn
                stop_motors()
                set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
                with yaw_lock:
                    yaw = 0.0
                turn_in_progress = False
                turn_target_deg = 0.0
                turn_direction = 1
                # after turn, resume forward motion next cycle
        else:
            # Normal streaming behavior: keep motors moving forward
            set_motor_speed(MOTOR_FWD, MOTOR_REV, 20)

            # Only use color sensor to trigger turns (no camera/ultrasonic/ToF decision)
            if label == "Blue":
                # start a left turn of 50 degrees using IMU yaw
                turn_in_progress = True
                turn_target_deg = 90.0
                turn_direction = 1
                with yaw_lock:
                    yaw = 0.0
                turn_start_time = time.time()
            elif label == "Orange":
                # start a left turn of 130 degrees using IMU yaw
                turn_in_progress = True
                turn_target_deg = 90.0
                turn_direction = 1
                with yaw_lock:
                    yaw = 0.0
                turn_start_time = time.time()

        # Small non-blocking delay to yield CPU
        time.sleep(0.02)

except KeyboardInterrupt:
    set_motor_speed(MOTOR_FWD, MOTOR_REV, 0)
    set_servo_angle(SERVO_CHANNEL, CENTER_ANGLE)
    GPIO.cleanup()
    print("\nStopping...")

finally:
    for s in sensors.values():
        try: s.stop_continuous()
        except Exception: pass
    for x in xshuts.values():
        x.value = False
    print("Cleaned up.")
