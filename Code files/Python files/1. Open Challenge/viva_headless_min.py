
# -------------------------------------------------------------------------------------------------
# VivaLaVida WRO 2025 - Headless Minimal Controller
# Keeps only the features you selected (see checklist mapping in comments).
# Raspberry Pi 5 compatible.
# -------------------------------------------------------------------------------------------------

import time
import threading
from threading import Event
from collections import deque

import RPi.GPIO as GPIO
import board
import busio
import digitalio

from gpiozero import DistanceSensor
from adafruit_pca9685 import PCA9685
import adafruit_mpu6050
import adafruit_vl53l0x

# ===============================
# CONFIGURATION (1.2, 1.3)
# ===============================

DEBUG = 1  # (1.2) global prints toggle

# Speeds etc. (1.3)
SPEED_IDLE = 0
SPEED_CRUISE = 25
SPEED_TURN_INIT = 17
SPEED_TURN = 20
SPEED_POST_TURN = 20

SOFT_MARGIN = 30           # (8.1)
MAX_CORRECTION = 7         # (8.1)
CORRECTION_DURATION = 0.25 # (8.2)

STOP_THRESHOLD = 20        # (11.1)
OBSTACLE_WAIT_TIME = 5.0   # (11.2)

FRONT_TURN_TRIGGER = 90    # (9.1)
TURN_DECISION_THRESHOLD = 90 # (9.2)
TURN_ANGLE_LEFT = 60       # (turn command angle)
TURN_ANGLE_RIGHT = 120
FRONT_SAFE_DISTANCE = 160  # (10.2 - optional context)
SIDE_SAFE_DISTANCE = 30    # (10.2 - optional context)
TURN_TIMEOUT = 4.5         # (10.2)
TURN_LOCKOUT = 1.0         # (9.4)
POST_TURN_DURATION = 0.5   # (10.3)
LOCK_TURN_DIRECTION = 1    # (9.3)
TARGET_TURN_ANGLE = 85     # (10.1)
TURN_ANGLE_TOLERANCE = 5   # (10.2)

MIN_TURN_ANGLE = 65        # (10.2 bounds)
MAX_TURN_ANGLE = 120       # (10.2 bounds)

MAX_LAPS = 3               # (12.1)
POST_LAP_DURATION = 1.0    # (12.1)

# Narrow corridor (7.1)
NARROW_SUM_THRESHOLD = 60
NARROW_HYSTERESIS = 10
NARROW_FACTOR_SPEED = 1.0
NARROW_FACTOR_DIST  = 1.0

# Filtering (4.1, 4.2, 4.3)
N_READINGS = 3
FILTER_ALPHA_US = 0.6
FILTER_JUMP_US = 60
FILTER_ALPHA_TOF = 0.6
FILTER_JUMP_TOF = 60

US_QUEUE_LEN = 5
US_MAX_DISTANCE_FRONT = 3.43  # meters
US_MAX_DISTANCE_SIDE = 1.72   # meters

# Loop timing (13.2)
LOOP_DELAY = 0.003
SENSOR_DELAY = 0.003

RAD2DEG = 57.29577951308232

# Sensor choice (3.1, 3.2). Set according to your rig.
USE_TOF_SIDES = 0
USE_TOF_FRONT = 0

# ===============================
# GPIO PINS (2.1, 2.2)
# ===============================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

START_BTN = 20
GPIO.setup(START_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GREEN_LED = 19
RED_LED   = 13
GPIO.setup(GREEN_LED, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RED_LED,   GPIO.OUT, initial=GPIO.LOW)

# Ultrasonic pins (3.1)
TRIG_FRONT, ECHO_FRONT = 22, 23
TRIG_LEFT,  ECHO_LEFT  = 27, 17
TRIG_RIGHT, ECHO_RIGHT = 5, 6

# ===============================
# I2C + Devices (2.3, 2.4, 3.2)
# ===============================

i2c = busio.I2C(board.SCL, board.SDA)

# PCA9685 (2.3)
pca = PCA9685(i2c)
pca.frequency = 50

# Servo & Motor channels
SERVO_CHANNEL = 0
SERVO_CENTER = 88
SERVO_MIN_ANGLE = 50
SERVO_MAX_ANGLE = 130
SERVO_PULSE_MIN = 1000  # µs
SERVO_PULSE_MAX = 2000  # µs
SERVO_PERIOD   = 20000  # µs

MOTOR_FWD = 1
MOTOR_REV = 2

# IMU (2.4)
mpu = adafruit_mpu6050.MPU6050(i2c)

# ToF (3.2) — optional; only init those requested
vl53_left = vl53_right = vl53_front = vl53_back = None

def init_tof_if_requested():
    global vl53_left, vl53_right, vl53_front, vl53_back
    if not (USE_TOF_SIDES or USE_TOF_FRONT):
        return
    try:
        xshut_left  = digitalio.DigitalInOut(board.D16)
        xshut_right = digitalio.DigitalInOut(board.D25)
        xshut_front = digitalio.DigitalInOut(board.D26)
        xshut_back  = digitalio.DigitalInOut(board.D24)
        for x in (xshut_left, xshut_right, xshut_front, xshut_back):
            x.direction = digitalio.Direction.OUTPUT
            x.value = False
        time.sleep(0.05)

        if USE_TOF_SIDES:
            xshut_left.value = True;  time.sleep(0.02)
            vl53_left = adafruit_vl53l0x.VL53L0X(i2c); vl53_left.set_address(0x30)
            vl53_left.measurement_timing_budget = 20000; vl53_left.start_continuous()

            xshut_right.value = True; time.sleep(0.02)
            vl53_right = adafruit_vl53l0x.VL53L0X(i2c); vl53_right.set_address(0x31)
            vl53_right.measurement_timing_budget = 20000; vl53_right.start_continuous()

        if USE_TOF_FRONT:
            xshut_front.value = True; time.sleep(0.02)
            vl53_front = adafruit_vl53l0x.VL53L0X(i2c); vl53_front.set_address(0x32)
            vl53_front.measurement_timing_budget = 20000; vl53_front.start_continuous()

        # Optional: back ToF kept available
        xshut_back.value = True; time.sleep(0.02)
        vl53_back = adafruit_vl53l0x.VL53L0X(i2c); vl53_back.set_address(0x33)
        vl53_back.measurement_timing_budget = 20000; vl53_back.start_continuous()

        dprint("ToF initialized.")
    except Exception as e:
        dprint(f"[WARN] ToF init failed: {e}")
        # fall back to ultrasonics only
        vl53_left = vl53_right = vl53_front = vl53_back = None

# Ultrasonic (3.1)
us_front = us_left = us_right = None
def init_ultrasonic():
    global us_front, us_left, us_right
    if not USE_TOF_FRONT:
        us_front = DistanceSensor(echo=ECHO_FRONT, trigger=TRIG_FRONT,
                                  max_distance=US_MAX_DISTANCE_FRONT, queue_len=US_QUEUE_LEN)
    if not USE_TOF_SIDES:
        us_left  = DistanceSensor(echo=ECHO_LEFT,  trigger=TRIG_LEFT,
                                  max_distance=US_MAX_DISTANCE_SIDE, queue_len=US_QUEUE_LEN)
        us_right = DistanceSensor(echo=ECHO_RIGHT, trigger=TRIG_RIGHT,
                                  max_distance=US_MAX_DISTANCE_SIDE, queue_len=US_QUEUE_LEN)

# ===============================
# Helpers (5.3) + Debug
# ===============================

def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def norm180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0

def snap90(a: float) -> float:
    return round(a / 90.0) * 90.0

# ===============================
# State / Flags (13.1)
# ===============================

readings_event = Event()
loop_event = Event()
sensor_tick = Event()

turn_count = 0
lap_count = 0
stop_reason = None

# Env factors (7.1)
SPEED_ENV_FACTOR = 1.0
DIST_ENV_FACTOR  = 1.0

# ===============================
# Controller (6.2, 4.x, 8.x, 9.x)
# ===============================

class RobotController:
    def __init__(self, pca):
        self.pca = pca
        self.front_history = deque(maxlen=N_READINGS)
        self.left_history  = deque(maxlen=N_READINGS)
        self.right_history = deque(maxlen=N_READINGS)
        self.smooth_front = None
        self.smooth_left  = None
        self.smooth_right = None
        self.gyro_z_prev = 0.0

        # latest filtered distances (cm)
        self.d_front = None
        self.d_left  = None
        self.d_right = None

    # --- Actuators (6.2) ---
    def set_servo(self, angle):
        # clamp
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        pulse = int(SERVO_PULSE_MIN + (SERVO_PULSE_MAX - SERVO_PULSE_MIN) *
                    ((angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)))
        self.pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / SERVO_PERIOD)

    def rotate_motor(self, speed_percent):
        dc = int(min(max(abs(speed_percent), 0), 100) / 100.0 * 65535)
        if speed_percent >= 0:
            self.pca.channels[MOTOR_FWD].duty_cycle = dc
            self.pca.channels[MOTOR_REV].duty_cycle = 0
        else:
            self.pca.channels[MOTOR_FWD].duty_cycle = 0
            self.pca.channels[MOTOR_REV].duty_cycle = dc

    def stop_motor(self):
        self.pca.channels[MOTOR_FWD].duty_cycle = 0
        self.pca.channels[MOTOR_REV].duty_cycle = 0

    # --- Filtering (4.1, 4.2, 4.3) ---
    @staticmethod
    def _ema(prev, new, alpha):
        if prev is None: return new
        return alpha * new + (1 - alpha) * prev

    @staticmethod
    def _median(values):
        vals = [v for v in values if v is not None]
        if not vals: return None
        vals.sort()
        n = len(vals)
        mid = n // 2
        if n % 2 == 1:
            return vals[mid]
        else:
            return 0.5 * (vals[mid - 1] + vals[mid])

    def _stable_filter(self, new_val, prev_val, alpha, max_jump):
        if new_val is None:
            return prev_val
        if prev_val is not None and abs(new_val - prev_val) > max_jump:
            new_val = prev_val
        return self._ema(prev_val, new_val, alpha)

    def filtered_distance(self, sensor_obj, history, smooth_attr, sensor_type='us'):
        try:
            if sensor_type == 'tof':
                d = sensor_obj.range / 10.0  # mm -> cm
            else:
                dm = sensor_obj.distance      # meters
                d = dm * 100.0
        except Exception:
            d = None

        history.append(d)
        med = self._median(history)
        prev = getattr(self, smooth_attr)
        alpha = FILTER_ALPHA_TOF if sensor_type == 'tof' else FILTER_ALPHA_US
        max_jump = FILTER_JUMP_TOF if sensor_type == 'tof' else FILTER_JUMP_US
        smoothed = self._stable_filter(med, prev, alpha, max_jump)
        setattr(self, smooth_attr, smoothed)
        return smoothed

    # --- Wall following (8.1, 8.2) ---
    def eff_soft_margin(self):
        return int(SOFT_MARGIN * DIST_ENV_FACTOR)

    def eff_max_correction(self):
        return max(1, int(MAX_CORRECTION * DIST_ENV_FACTOR))

    def safe_straight_control(self, d_left, d_right):
        correction = 0
        if d_left is not None and d_left < self.eff_soft_margin():
            correction = (self.eff_soft_margin() - d_left) * 2
        elif d_right is not None and d_right < self.eff_soft_margin():
            correction = -(self.eff_soft_margin() - d_right) * 2
        correction = max(-self.eff_max_correction(), min(self.eff_max_correction(), correction))
        angle = SERVO_CENTER + correction
        return max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))

    # --- Turn decision (9.2) ---
    @staticmethod
    def turn_decision(d_left, d_right):
        left_open  = (d_left  is None) or (d_left  > TURN_DECISION_THRESHOLD)
        right_open = (d_right is None) or (d_right > TURN_DECISION_THRESHOLD)
        if left_open ^ right_open:
            return "LEFT" if left_open else "RIGHT"
        return None

robot = RobotController(pca)

# ===============================
# Sensor thread (3.3)
# ===============================

sensor_data = {"front": None, "left": None, "right": None}
sensor_lock = threading.Lock()

def sensor_reader():
    while True:
        # Left
        if vl53_left:
            left = robot.filtered_distance(vl53_left, robot.left_history, "smooth_left", "tof")
        elif us_left:
            left = robot.filtered_distance(us_left, robot.left_history, "smooth_left", "us")
        else:
            left = None
        # Right
        if vl53_right:
            right = robot.filtered_distance(vl53_right, robot.right_history, "smooth_right", "tof")
        elif us_right:
            right = robot.filtered_distance(us_right, robot.right_history, "smooth_right", "us")
        else:
            right = None
        # Front
        if vl53_front:
            front = robot.filtered_distance(vl53_front, robot.front_history, "smooth_front", "tof")
        elif us_front:
            front = robot.filtered_distance(us_front, robot.front_history, "smooth_front", "us")
        else:
            front = None

        with sensor_lock:
            sensor_data["left"] = left
            sensor_data["right"] = right
            sensor_data["front"] = front

        sensor_tick.wait(SENSOR_DELAY)
        sensor_tick.clear()

# ===============================
# FSM loop (9.x, 10.x, 11.x, 12.1, 13.2)
# ===============================

class State:
    IDLE = "IDLE"
    CRUISE = "CRUISE"
    TURN_INIT = "TURN_INIT"
    TURNING = "TURNING"
    POST_TURN = "POST_TURN"
    STOPPED = "STOPPED"

locked_turn_direction = None

def robot_loop():
    global turn_count, lap_count, stop_reason, locked_turn_direction
    state = State.IDLE
    yaw = 0.0
    last_turn_time = -999.0

    # Gyro bias (5.1)
    dprint("Calibrating gyro bias...")
    bias = 0.0; N = 500
    for _ in range(N):
        bias += mpu.gyro[2]
        time.sleep(0.002)
    bias /= N
    dprint(f"Gyro bias: {bias:.6f} rad/s")

    robot.stop_motor()
    robot.set_servo(SERVO_CENTER)

    # Correction timing (8.2)
    correction_active = False
    correction_start_time = 0.0

    # Turn bookkeeping (10.1-10.3)
    turn_start_yaw = 0.0
    turn_start_time = 0.0
    turn_target_delta = 0.0
    direction = None

    # For timing (13.2)
    last_ns = time.monotonic_ns()

    # Narrow corridor memory (7.1)
    narrow_mode = False

    while True:
        now_ns = time.monotonic_ns()
        dt = (now_ns - last_ns) * 1e-9
        last_ns = now_ns
        now = now_ns * 1e-9

        if not readings_event.is_set():
            robot.stop_motor()
            state = State.IDLE
            sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()
            continue

        # Read sensors (thread-safe)
        with sensor_lock:
            d_front = sensor_data["front"]
            d_left  = sensor_data["left"]
            d_right = sensor_data["right"]
        robot.d_front, robot.d_left, robot.d_right = d_front, d_left, d_right

        # Narrow corridor update (7.1)
        l = d_left  if (d_left  is not None) else None
        r = d_right if (d_right is not None) else None
        sum_lr = None if (l is None or r is None) else (l + r)
        enter_t = NARROW_SUM_THRESHOLD
        exit_t  = NARROW_SUM_THRESHOLD + NARROW_HYSTERESIS
        prev_mode = narrow_mode
        if sum_lr is not None:
            if not narrow_mode and sum_lr < enter_t:
                narrow_mode = True
            elif narrow_mode and sum_lr > exit_t:
                narrow_mode = False
        # Apply env scaling
        global SPEED_ENV_FACTOR, DIST_ENV_FACTOR
        SPEED_ENV_FACTOR = NARROW_FACTOR_SPEED if narrow_mode else 1.0
        DIST_ENV_FACTOR  = NARROW_FACTOR_DIST  if narrow_mode else 1.0
        if prev_mode != narrow_mode:
            dprint(f"[corridor] {'ON' if narrow_mode else 'OFF'} (L+R={sum_lr:.1f} cm)" if sum_lr is not None else f"[corridor] {'ON' if narrow_mode else 'OFF'}")

        # Gyro integration (5.2)
        raw_z = mpu.gyro[2] - bias  # rad/s
        ALPHA = 0.8
        filt_z = ALPHA * raw_z + (1 - ALPHA) * robot.gyro_z_prev
        robot.gyro_z_prev = filt_z
        yaw += (filt_z * RAD2DEG) * dt  # degrees

        # FSM
        if not loop_event.is_set():
            robot.stop_motor()
            state = State.IDLE
            sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()
            continue

        if state == State.IDLE:
            robot.stop_motor()
            robot.set_servo(SERVO_CENTER)
            correction_active = False
            state = State.CRUISE
            # soft start to cruise
            for s in range(0, SPEED_CRUISE + 1, 2):
                if not loop_event.is_set(): break
                robot.rotate_motor(s)
                sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()

        elif state == State.CRUISE:
            # Emergency stop (11.1)
            if d_front is not None and d_front < int(STOP_THRESHOLD * DIST_ENV_FACTOR):
                robot.stop_motor()
                robot.set_servo(SERVO_CENTER)
                dprint("Stop: obstacle ahead")
                state = State.STOPPED
                global stop_reason
                stop_reason = "OBSTACLE"
                obstacle_wait_deadline = now + OBSTACLE_WAIT_TIME

                # inner loop: handle obstacle wait
                while state == State.STOPPED and stop_reason == "OBSTACLE":
                    remaining = max(0.0, obstacle_wait_deadline - now)
                    # re-check on deadline
                    if now >= obstacle_wait_deadline:
                        if robot.d_front is not None and robot.d_front >= STOP_THRESHOLD:
                            dprint("Obstacle cleared — resuming")
                            stop_reason = None
                            state = State.CRUISE
                            robot.rotate_motor(SPEED_CRUISE)
                            break
                        else:
                            obstacle_wait_deadline = now + OBSTACLE_WAIT_TIME
                    sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()
                    now = time.monotonic_ns() * 1e-9
                continue

            # Turn trigger (9.1) + lockout (9.4)
            trig = (d_front is not None and d_front < int(FRONT_TURN_TRIGGER * DIST_ENV_FACTOR))
            if trig and (now - last_turn_time >= TURN_LOCKOUT):
                state = State.TURN_INIT
                dprint("Approaching turn — waiting for open side")
                continue

            # Straight control (8.1, 8.2)
            desired = robot.safe_straight_control(d_left, d_right)
            if (d_left is not None and d_left < robot.eff_soft_margin()) or \
               (d_right is not None and d_right < robot.eff_soft_margin()):
                if not correction_active:
                    correction_active = True
                    correction_start_time = now
                    dprint(f"Correction start: servo {desired:.1f}")
            if correction_active:
                if now - correction_start_time >= CORRECTION_DURATION:
                    correction_active = False
                    desired = SERVO_CENTER
                    dprint("Correction end")
            robot.set_servo(desired)
            robot.rotate_motor(int(SPEED_CRUISE * SPEED_ENV_FACTOR))

        elif state == State.TURN_INIT:
            robot.rotate_motor(int(SPEED_TURN_INIT * SPEED_ENV_FACTOR))
            # If front becomes safe again, abort turn
            if d_front is not None and d_front >= int(FRONT_TURN_TRIGGER * DIST_ENV_FACTOR):
                robot.set_servo(SERVO_CENTER)
                state = State.CRUISE
                continue

            # keep centered (or mild follow); we apply centered here
            robot.set_servo(SERVO_CENTER)

            proposed = RobotController.turn_decision(d_left, d_right)  # (9.2)
            if proposed is None:
                sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()
                continue

            # Direction lock (9.3)
            global locked_turn_direction
            if LOCK_TURN_DIRECTION == 1:
                if locked_turn_direction is None:
                    locked_turn_direction = proposed
                    dprint(f"Turn direction locked: {locked_turn_direction}")
                elif proposed != locked_turn_direction:
                    # wait for the locked side
                    sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()
                    continue
                direction = locked_turn_direction
            else:
                direction = proposed

            # Commit turn (10.1)
            dprint(f"Turn INIT -> {direction}")
            turn_start_yaw = yaw
            turn_start_time = now
            entry_skew = norm180(yaw - snap90(yaw))  # + means already rotated left
            base = TARGET_TURN_ANGLE if direction == "LEFT" else -TARGET_TURN_ANGLE
            turn_target_delta = base - entry_skew
            turn_target_delta = max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, turn_target_delta))
            if abs(turn_target_delta) < MIN_TURN_ANGLE:
                turn_target_delta = MIN_TURN_ANGLE if turn_target_delta >= 0 else -MIN_TURN_ANGLE

            angle = TURN_ANGLE_LEFT if direction == "LEFT" else TURN_ANGLE_RIGHT
            robot.set_servo(angle)
            robot.rotate_motor(int(SPEED_TURN * SPEED_ENV_FACTOR))
            state = State.TURNING

        elif state == State.TURNING:
            turn_angle = yaw - turn_start_yaw
            stop = False
            if abs(turn_angle - turn_target_delta) <= TURN_ANGLE_TOLERANCE:
                dprint("Stop Turn: target angle reached")
                stop = True
            if now - turn_start_time > TURN_TIMEOUT:
                dprint("Stop Turn: timeout")
                stop = True
            if abs(turn_angle) >= MAX_TURN_ANGLE:
                dprint("Stop Turn: max angle bound")
                stop = True

            if stop:
                robot.stop_motor()
                robot.set_servo(SERVO_CENTER)
                last_turn_time = now
                # Snap to axis to avoid drift
                yaw = snap90(yaw)
                turn_count += 1
                if turn_count % 4 == 0:
                    lap_count += 1
                    dprint(f"Lap {lap_count} completed")
                    if MAX_LAPS > 0 and lap_count >= MAX_LAPS:
                        robot.set_servo(SERVO_CENTER)
                        robot.rotate_motor(int(SPEED_POST_TURN * SPEED_ENV_FACTOR))
                        time.sleep(POST_LAP_DURATION)
                        robot.stop_motor()
                        stop_reason = "LAPS"
                        loop_event.clear()
                        state = State.STOPPED
                        continue
                state = State.POST_TURN
                post_turn_start = now
            else:
                robot.rotate_motor(int(SPEED_TURN * SPEED_ENV_FACTOR))

        elif state == State.POST_TURN:
            if (time.monotonic_ns() * 1e-9) - post_turn_start < POST_TURN_DURATION:
                robot.set_servo(SERVO_CENTER)
                robot.rotate_motor(int(SPEED_POST_TURN * SPEED_ENV_FACTOR))
            else:
                state = State.CRUISE

        sensor_tick.wait(LOOP_DELAY); sensor_tick.clear()

# ===============================
# MAIN (14.1, 14.2, 14.3)
# ===============================

def main():
    try:
        # LEDs ready (14.1)
        GPIO.output(RED_LED, GPIO.HIGH)
        GPIO.output(GREEN_LED, GPIO.LOW)

        init_tof_if_requested()
        init_ultrasonic()

        # Start sensor + loop threads
        t = threading.Thread(target=sensor_reader, daemon=True)
        t.start()
        t2 = threading.Thread(target=robot_loop, daemon=True)
        t2.start()

        # Wait for start button (14.2)
        dprint("Headless: waiting for START button...")
        GPIO.output(RED_LED, GPIO.LOW)
        GPIO.output(GREEN_LED, GPIO.HIGH)
        while GPIO.input(START_BTN) == 1:
            time.sleep(0.05)

        dprint("START pressed — beginning")
        readings_event.set()
        time.sleep(1.0)
        loop_event.set()

        # Keep main thread alive
        while t.is_alive() and t2.is_alive():
            time.sleep(0.5)

    except KeyboardInterrupt:
        dprint("KeyboardInterrupt — stopping (14.3)")
    finally:
        try:
            # stop ToF if running
            for sensor in (vl53_left, vl53_right, vl53_front, vl53_back):
                if sensor is not None:
                    try: sensor.stop_continuous()
                    except Exception: pass
        finally:
            robot.stop_motor()
            robot.set_servo(SERVO_CENTER)
            GPIO.cleanup()

if __name__ == "__main__":
    main()
