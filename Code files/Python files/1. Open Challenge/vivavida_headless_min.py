#!/usr/bin/env python3
"""
VivaLaVida WRO2025 - Headless Minimal Controller
------------------------------------------------
- Headless-only (no GUI, no plotting, no sliders)
- Ultrasonic sensors only (gpiozero.DistanceSensor)
- PCA9685 for motor & steering servo
- MPU6050 gyro for yaw-based turning
- Minimal FSM: CRUISE -> TURN_INIT -> TURNING -> POST_TURN -> (STOPPED/CRUISE)
- Start with physical START button, long-press (1.5s) START to stop
"""

import time
import threading
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
import busio
import board
from adafruit_pca9685 import PCA9685
import adafruit_mpu6050

# ===============================
# CONFIG
# ===============================

DEBUG = 1                     # 1 = prints enabled, 0 = silent
MAX_LAPS = 3                  # 0 = unlimited
SENSOR_DT = 0.01              # seconds between sensor updates
LOOP_DT = 0.01                # seconds between control iterations

# Speeds (0-100% duty, mapped to PCA9685 16-bit)
SPEED_CRUISE     = 25
SPEED_TURN_INIT  = 17
SPEED_TURN       = 20
SPEED_POST_TURN  = 20

# Safety / distances (cm)
SOFT_MARGIN          = 30     # start steering away from wall inside this
MAX_CORRECTION_DEG   = 7      # max steering correction at SOFT_MARGIN
STOP_THRESHOLD       = 20     # immediate stop if front < STOP_THRESHOLD
FRONT_TURN_TRIGGER   = 90     # begin TURN_INIT when front < FRONT_TURN_TRIGGER
TURN_DECISION_THRESH = 90     # side is "open" if > this value (or invalid)

# Yaw-based turn control
TARGET_TURN_ANGLE     = 85    # desired nominal corner angle
TURN_ANGLE_TOLERANCE  = 5     # acceptable error on yaw target
MIN_TURN_ANGLE        = 65    # clamp target min
MAX_TURN_ANGLE        = 120   # clamp target max
TURN_TIMEOUT          = 4.5   # seconds
POST_TURN_DURATION    = 0.5   # seconds
TURN_LOCKOUT          = 1.0   # min time between turns (s)

# Ultrasonic ranges (gpiozero expects meters)
US_MAX_DISTANCE_FRONT = 3.43  # m
US_MAX_DISTANCE_SIDE  = 1.72  # m
US_QUEUE_LEN          = 5

# Low-pass for sensors & gyro
EMA_ALPHA_SENSOR = 1.0        # 1.0 = no smoothing
MAX_JUMP_CM      = 999        # spike reject (cm); set low to be stricter
GYRO_ALPHA       = 0.8        # gyro LPF (0..1), higher = more smoothing

# Servo & motor (PCA9685 @50Hz)
SERVO_CHANNEL    = 0
SERVO_CENTER     = 88
SERVO_MIN_ANGLE  = 50
SERVO_MAX_ANGLE  = 130
SERVO_PULSE_MIN  = 1000       # us
SERVO_PULSE_MAX  = 2000       # us
SERVO_PERIOD_US  = 20000
SERVO_SLEW_DPS   = 0          # 0 = disabled (max slewing), else deg/s

MOTOR_FWD_CH     = 1
MOTOR_REV_CH     = 2

# Pins (BCM numbering)
START_BTN = 20
GREEN_LED = 19
RED_LED   = 13

TRIG_FRONT, ECHO_FRONT = 22, 23  # front
TRIG_LEFT,  ECHO_LEFT  = 27, 17  # left
TRIG_RIGHT, ECHO_RIGHT = 5,  6   # right

RAD2DEG = 57.29577951308232

def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

def norm180(a):
    """Normalize angle to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0

def snap90(a):
    """Nearest multiple of 90° (…,-180,-90,0,90,180,…)"""
    return round(a / 90.0) * 90.0

# ===============================
# HARDWARE INIT
# ===============================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(START_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GREEN_LED, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RED_LED,   GPIO.OUT, initial=GPIO.LOW)

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
mpu = adafruit_mpu6050.MPU6050(i2c)

# Distance sensors (ultrasonic only)
us_front = DistanceSensor(echo=ECHO_FRONT, trigger=TRIG_FRONT,
                          max_distance=US_MAX_DISTANCE_FRONT, queue_len=US_QUEUE_LEN)
us_left  = DistanceSensor(echo=ECHO_LEFT,  trigger=TRIG_LEFT,
                          max_distance=US_MAX_DISTANCE_SIDE,  queue_len=US_QUEUE_LEN)
us_right = DistanceSensor(echo=ECHO_RIGHT, trigger=TRIG_RIGHT,
                          max_distance=US_MAX_DISTANCE_SIDE,  queue_len=US_QUEUE_LEN)

# ===============================
# ROBOT CONTROL
# ===============================

class Robot:
    def __init__(self, pca):
        self.pca = pca
        self._servo_last_angle = SERVO_CENTER
        self._servo_last_ns = time.monotonic_ns()

    def motor(self, speed):
        """speed: -100..100; maps to MOTOR_FWD_CH/MOTOR_REV_CH duty_cycle"""
        dc = int(min(max(abs(speed), 0), 100) / 100 * 65535)
        if speed >= 0:
            self.pca.channels[MOTOR_FWD_CH].duty_cycle = dc
            self.pca.channels[MOTOR_REV_CH].duty_cycle = 0
        else:
            self.pca.channels[MOTOR_FWD_CH].duty_cycle = 0
            self.pca.channels[MOTOR_REV_CH].duty_cycle = dc

    def stop(self):
        self.pca.channels[MOTOR_FWD_CH].duty_cycle = 0
        self.pca.channels[MOTOR_REV_CH].duty_cycle = 0

    def set_servo(self, angle):
        # clamp
        target = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))

        # optional slew limiting
        now_ns = time.monotonic_ns()
        dt = max(0.0, (now_ns - self._servo_last_ns) * 1e-9)
        if SERVO_SLEW_DPS > 0 and dt > 0:
            max_step = SERVO_SLEW_DPS * dt
            diff = target - self._servo_last_angle
            if abs(diff) > max_step:
                target = self._servo_last_angle + (max_step if diff > 0 else -max_step)

        self._servo_last_angle = target
        self._servo_last_ns = now_ns

        # angle to PWM pulse (us) → 16-bit duty
        pulse = int(SERVO_PULSE_MIN + (SERVO_PULSE_MAX - SERVO_PULSE_MIN) *
                    ((target - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)))
        self.pca.channels[SERVO_CHANNEL].duty_cycle = int(pulse * 65535 / SERVO_PERIOD_US)
        return target

    # --- behaviors ---
    def cruise_servo(self, d_left, d_right):
        """Simple wall-following: steer away if within SOFT_MARGIN of either wall."""
        correction = 0.0
        if d_left  is not None and d_left  < SOFT_MARGIN:
            correction = (SOFT_MARGIN - d_left) * 2.0
        elif d_right is not None and d_right < SOFT_MARGIN:
            correction = -(SOFT_MARGIN - d_right) * 2.0

        if correction >  MAX_CORRECTION_DEG: correction =  MAX_CORRECTION_DEG
        if correction < -MAX_CORRECTION_DEG: correction = -MAX_CORRECTION_DEG
        return self.set_servo(SERVO_CENTER + correction)

robot = Robot(pca)

# ===============================
# SENSOR THREAD
# ===============================

sensor = {"front": None, "left": None, "right": None}
s_lock = threading.Lock()

def ema(prev, new, alpha, max_jump):
    if new is None:
        return prev
    if prev is not None and abs(new - prev) > max_jump:
        new = prev
    if prev is None:
        return new
    return alpha * new + (1-alpha) * prev

def _read_cm(ds):
    try:
        m = ds.distance  # 0..max (meters)
        if m is None:
            return None
        return m * 100.0
    except Exception:
        return None

def sensor_loop(stop_evt):
    front_s = left_s = right_s = None
    while not stop_evt.is_set():
        f = _read_cm(us_front)
        l = _read_cm(us_left)
        r = _read_cm(us_right)

        front_s = ema(front_s, f, EMA_ALPHA_SENSOR, MAX_JUMP_CM)
        left_s  = ema(left_s,  l, EMA_ALPHA_SENSOR, MAX_JUMP_CM)
        right_s = ema(right_s, r, EMA_ALPHA_SENSOR, MAX_JUMP_CM)

        with s_lock:
            sensor["front"] = front_s
            sensor["left"]  = left_s
            sensor["right"] = right_s

        time.sleep(SENSOR_DT)

# ===============================
# FSM LOOP
# ===============================

def turn_decision(d_left, d_right):
    left_open  = (d_left  is None) or (d_left  > TURN_DECISION_THRESH)
    right_open = (d_right is None) or (d_right > TURN_DECISION_THRESH)
    if left_open ^ right_open:
        return "LEFT" if left_open else "RIGHT"
    return None  # both open or both closed

def fsm_loop(stop_evt):
    # Gyro bias
    dprint("Calibrating gyro...")
    N = 500
    bias = 0.0
    for _ in range(N):
        bias += mpu.gyro[2]
        time.sleep(0.003)
    bias /= N

    yaw = 0.0
    gyro_prev = 0.0
    turn_count = 0
    lap_count = 0
    last_turn_time = -999.0
    state = "CRUISE"
    direction = None
    turn_start_yaw = 0.0
    turn_target_delta = 0.0
    turn_start_time = 0.0
    post_turn_start = 0.0

    btn_down_since = None

    # Start straight, stopped motors until loop begins
    robot.stop()
    robot.set_servo(SERVO_CENTER)

    last_ns = time.monotonic_ns()
    while not stop_evt.is_set():
        # --- time step ---
        now_ns = time.monotonic_ns()
        dt = (now_ns - last_ns) * 1e-9
        last_ns = now_ns
        t = now_ns * 1e-9

        # --- button long-press (1.5s) to stop ---
        if GPIO.input(START_BTN) == 0:  # pressed (active low)
            if btn_down_since is None:
                btn_down_since = t
            elif (t - btn_down_since) >= 1.5:
                dprint("Long-press detected: stopping.")
                stop_evt.set()
                break
        else:
            btn_down_since = None

        # --- read sensors snapshot ---
        with s_lock:
            d_front = sensor["front"]
            d_left  = sensor["left"]
            d_right = sensor["right"]

        # --- yaw integration (filtered gyro) ---
        raw_gz = mpu.gyro[2] - bias  # rad/s
        gz = GYRO_ALPHA * raw_gz + (1.0 - GYRO_ALPHA) * gyro_prev
        gyro_prev = gz
        yaw += gz * RAD2DEG * dt

        # --- FSM ---
        if state == "CRUISE":
            # emergency stop
            if d_front is not None and d_front < STOP_THRESHOLD:
                robot.stop()
                state = "STOPPED"
                obstacle_until = t + 5.0
                dprint("Stopped: obstacle ahead")
                time.sleep(LOOP_DT)
                continue

            # turn trigger
            if (d_front is not None and d_front < FRONT_TURN_TRIGGER and
                (t - last_turn_time) >= TURN_LOCKOUT):
                state = "TURN_INIT"
                robot.motor(SPEED_TURN_INIT)
                dprint("Turn INIT: waiting for open side")
                time.sleep(LOOP_DT)
                continue

            # normal cruise
            robot.motor(SPEED_CRUISE)
            robot.cruise_servo(d_left, d_right)

        elif state == "TURN_INIT":
            robot.motor(SPEED_TURN_INIT)
            robot.cruise_servo(d_left, d_right)  # gentle keep-straight

            # if front becomes safe again, abort
            if d_front is not None and d_front >= FRONT_TURN_TRIGGER:
                state = "CRUISE"
                dprint("Front cleared, back to CRUISE")
                time.sleep(LOOP_DT)
                continue

            proposed = turn_decision(d_left, d_right)
            if proposed is None:
                time.sleep(LOOP_DT)
                continue

            # commit
            direction = proposed
            entry_skew = norm180(yaw - snap90(yaw))   # + = left-leaning
            base = TARGET_TURN_ANGLE if direction == "LEFT" else -TARGET_TURN_ANGLE
            turn_target_delta = base - entry_skew
            if turn_target_delta >  MAX_TURN_ANGLE: turn_target_delta =  MAX_TURN_ANGLE
            if turn_target_delta < -MAX_TURN_ANGLE: turn_target_delta = -MAX_TURN_ANGLE
            if abs(turn_target_delta) < MIN_TURN_ANGLE:
                turn_target_delta = MIN_TURN_ANGLE if turn_target_delta >= 0 else -MIN_TURN_ANGLE

            turn_start_yaw  = yaw
            turn_start_time = t
            robot.set_servo(60 if direction == "LEFT" else 120)  # fixed wheel angle
            robot.motor(SPEED_TURN)
            state = "TURNING"
            dprint(f"TURNING {direction} | target Δ={turn_target_delta:.1f}° (entry skew {entry_skew:.1f}°)")

        elif state == "TURNING":
            robot.motor(SPEED_TURN)
            turn_angle = yaw - turn_start_yaw

            stop_cond = False
            if abs(turn_angle - turn_target_delta) <= TURN_ANGLE_TOLERANCE:
                stop_cond = True
                dprint("Turn: reached target angle")
            if (t - turn_start_time) > TURN_TIMEOUT:
                stop_cond = True
                dprint("Turn: timeout")
            if abs(turn_angle) >= MAX_TURN_ANGLE:
                stop_cond = True
                dprint("Turn: max angle reached")

            if stop_cond:
                robot.stop()
                robot.set_servo(SERVO_CENTER)
                yaw = snap90(yaw)  # snap to corridor axis
                last_turn_time = t
                turn_count += 1
                if turn_count % 4 == 0:
                    lap_count += 1
                    dprint(f"Lap completed: {lap_count}")
                    if MAX_LAPS > 0 and lap_count >= MAX_LAPS:
                        # small roll-out then stop
                        robot.motor(SPEED_POST_TURN)
                        robot.set_servo(SERVO_CENTER)
                        time.sleep(0.5)
                        stop_evt.set()
                        dprint("Max laps reached. Stopping.")
                        break

                post_turn_start = t
                state = "POST_TURN"

        elif state == "POST_TURN":
            robot.motor(SPEED_POST_TURN)
            robot.set_servo(SERVO_CENTER)
            if (t - post_turn_start) >= POST_TURN_DURATION:
                state = "CRUISE"

        elif state == "STOPPED":
            robot.stop()
            robot.set_servo(SERVO_CENTER)
            # auto-retry after 5s if front is clear
            if t >= obstacle_until:
                if d_front is None or d_front >= STOP_THRESHOLD:
                    dprint("Obstacle cleared. Resuming CRUISE.")
                    state = "CRUISE"
                else:
                    obstacle_until = t + 5.0
            time.sleep(LOOP_DT)
            continue

        time.sleep(LOOP_DT)

# ===============================
# MAIN
# ===============================

def main():
    # Status LEDs
    GPIO.output(RED_LED, GPIO.HIGH)
    GPIO.output(GREEN_LED, GPIO.LOW)

    # Wait for START press
    dprint("Headless: waiting for START button...")
    try:
        while GPIO.input(START_BTN) == 1:
            time.sleep(0.02)
    except KeyboardInterrupt:
        GPIO.cleanup()
        return

    dprint("START pressed. Launching...")
    GPIO.output(RED_LED, GPIO.LOW)
    GPIO.output(GREEN_LED, GPIO.HIGH)

    stop_evt = threading.Event()

    # Start sensor thread
    th = threading.Thread(target=sensor_loop, args=(stop_evt,), daemon=True)
    th.start()

    # Run FSM in main thread so Ctrl+C is responsive
    try:
        fsm_loop(stop_evt)
    except KeyboardInterrupt:
        dprint("Keyboard interrupt: stopping.")
        stop_evt.set()
    finally:
        robot.stop()
        robot.set_servo(SERVO_CENTER)
        GPIO.output(GREEN_LED, GPIO.LOW)
        GPIO.output(RED_LED, GPIO.HIGH)
        GPIO.cleanup()
        dprint("Cleaned up. Bye.")

if __name__ == "__main__":
    main()
