import RPi.GPIO as GPIO
import time
from sensors.tof_sensors import TOFSensors
from sensors.ultrasonic import UltrasonicSensor
from sensors.mpu6050 import MPU6050
from control.motor_servo import MotorServoController

# ------------------- ΡΥΘΜΙΣΗ ΠΙΝΩΝ -------------------
START_BTN = 21
STOP_BTN = 20
GREEN_LED = 19
RED_LED = 13
BUZZER = 4

GPIO.setmode(GPIO.BCM)

# Κουμπιά
GPIO.setup(START_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(STOP_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# LED & Buzzer
GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)

# Αρχικά όλα κλειστά
GPIO.output(GREEN_LED, GPIO.LOW)
GPIO.output(RED_LED, GPIO.LOW)
GPIO.output(BUZZER, GPIO.LOW)

print("🔘 Πάτησε το κουμπί στο GPIO25 για να ξεκινήσει...")

# Περιμένει μέχρι να πατηθεί το κουμπί (λογικό LOW)
while GPIO.input(START_BTN):
    time.sleep(0.5)

# ------------------- ΕΝΑΡΞΗ ΠΡΟΓΡΑΜΜΑΤΟΣ -------------------
print("🚀 Πρόγραμμα ξεκίνησε!\n")

# LED Πράσινο ON και beep
GPIO.output(GREEN_LED, GPIO.HIGH)
GPIO.output(BUZZER, GPIO.HIGH)
time.sleep(1.0)
GPIO.output(BUZZER, GPIO.LOW)

# ------------------- ΡΥΘΜΙΣΗ ΣΥΣΤΗΜΑΤΟΣ -------------------
xshut_pins = [16, 26, 25, 24]
i2c_addresses = [0x30, 0x31, 0x32, 0x33]
tof = TOFSensors(xshut_pins, i2c_addresses)

front_ultra = UltrasonicSensor(trig_pin=22, echo_pin=23)
left_ultra = UltrasonicSensor(trig_pin=27, echo_pin=17)
right_ultra = UltrasonicSensor(trig_pin=5, echo_pin=6)

car = MotorServoController()
mpu = MPU6050()

print("✅ Όλα τα συστήματα ενεργοποιήθηκαν!\n")

# ------------------- ΚΥΡΙΟΣ ΒΡΟΧΟΣ -------------------
try:
    running = True
    while running:
        # --- TOF ---
        tof_dist = tof.get_distances()
        front_tof = tof_dist["front"]
        left_tof = tof_dist["left"]
        right_tof = tof_dist["right"]
        back_tof = tof_dist["back"]

        # --- Ultrasonic ---
        front_ultra_d = front_ultra.get_distance()
        left_ultra_d = left_ultra.get_distance()
        right_ultra_d = right_ultra.get_distance()

        # --- MPU6050 ---
        mpu_data = mpu.get_accel_gyro()
        ax, ay, az = mpu_data["accel"]
        gx, gy, gz = mpu_data["gyro"]

        # --- Εμφάνιση όλων ---
        print("------------------------------------------------")
        print(f"[TOF]   F:{front_tof} cm | L:{left_tof} cm | R:{right_tof} cm | B:{back_tof} cm")
        print(f"[ULTRA] F:{front_ultra_d} cm | L:{left_ultra_d} cm | R:{right_ultra_d} cm")
        print(f"[MPU]   Accel(X,Y,Z): {ax},{ay},{az} | Gyro(X,Y,Z): {gx},{gy},{gz}")
        print("------------------------------------------------")

        # --- Παράδειγμα κίνησης ---
       # if front_ultra_d is not None and front_ultra_d > 20:
        car.set_servo_angle(90)
           # car.set_motor_speed(40)
        #else:
           # car.stop_motor()
           # car.set_servo_angle(120)
           # time.sleep(0.8)
           # car.set_servo_angle(90)

        # Έλεγχος κουμπιού STOP
        if GPIO.input(STOP_BTN) == 0:
            print("\n🛑 Πάτησες το STOP (GPIO20) — τερματισμός.")
            running = False

        

except KeyboardInterrupt:
    print("\n⏹️ Τερματισμός από χρήστη.")

finally:
    # LED RED ON, GREEN OFF
    GPIO.output(GREEN_LED, GPIO.LOW)
    GPIO.output(RED_LED, GPIO.HIGH)

    # Buzzer beep 2 φορές
    for _ in range(2):
        GPIO.output(BUZZER, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(0.3)

    car.cleanup()
    GPIO.cleanup()
    print("🔁 Όλα τα GPIO καθαρίστηκαν.\n")
