from gpiozero import Button, LED, Buzzer
import time
from sensors.tof_sensors import TOFSensors
from sensors.ultrasonic import UltrasonicSensor
from sensors.mpu6050 import MPU6050
from control.motor_servo import MotorServoController

# ------------------- ΡΥΘΜΙΣΗ ΠΙΝΩΝ -------------------
START_BTN = 20
STOP_BTN = 21
GREEN_LED = 19
RED_LED = 13
BUZZER_PIN = 4

# Αντικατάσταση των RPi.GPIO αντικειμένων με gpiozero
start_button = Button(START_BTN, pull_up=True)
stop_button = Button(STOP_BTN, pull_up=True)
green_led = LED(GREEN_LED)
red_led = LED(RED_LED)
buzzer = Buzzer(BUZZER_PIN)

# Αρχικά όλα κλειστά
green_led.off()
red_led.off()
buzzer.off()

print("🔘 Πάτησε το κουμπί στο GPIO21 για να ξεκινήσει...")

# Περιμένει μέχρι να πατηθεί το κουμπί έναρξης
start_button.wait_for_press()

# ------------------- ΕΝΑΡΞΗ ΠΡΟΓΡΑΜΜΑΤΟΣ -------------------
print("🚀 Πρόγραμμα ξεκίνησε!\n")

# LED Πράσινο ON και beep
green_led.on()
buzzer.on()
time.sleep(1.0)
buzzer.off()

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
        # else:
        #     car.stop_motor()
        #     car.set_servo_angle(120)
        #     time.sleep(0.8)
        #     car.set_servo_angle(90)

        # Έλεγχος κουμπιού STOP
        if stop_button.is_pressed:
            print("\n🛑 Πάτησες το STOP (GPIO20) — τερματισμός.")
            running = False

        time.sleep(0.3)

except KeyboardInterrupt:
    print("\n⏹️ Τερματισμός από χρήστη.")

finally:
    # LED RED ON, GREEN OFF
    green_led.off()
    red_led.on()

    # Buzzer beep 2 φορές
    for _ in range(2):
        buzzer.on()
        time.sleep(0.3)
        buzzer.off()
        time.sleep(0.3)

    car.cleanup()
    print("🔁 Όλα τα GPIO καθαρίστηκαν.\n")

