import RPi.GPIO as GPIO
import time
import smbus
from sensors.tof_sensors import TOFSensors
from sensors.ultrasonic import UltrasonicSensor
from sensors.mpu6050 import MPU6050
from control.motor_servo import MotorServoController

START_BTN = 21
STOP_BTN = 20
GREEN_LED = 19
RED_LED = 13
BUZZER = 4

GPIO.setmode(GPIO.BCM)

GPIO.setup(START_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(STOP_BTN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)

GPIO.output(GREEN_LED, GPIO.LOW)
GPIO.output(RED_LED, GPIO.LOW)
GPIO.output(BUZZER, GPIO.LOW)

print("Press button to start...")

while GPIO.input(START_BTN):
    time.sleep(0.5)

print("Program started!\n")

GPIO.output(GREEN_LED, GPIO.HIGH)
GPIO.output(BUZZER, GPIO.HIGH)
time.sleep(1.0)
GPIO.output(BUZZER, GPIO.LOW)

xshut_pins = [16, 26, 25, 24]
i2c_addresses = [0x30, 0x31, 0x32, 0x33]
tof = TOFSensors(xshut_pins, i2c_addresses)

front_ultra = UltrasonicSensor(trig_pin=22, echo_pin=23)
left_ultra = UltrasonicSensor(trig_pin=27, echo_pin=17)
right_ultra = UltrasonicSensor(trig_pin=5, echo_pin=6)

car = MotorServoController()

bus = smbus.SMBus(1)
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_SCALE_MODIFIER = 131.0
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

prev_time = time.time()
angle_x = 0.0
angle_y = 0.0
angle_z = 0.0

direction = "NONE"
turns = 0
print("all sensors and motors initialized.")

def endturn():
    if gyro_z > 90.0 or gyro_z < -90.0:
        turns += 1
        print(f"Turns: {turns}")
        GPIO.output(BUZZER, GPIO.HIGH)
        car.set_servo_angle(91)
        gyro_z = 0.0

try:
    running = True
    while running:
        tof_dist = tof.get_distances()
        front_tof = tof_dist["front"]
        left_tof = tof_dist["left"]
        right_tof = tof_dist["right"]
        back_tof = tof_dist["back"]

        front_ultra_d = front_ultra.get_distance()
        left_ultra_d = left_ultra.get_distance()
        right_ultra_d = right_ultra.get_distance()

        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = read_raw_data(GYRO_XOUT_H + 4)

        gx = gyro_x / GYRO_SCALE_MODIFIER
        gy = gyro_y / GYRO_SCALE_MODIFIER
        gz = gyro_z / GYRO_SCALE_MODIFIER

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        angle_x += gx * dt
        angle_y += gy * dt
        angle_z += gz * dt
        # Drift correction
        angle_z += 0.045

        print("------------------------------------------------")
        print(f"[TOF]   F:{front_tof} cm | L:{left_tof} cm | R:{right_tof} cm | B:{back_tof} cm")
        print(f"[ULTRA] F:{front_ultra_d} cm | L:{left_ultra_d} cm | R:{right_ultra_d} cm")
        print(f"[GYRO]  Rate X:{gx:.2f}°/s Y:{gy:.2f}°/s Z:{gz:.2f}°/s | Angle X:{angle_x:.2f}° Y:{angle_y:.2f}° Z:{angle_z:.2f}°")
        print("------------------------------------------------")

        if direction == "NONE":
            if front_tof > 30 and front_ultra_d > 30:
                car.set_servo_angle(91)
                car.set_motor_speed(10)
            else:  
                if left_tof > 100 and left_ultra_d >100:
                    direction = "LEFT"
                    car.set_servo_angle(55)
                    print("direction set to LEFT")
                elif right_tof > 100 and right_ultra_d > 100:  
                    direction = "RIGHT"
                    car.set_servo_angle(135)
                    print("direction set to RIGHT")

        if direction != "NONE":
            if turns == 0:
                if direction == "LEFT":
                    angle_z = 0.0
                    car.set_servo_angle(55)
                    car.set_motor_speed(10)
                    endturn()
                    
                elif direction == "RIGHT":
                    angle_z = 0.0
                    car.set_servo_angle(135)
                    car.set_motor_speed(10)
                    endturn()
            elif turns > 0 and turns < 12:
                if direction == "LEFT":
                    if left_tof < 100 and left_ultra_d < 100:
                        car.set_motor_speed(20)
                    else:
                        angle_z = 0.0
                        car.set_servo_angle(55)
                        car.set_motor_speed(10)
                        endturn()
                elif direction == "RIGHT":
                    if right_tof < 100 and right_ultra_d < 100:
                        car.set_motor_speed(20)
                    else:
                        angle_z = 0.0
                        car.set_servo_angle(135)
                        car.set_motor_speed(10)
                        endturn()
            elif turns >= 12:
                print("Completed 12 turns, stopping.")
                time.sleep(1)
                car.set_motor_speed(0)
                car.set_servo_angle(91)
                direction = "NONE"
            else:
                print("Error in turn logic.")
                car.set_motor_speed(0)
                car.set_servo_angle(91)
                direction = "NONE"


        if GPIO.input(STOP_BTN) == 0:
            print("Ending.")
            running = False


except KeyboardInterrupt:
    print("User interrupt.")

finally:
    GPIO.output(GREEN_LED, GPIO.LOW)
    GPIO.output(RED_LED, GPIO.HIGH)

    for _ in range(2):
        GPIO.output(BUZZER, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(0.2)

    car.cleanup()
    GPIO.cleanup()
    print("All GPIO cleaned up.\n")
