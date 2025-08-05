import time
from pca_motor_servo import PCAController, Servo, DCMotor

# Setup
pca = PCAController()
servo = Servo(pca, channel=0)
motor = DCMotor(pca, in1=1, in2=2)

try:
    # Servo test
    for angle in [0, 90, 180, 90]:
        print(f"Servo to {angle}°")
        servo.set_angle(angle)
        time.sleep(1)

    # Motor test
    print("Motor forward 80%")
    motor.run(80)
    time.sleep(2)

    print("Motor reverse 60%")
    motor.run(-60)
    time.sleep(2)

    print("Motor stop")
    motor.run(0)

finally:
    pca.cleanup()
