# Constants
STEERING_CENTER = 48
STEERING_LEFT = 5
STEERING_RIGHT = 165

TURN_ADJUST_LEFT = 80
TURN_ADJUST_RIGHT = 100

THROTTLE = 50
TOF_THRESHOLD = 70 
ALIGN_THRESHOLD = 7

FRONT_DISTANCE_THRESHOLD = 100

def stop_motor():
    pca9685_module.set_motor(1, 2, 50)  # Stop logic

# Start the motor forward at constant throttle
pca9685_module.set_motor(1, 2, THROTTLE)

try:
    # Main loop
    while True:
        d1 = sensor_left.range / 10   # Left
        d2 = sensor_right.range / 10  # Right
        d3 = sensor_front.range / 10  # Front
        d4 = sensor_back.range / 10   # Back

        print(f"Left: {d1:.1f} cm, Right: {d2:.1f} cm, Front: {d3:.1f} cm, Back: {d4:.1f} cm")

        # Alignment phase
        if d1 > d2:
            print("Adjusting right until right ≤ 7cm")
            while sensor_right.range / 10 > ALIGN_THRESHOLD:
                d1 = sensor_left.range / 10
                d2 = sensor_right.range / 10
                d3 = sensor_front.range / 10

                if d3 < FRONT_DISTANCE_THRESHOLD:
                    print("Front obstacle detected during alignment – breaking")
                    break

                pca9685_module.set_servo_angle(0, TURN_ADJUST_LEFT)
                time.sleep(0.05)
            continue

        elif d2 > d1:
            print("Adjusting left until left ≤ 7cm")
            while sensor_left.range / 10 > ALIGN_THRESHOLD:
                d1 = sensor_left.range / 10
                d2 = sensor_right.range / 10
                d3 = sensor_front.range / 10

                if d3 < FRONT_DISTANCE_THRESHOLD:
                    print("Front obstacle detected during alignment – breaking")
                    break

                pca9685_module.set_servo_angle(0, TURN_ADJUST_RIGHT)
                time.sleep(0.05)
            continue

        # Front obstacle handling
        if d3 < FRONT_DISTANCE_THRESHOLD:
            if d2 > TOF_THRESHOLD:
                print("Turning hard right (front block + space on right)")
                pca9685_module.set_servo_angle(0, STEERING_RIGHT)
            elif d1 > TOF_THRESHOLD:
                print("Turning hard left (front block + space on left)")
                pca9685_module.set_servo_angle(0, STEERING_LEFT)
            else:
                print("Obstacle ahead, but no side is clear")
                stop_motor()
            time.sleep(0.1)
            continue

        # Default straight
        print("Going straight")
        pca9685_module.set_servo_angle(0, STEERING_CENTER)
        time.sleep(0.1)

finally:
    stop_motor()

