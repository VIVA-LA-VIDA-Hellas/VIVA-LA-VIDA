import time
from ultrasonic import UltrasonicSensor

sensor = UltrasonicSensor(trig_pin=23, echo_pin=24)

try:
    while True:
        distance = sensor.get_distance(samples=5)
        if distance != -1:
            print(f"Distance: {distance} cm")
        else:
            print("Sensor timeout")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    sensor.cleanup()
