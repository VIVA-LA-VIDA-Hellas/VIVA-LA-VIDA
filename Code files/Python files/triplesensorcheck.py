import adafruit_hcsr04
import board
import time
import adafruit_tcs34725

front_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D17, echo_pin=board.D27)
left_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D5, echo_pin=board.D6)
right_sensor = adafruit_hcsr04.HCSR04(trigger_pin=board.D23, echo_pin=board.D24)

def get_distance(sensor):
    try:
        return round(sensor.distance, 2)
    except RuntimeError:
        return None

while True:
    distance = get_distance(left_sensor)
    print(f"Distance: {distance} cm")

    distance = get_distance(right_sensor)
    print(f"Distance: {distance} cm")

    distance = get_distance(front_sensor)
    print(f"Distance: {distance} cm")

    time.sleep(0.1)  # Adjust the delay as needed
