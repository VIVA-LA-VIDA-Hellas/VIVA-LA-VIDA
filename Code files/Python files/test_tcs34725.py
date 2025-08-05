import time
from tcs34725_simple import TCS34725_Sensor

# Initialize sensor on transistor GPIO 12, gain=16, integration_time=16ms
sensor = TCS34725_Sensor(transistor_gpio_pin=12, gain=16, integration_time=16)

sensor.open()

try:
    while True:
        r = sensor.read_red()
        g = sensor.read_green()
        b = sensor.read_blue()
        clear = sensor.read_color()[3]
        lux = sensor.read_lux()
        color_temp = sensor.read_color_temperature()

        print(f"R: {r}, G: {g}, B: {b}, Clear: {clear}, Lux: {lux:.2f}, Temp: {color_temp}K")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nExiting gracefully...")

finally:
    sensor.close()
