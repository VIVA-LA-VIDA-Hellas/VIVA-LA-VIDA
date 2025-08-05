import time
from lsm6ds3_simple import LSM6DS3

sensor = LSM6DS3()

try:
    while True:
        ax = sensor.read_acceleration("x")
        ay = sensor.read_acceleration("y")
        az = sensor.read_acceleration("z")

        gx = sensor.read_gyroscope("x")
        gy = sensor.read_gyroscope("y")
        gz = sensor.read_gyroscope("z")

        temp = sensor.read_temperature()

        print(f"Accel X: {ax:.2f} m/s² | Y: {ay:.2f} m/s² | Z: {az:.2f} m/s²")
        print(f"Gyro  X: {gx:.2f} dps  | Y: {gy:.2f} dps  | Z: {gz:.2f} dps")
        print(f"Temp: {temp:.2f} °C")
        print("------")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")
