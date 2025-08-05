import board
import busio
import adafruit_lsm6ds.lsm6ds3

class LSM6DS3:
    def __init__(self):
        # Initialize I2C and sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lsm6ds.lsm6ds3.LSM6DS3(i2c)

    def read_acceleration(self, axis):
        accel = self.sensor.acceleration  # returns (x, y, z) in m/s^2
        return self._select_axis(accel, axis, "acceleration")

    def read_gyroscope(self, axis):
        gyro = self.sensor.gyro  # returns (x, y, z) in dps
        return self._select_axis(gyro, axis, "gyroscope")

    def read_temperature(self):
        return self.sensor.temperature  # °C

    def _select_axis(self, vector, axis, name):
        axis = axis.lower()
        if axis == 'x':
            return vector[0]
        elif axis == 'y':
            return vector[1]
        elif axis == 'z':
            return vector[2]
        else:
            raise ValueError(f"Invalid axis '{axis}' for {name}. Use 'x', 'y', or 'z'.")
