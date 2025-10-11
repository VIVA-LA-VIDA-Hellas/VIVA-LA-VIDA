import smbus2
import time

class MPU6050:
    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = smbus2.SMBus(bus)
        # Ξυπνάμε το MPU6050 (default κοιμάται)
        self.bus.write_byte_data(self.address, 0x6B, 0)

    def read_raw_data(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def get_accel_gyro(self):
        # Διαβάζει επιτάχυνση (m/s² περίπου) και γωνιακή ταχύτητα (°/s)
        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0

        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0

        return {
            "accel": (round(acc_x, 2), round(acc_y, 2), round(acc_z, 2)),
            "gyro": (round(gyro_x, 2), round(gyro_y, 2), round(gyro_z, 2))
        }
