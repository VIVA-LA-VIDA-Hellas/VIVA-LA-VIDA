import smbus
import time

# -----------------------------
# MPU6050 Registers and Constants
# -----------------------------
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43

# Sensitivity (depends on FS_SEL in config, here ï¿½250ï¿½/s)
GYRO_SCALE_MODIFIER = 131.0  # LSB per ï¿½/s

# -----------------------------
# Initialize I2C (bus 1 for Raspberry Pi)
# -----------------------------
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up MPU6050

# -----------------------------
# Helper function to read 16-bit raw gyro data
# -----------------------------
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

# -----------------------------
# Initialize variables
# -----------------------------
prev_time = time.time()
angle_x = 0.0
angle_y = 0.0
angle_z = 0.0

print("Reading MPU6050 gyro data (Ctrl+C to stop)...\n")

# -----------------------------
# Main loop
# -----------------------------
try:
    while True:
        # Read raw gyro data
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = read_raw_data(GYRO_XOUT_H + 4)

        # Convert to degrees/second
        gx = gyro_x / GYRO_SCALE_MODIFIER
        gy = gyro_y / GYRO_SCALE_MODIFIER
        gz = gyro_z / GYRO_SCALE_MODIFIER

        # Time difference
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Integrate to get angle
        angle_x += gx * dt
        angle_y += gy * dt
        angle_z += gz * dt

        # Print results
        print(f"Gyro rate: X={gx:.2f}ï¿½/s Y={gy:.2f}ï¿½/s Z={gz:.2f}ï¿½/s | "
              f"Angle: X={angle_x:.2f}ï¿½ Y={angle_y:.2f}ï¿½ Z={angle_z:.2f}ï¿½")

        time.sleep(0.05)
        angle_z += 0.045

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    print("Gyro tracking ended.")
