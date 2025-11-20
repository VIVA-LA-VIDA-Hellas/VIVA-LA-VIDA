# Gyroscope

The selected sensor suite is engineered to provide the robot with **robust, real-time spatial awareness** under the constraints of the WRO 2025 FE environment. Each sensor category contributes different strengths: short-range precision, long-range detection, and orientation stability. Together, they form a redundant, multi-modal perception system that enables reliable autonomous navigation even in unpredictable wall placements and obstacle scenarios.

---

## 1 Acceleration and Rotation Sensors

---

## a) IMU – GY-521 (MPU6050)

The IMU provides essential orientation data that replaces wheel encoders. The robot relies on **gyro integration** to maintain heading and stability across the 3×3m field.

### Purpose in the Robot
- **Yaw estimation** for straight-line driving.
- Stabilized **turning angles** and orientation resets.
- Compensation for wheel slip or irregular surfaces.

### MPU6050 Technical Specifications
- **Sensor type:** 6-axis IMU  
  - 3-axis gyroscope  
  - 3-axis accelerometer  
- **Interface:** I²C (3.3 V logic)
- **Gyroscope range:** ±250 / ±500 / ±1000 / ±2000 °/s
- **Accelerometer range:** ±2g / ±4g / ±8g / ±16g
- **Gyro sensitivity:** 131 LSB/(°/s) at ±250 setting
- **Sampling:** up to 1 kHz
- **DMP:** available but not used in this project
- **Power:** 3.3–5 V (regulated to 3.3 V onboard)

### Why IMU is Essential
- Robot **does not use wheel encoders**, so IMU is the only odometry feedback.
- Provides smooth trajectory correction.
- Detects drift over long straight segments.

---

## 3 Optional / “Not Used” Sensors

The PCB includes footprints for possible expansion modules:

- Colour sensor  
- OLED diagnostic display  
- Extra tactile button  
- Piezo buzzer  
- DIP configuration switch  
- LIDAR interface header  

These are currently **NOT USED**, but the PCB layout supports future features such as color-based decision making, on-device debugging, or long-range scanning without redesigning the hardware.

---

## Alternative Sensors Considered

Several alternative sensing technologies were evaluated but not selected due to cost or complexity.

### 1. IMU with Magnetometer (MPU9250)
- Pros: absolute north-based heading  
- Cons: indoor magnetic interference makes it unreliable on metal-frame competition tables

---
