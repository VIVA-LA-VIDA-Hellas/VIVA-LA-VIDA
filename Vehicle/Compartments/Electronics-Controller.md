# Controller – Raspberry Pi 5

The **Raspberry Pi 5** was selected as the primary controller due to its high processing capability, extensive I/O support, mature software ecosystem, and compatibility with all sensing and control requirements of the robot. It provides the computational headroom necessary for real-time camera processing, sensor fusion, autonomous decision making, and system diagnostics.

---

## 1.1 Why Raspberry Pi 5 Was Chosen

- Powerful enough to run **Linux + Python** alongside real-time tasks such as:
  - OpenCV image processing  
  - Multi-sensor data acquisition (ToF, Ultrasonic, IMU)  
  - Sensor fusion and decision logic  
  - Logging, threading, and GUI monitoring (if needed)
- The **40-pin GPIO header** exposes all required interfaces:
  - **I²C buses** for IMU, ToF sensors, PCA9685, colour sensor, OLED  
  - **GPIO** pins for HC-SR04 ultrasonic sensors, LEDs, and push buttons  
  - **PWM-capable pins** (via hardware or PCA9685 expansion)
- Comprehensive Python library support:
  - `vl53l0x` for ToF  
  - `hcsr04` for ultrasonic  
  - `mpu6050` for IMU  
  - `adafruit_pca9685` for motor/servo control  
  - `picamera2` for camera streaming  
- Operates on **5 V**, fully compatible with the robot’s regulated **5 V DC-DC supply**, ensuring a simple and reliable power architecture.

---

## 1.2 Raspberry Pi 5 – Technical Specifications

- **CPU:** 4-core ARM Cortex-A76 @ 2.4 GHz  
- **GPU:** VideoCore VII  
- **RAM:** 8 GB LPDDR4X
- **I/O Interfaces:**  
  - 40-pin GPIO header (3.3 V logic)  
  - **Two** I²C buses  
  - SPI, UART, PWM support  
  - PCIe 2.0 lane (unused in this project)  
- **USB Ports:** 2× USB 3.0 + 2× USB 2.0  
- **Networking:** Gigabit Ethernet, Wi-Fi 802.11ac, Bluetooth 5.0  
- **Camera Interface:** CSI connector for high-resolution cameras  
- **Power Input:** 5 V via USB-C (minimum 5 A recommended)  
- **Operating System:** Raspberry Pi OS (Linux)

These capabilities provide more than enough performance headroom for real-time autonomous robotics tasks.

---

## 1.3 Controller Role in the Robot

The Raspberry Pi 5 is responsible for:

- **High-level autonomy:** obstacle detection, path tracking, state machine logic
- **Camera processing:** colour detection, object tracking, horizon cropping, line sensing
- **Sensor integration:** ToF, Ultrasonic, IMU data handling and fusion
- **Actuator control:** steering and motor PWM control via PCA9685
- **Communication:** debugging, logging, on-field testing
- **Timing & threading:** synchronized sensor loops, soft-real-time tasks

The Pi 5 centralizes all functionality, eliminating the need for additional microcontrollers.

---

## 1.4 Alternative Controller Options Considered

### 1. Arduino / ESP32 Class Microcontrollers
- **Pros:** low cost, low power, excellent for simple robots  
- **Cons:** insufficient for real-time camera processing and multi-threaded sensor fusion  

### 2. Raspberry Pi 4
- **Pros:** cheaper, widely supported  
- **Cons:** ~30–40% slower; thermal throttling under continuous OpenCV workloads  

### 3. NVIDIA Jetson Nano / Orin Nano
- **Pros:** GPU acceleration, excellent for advanced vision  
- **Cons:** high power consumption, unnecessary for WRO tasks, physically larger  

### 4. Pi Zero 2 W
- **Pros:** small, power-efficient  
- **Cons:** too slow for real-time camera analysis and advanced logic  

---

## Final Justification

The **Raspberry Pi 5** provides the ideal balance of **performance**, **I/O capability**, **library support**, and **ease of development**, making it the most effective single-board controller for a fully autonomous WRO 2025 FE robot.

---
