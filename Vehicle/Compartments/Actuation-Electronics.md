# Actuation Electronics

## 1 PCA9685 – 16-Channel I²C Servo Driver

The **PCA9685** is used to generate precise, stable PWM signals for servos and other actuators. It offloads timing from the Raspberry Pi and provides highly accurate pulse control.

### Technical Specifications
- **16 independent PWM channels**  
- **I²C communication** (up to 1 MHz Fast-Mode Plus)  
- **12-bit resolution** (4096 steps)  
- **Frequency range:** 40–1000 Hz  
- **Logic level:** 3.3 V (Pi-compatible)  
- **Output power for servos:** servo rail powered at **5 V**  
- **On-chip oscillator** for autonomous pulse generation

### Why We Use the PCA9685
- Provides **stable, jitter-free** servo pulses  
- Raspberry Pi no longer needs to generate real-time PWM  
- 16 channels offer large expansion headroom:
  - Main steering servo  
  - Additional servos for future mechanisms  
  - LEDs or other PWM devices  
- Clean separation between **logic (3.3 V)** and **servo power (5 V)**  
- Allows the robot to maintain steering precision even under heavy CPU load (OpenCV, sensor fusion)

### PCB Integration
- Connected via the **I²C bus** (shared with ToF sensors, IMU, etc.)  
- Servo power supplied from a dedicated **5 V high-current rail**  
- Ground plane shared for stable reference  
- Steering servo plugged into **Channel 0**, leaving 15 channels for future expansion

---

## 2 Benefits of This Dual-Driver Architecture

- **Dedicated high-current path** for the drive motor (DRV8871)  
- **High-precision PWM generation** for steering (PCA9685)  
- **Reduced CPU overhead** on the Raspberry Pi  
- **Electrical isolation** between motor noise and servo signals  
- **Easy swapping** of motors and servos during competition  
- **Scalability** for additional mechanisms without redesigning the board  

---

## 3 Alternative Driver Solutions Considered

### 1. Raspberry Pi GPIO Softw
