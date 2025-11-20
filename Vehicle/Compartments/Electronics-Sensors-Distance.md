# Distance Sensors – Why This Combination

The selected sensor suite is engineered to provide the robot with **robust, real-time spatial awareness** under the constraints of the WRO 2025 FE environment. Each sensor category contributes different strengths: short-range precision, long-range detection, and orientation stability. Together, they form a redundant, multi-modal perception system that enables reliable autonomous navigation even in unpredictable wall placements and obstacle scenarios.

---

## 1 Distance Metering Sensors

---

## a) Time-of-Flight (ToF) Sensors – VL53L0X

Time-of-Flight sensors are used for **high-precision, short-range** distance measurement. They operate using **laser-based VCSEL emitters** and measure the time taken for light to return from a surface.

### Purpose in the Robot
- Provide high-resolution (<5 mm) distance data for **close-proximity navigation**.
- Essential for **unparking logic** during the obstacle challenge.
- More reliable on **dark, matte, or angled surfaces** compared to ultrasonics.
- Excellent in detecting narrow gaps or side distances during tight maneuvers.

### Hardware & Positioning
Six (6) **VL53L0X** modules are mounted around the chassis:
        Front
Left-Front Right-Front
Left             Right
        Back

### VL53L0X Technical Specifications
- **Sensor type:** Time-of-Flight (laser ranging, VCSEL)
- **Interface:** I²C (400 kHz Fast Mode)
- **Operating voltage:** 2.6–5 V (5 V rail used)
- **Range:** 2–200 cm (optimal precision <50 cm)
- **Resolution:** ±3 mm typical
- **Field of View:** ~25°
- **Measurement speed:** up to 50 Hz
- **Key advantage:** immune to acoustic noise and surface angle variability

---

## b) Ultrasonic Sensors – HC-SR04

Ultrasonic sensors provide **long-range detection** using acoustic pulses, complementing the short-range precision of the ToF sensors.

### Purpose in the Robot
- Detect walls and open space at distances **up to 3m**.
- Used during **cruising**, **turn decisions**, and **wall-following**.
- Redundant to ToF for improved situational awareness.

### Hardware & Positioning
Three (3) **HC-SR04** units are mounted:
        Front
Left             Right

### HC-SR04 Technical Specifications
- **Sensor type:** ultrasonic (40 kHz)
- **Operating voltage:** 5 V (with resistor divider on echo → 3.3 V GPIO safe)
- **Range:** 2–400 cm
- **Resolution:** ~3 mm
- **Beam angle:** ~15°
- **Trigger pulse:** 10 µs
- **Sampling frequency:** ~20 Hz
- **Limitations:** acoustic reflections, noise spikes, angle-dependent accuracy

### Combined Benefits (ToF + Ultrasonic)
Using both categories produces a robust hybrid sensing model:

- **Short-range precision** (ToF)
- **Long-range awareness** (ultrasonic)
- **Redundancy** when one fails due to:
  - acoustic noise
  - reflective surfaces
  - angled walls causing weak returns

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

## Final Outcome – Sensor Roles in the Final Design

After extensive testing:

### ToF – Final Evaluation
- **Pros**
  - Excellent <30 cm accuracy  
  - Very stable readings (low noise)  
  - Resistant to dark/angled surfaces  
- **Cons**
  - Limited practical range (>30–50 cm unreliable in vertical-wall environments)

### Ultrasonic – Final Evaluation
- **Pros**
  - Reliable long-range detection (>1 m)  
  - Good for early warning of walls  
- **Cons**
  - Susceptible to noise, reflections, and angle errors  

### Final Usage
- **Ultrasonic sensors**  
  - Used during **cruising**, **turning decisions**, and **general wall-following**.  
- **ToF sensors**  
  - Used for **unparking decisions** in the obstacle challenge.  
  - Used for precise short-range calibrations.

---

## Alternative Sensors Considered

Several alternative sensing technologies were evaluated but not selected due to cost or complexity.

### 1. Infrared (IR) Proximity Sensors
- Pros: cheap, wide availability  
- Cons: poor performance in ambient light, reflective-surface errors

### 2. [Lidar](https://github.com/VIVA-LA-VIDA-Hellas/VIVA-LA-VIDA/blob/main/Vehicle/Compartments/Lidar.md)
- Pros: full 360° mapping, excellent accuracy  
- Cons: too large, too heavy, not allowed or impractical for WRO FE robot dimensions

### 3. Wheel Encoders
- Pros: accurate drive-distance estimation  
- Cons: add mechanical complexity; unreliable due to robot vibrations and tight-turn requirements

### 4. Depth Cameras (Intel RealSense)
- Pros: high-resolution 3D obstacle detection  
- Cons: expensive, high power consumption, overkill for 3×3 m field

---
