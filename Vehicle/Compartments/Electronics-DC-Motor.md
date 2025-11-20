# DC Drive Motor – Metal Gearmotor 25 mm, 210 RPM, 9–12 V

The robot uses a **25 mm all-metal gearmotor** rated for 9–12 V with a **210 RPM** free speed at 12 V. This motor provides a strong balance between torque, speed, durability, and electrical compatibility with our 3S Li-ion power system, making it ideal for the WRO FE environment.

---

## 1. Motor Specifications (Metal Gearmotor 25 mm, 34:1 Ratio)

**Electrical Characteristics**
- **Rated voltage:** 9–12 V  
- **Free speed:** ~210 RPM @ 12 V  
- **No-load current:** ~70 mA  
- **Stall current:** ~2.1 A  
- **Operating current (typical load):** 300–800 mA  
- **Polarity:** Standard brushed DC motor

**Mechanical Characteristics**
- **Gear ratio:** 34:1 steel gearbox  
- **Stall torque:** ~4.5 kg·cm  
- **Shaft diameter:** 4 mm D-shaft  
- **Body diameter:** 25 mm  
- **Construction:** all-metal gears, supported output shaft  
- **Weight:** ~95–120 g (depending on motor variant)

These characteristics match the robot’s performance needs while remaining electrically compatible with the chosen motor driver and fuse ratings.

---

## 2. Why This Motor Fits the Robot

### 1. **Voltage matches the 3S Li-ion Battery (11.1–12.6 V)**
- The motor runs optimally at **9–12 V**, aligning perfectly with the battery's range.
- Allows direct drive from **Vbat → DRV8871** without additional DC-DC converters.
- Reduces power losses and simplifies the electrical design.

### 2. **Current requirements fit within DRV8871 limits**
- The motor’s **~2.1 A stall current** is within:
  - DRV8871 driver peak capability  
  - The system’s **4 A fuse rating**  
- Current limit can be configured near 2 A to:
  - Protect the motor  
  - Protect PCB traces  
  - Avoid brownout conditions on the Raspberry Pi  

This ensures safe and reliable acceleration without overstressing components.

### 3. **Practical linear speed for the WRO field**
- ~210 RPM with our wheel diameter produces:
  - **0.6–0.8 m/s** top speed (depending on wheel size and load)
- Using PWM from the Raspberry Pi:
  - Smooth low-speed control when approaching obstacles  
  - Accurate turning behavior  
  - Enough maximum speed to complete **three laps comfortably**

### 4. **Mechanical compatibility with the chassis**
- 25 mm diameter fits the robot’s compact side profile.  
- 4 mm D-shaft mates with existing couplers and wheels.  
- Steel gears provide durability for repeated acceleration/decelleration cycles.  

Minimal custom machining is required, reducing build complexity and improving reliability.

---

## 3. Motor Driver Integration (DRV8871 – U10)

On the PCB, the motor is powered and controlled as follows:

- Motor wires connect via **TB1 (Motor Connector)**  
- DRV8871 (U10) is supplied directly from **Vbat**  
- Raspberry Pi GPIO pins provide **PWM and DIR** signals  
- Optional current limiting configured to ~2 A  

This design provides:
- A **clean single-channel drive path**  
- Easy debugging  
- Overcurrent protection  
- Consistent, predictable behavior

---

## 4. Alternative Motor Options Considered

### 1. **N20 Micro Gearmotors**
- **Pros:** Compact, lightweight  
- **Cons:** Too low torque for WRO floor friction and high-speed stability  

### 2. **37 mm High-Torque Gearmotors**
- **Pros:** Very high torque and robustness  
- **Cons:** Too large/heavy for chassis; unnecessary power draw  

### 3. **12 V Brushless DC Motors**
- **Pros:** High efficiency, long life  
- **Cons:** Require complex ESC control; harder to tune for precise low-speed moves  

### 4. **Stepper Motors**
- **Pros:** Accurate positioning  
- **Cons:** Heavy, inefficient, low top speed; unsuitable for WRO speed requirements  

### 5. **Servos (Continuous Rotation)**
- **Pros:** Easy control, integrated driver  
- **Cons:** Lower speed and torque; not suitable for high-speed driving  

---

## Summary

The **25 mm 210 RPM DC gearmotor** provides the ideal combination of speed, torque, voltage compatibility, and physical size for this robot. Its electrical profile matches the DRV8871 driver and system fuse ratings, while its mechanical form factor integrates cleanly with the chassis, resulting in a powerful and reliable drivetrain.

---

## Alternative
For this project, any type of DC motor can be used as long as it can be driven with 9–12 V.
The only difference will be in the code, since different motors have different gear ratios and speed characteristics.
