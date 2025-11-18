# Steering Servo – TowerPro MG996R

The robot uses a **TowerPro MG996R** high-torque metal-gear servo for steering. This servo is widely used in robotics due to its strength, reliability, and standard RC interface, making it well suited for precise and repeatable steering control under load.

---

## 1 MG996R Technical Specifications

**Electrical Characteristics**
- **Operating voltage:** 4.8–6.0 V  
- **Running current:** 0.5–0.9 A (typical)  
- **Stall current:** ≈2.5 A  
- **Signal requirements:**  
  - Standard RC PWM (50–60 Hz)  
  - Pulse width: ~1.0–2.0 ms (±10% tolerance)

**Mechanical Characteristics**
- **Stall torque:**  
  - ~9.4 kg·cm @ 4.8 V  
  - ~11 kg·cm @ 6.0 V  
- **Speed:** 0.15–0.17 s per 60°  
- **Gears:** Metal gear train  
- **Bearings:** Dual ball bearings  
- **Servo type:** Standard size (40 × 20 × 45 mm)  
- **Output spline:** 25T  
- **Rotation:** ~180° mechanical, ~90–120° effective control range  

These specifications provide more than enough torque and responsiveness for fast WRO field turns and fine IMU-guided corrections.

---

## 2 Why This Servo Was Chosen

### 1. **High torque at 5 V**
- Even at 5 V, the servo can hold steering angles under load.  
- Strong enough to resist forces when the robot pushes lightly against walls or during tight slalom-like turns.  

This ensures steering reliability throughout all three laps.

### 2. **Standard RC interface via PCA9685**
- MG996R uses the classic **50–60 Hz servo protocol**.  
- The PCA9685 provides:
  - **12-bit resolution PWM**  
  - Independent timing for all servo channels  
  - Zero CPU overhead for pulse generation  

This ensures precise and jitter-free steering angles without timing issues on the Raspberry Pi.

### 3. **Robust mechanical design**
- Metal gear train prevents stripping during sudden impacts.  
- Ball bearings improve durability under frequent directional changes.  
- Standard mounting holes and horn types simplify integration with the chassis.

This results in a durable, competition-ready steering assembly.

### 4. **Correct operating range for robot steering**
- Software uses:
  - **90°** → center  
  - ~**50°** → full left  
  - ~**130°** → full right  
- The MG996R easily supports this range while maintaining torque and precision.

This enables both tight turns around corners and smooth fine-angle corrections based on IMU feedback.

---

## 3 PCB and Power Integration

The servo connects to the electronics as follows:

- **PCA9685 Channel 0** → PWM steering command  
- **5 V servo rail** → high-current supply  
- **Ground** → shared with logic ground for stable signal reference  

### Advantages of this setup:
- Keeps **servo current** off the Raspberry Pi’s 3.3 V rail  
- Ensures timing stays synchronized with any additional servos  
- Avoids voltage dips on logic components when servo torque spikes  
- Provides stable, isolated power on a dedicated 5 V rail  

This architecture protects the controller while maintaining precise steering behavior.

---

## 4 Alternative Servos Considered

### 1. **SG90 / MG90S Micro Servos**
- **Pros:** cheap, lightweight  
- **Cons:** far too weak for steering; gear stripping risk  

### 2. **High-End Digital Servos (e.g., Savox, Futaba)**
- **Pros:** extremely precise and fast  
- **Cons:** expensive; unnecessary performance for WRO FE  

### 3. **Continuous-Rotation Servos**
- **Pros:** simple speed control  
- **Cons:** cannot hold an angle → unsuitable for steering mechanisms  

### 4. **Stepper-Based Steering (NEMA 14/17)**
- **Pros:** extremely accurate  
- **Cons:** heavy, power-hungry, requires drivers, slow  
- Overkill for a small, fast robot

---

## Summary

The **TowerPro MG996R** provides the optimal combination of **torque, durability, ease of control, and electrical compatibility** for the robot’s steering system. Its integration with the PCA9685 ensures reliable, precise angle control while maintaining electrical isolation and mechanical robustness.

---
