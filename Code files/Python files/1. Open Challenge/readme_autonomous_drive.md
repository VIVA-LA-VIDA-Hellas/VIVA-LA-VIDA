# 🤖 VivaLaVida Autonomous Drive – WRO 2025 (1st Mission)

## 🧠 Project Overview
This repository contains the control software for the **VivaLaVida** team’s autonomous robot competing in the **WRO 2025 Future Engineers** category. The robot performs the **1st Mission**: navigating a 3×3 m arena with a central obstacle area, completing three laps while avoiding collisions and adapting to dynamic wall configurations.

The program runs on a **Raspberry Pi** and offers two operation modes:
- **GUI Mode (Debug):** Visual interface with real-time plots and tuning sliders.
- **Headless Mode (Competition):** Fully autonomous execution via physical start button.

---

## 🚗 Core Functionality
- **Autonomous Navigation:**
  - Follows walls using ToF (VL53L0X) or Ultrasonic sensors.
  - Performs left/right turns based on open-space detection and front thresholds.

- **Finite-State Machine Logic:**
  - Operates under these main states:  
    `IDLE → CRUISE → TURN_INIT → TURNING → POST_TURN → STOPPED`
  - Each state manages specific motor and servo behaviors.

- **Sensor Fusion & Filtering:**
  - Combines ToF/ultrasonic inputs using median, spike rejection, and optional EMA smoothing.
  - Stabilizes readings and eliminates transient noise.

- **Yaw & Turn Control:**
  - Integrates MPU6050 gyro data for angular tracking.
  - Executes precision turns with direction locking, timeouts, and slew-limited servo control.

- **Adaptive Environment Handling:**
  - Detects narrow corridors via side-sum analysis.
  - Dynamically scales speed and correction margins for safer movement.

- **Safety & Lap Control:**
  - Instant front-stop and obstacle recheck timer.
  - Lap counting with automatic halt after the final lap.

---

## 🧩 GUI Features
Built with **Tkinter** and **matplotlib**, providing:

- Live plots (front, side, yaw)
- Color-coded state and servo visualization
- Adjustable sliders for key parameters
- Save/Load configuration to JSON
- Data export to CSV for post-run analysis

---

## ⚙️ Hardware Setup
| Component | Function |
|------------|-----------|
| Raspberry Pi | Core computing unit |
| PCA9685 | Motor and servo PWM driver |
| VL53L0X ToF Sensors | Distance measurement |
| Ultrasonic Sensors | Alternative wall sensing |
| MPU6050 | Gyro for yaw tracking |
| LEDs | Status indication |
| Button | Start trigger in headless mode |

---

## 📊 Configuration
Configuration values are defined in code but can be overridden using `1st_mission_variables.json`.
All GUI slider changes can be saved and reloaded.

### Example JSON Structure
```json
{
  "SPEED_CRUISE": 25,
  "TURN_ANGLE_LEFT": 60,
  "TURN_ANGLE_RIGHT": 120,
  "MAX_LAPS": 3,
  "FILTER_ALPHA": 0.5
}
```

---

## 🧱 System Architecture Diagram
```text
 ┌──────────────────────────────────────────┐
 │             SENSOR THREAD                │
 │  - ToF & Ultrasonic readings             │
 │  - Median & EMA filtering                │
 │  - Updates global sensor_data            │
 └──────────────────────────────────────────┘
                    │
                    ▼
 ┌──────────────────────────────────────────┐
 │            ROBOT LOOP (FSM)              │
 │  IDLE → CRUISE → TURN_INIT → TURNING →   │
 │  POST_TURN → STOPPED                     │
 │  - Yaw integration (MPU6050)             │
 │  - Servo & motor control                 │
 │  - Lap counting & safety                 │
 └──────────────────────────────────────────┘
                    │
                    ▼
 ┌──────────────────────────────────────────┐
 │             GUI / HEADLESS MODE          │
 │  - Tkinter dashboard                    │
 │  - Sliders, plots, and CSV export       │
 │  - Physical start button (headless)     │
 └──────────────────────────────────────────┘
```

---

## 📈 Mission Path Diagram
```
+-------------------------------------------+
|                                           |
|   ←──────────── Lap 1 ────────────→        |
|   |                               |        |
|   |     +-----------+             |        |
|   |     |  Center   |             |        |
|   |     |  Obstacle |             |        |
|   |     +-----------+             |        |
|   |                               |        |
|   ←──────────── Lap 2 ────────────→        |
|                                           |
+-------------------------------------------+
```

---

## 🧾 Parameter Summary
| Category | Variable | Description | Example |
|-----------|-----------|-------------|----------|
| Speed | `SPEED_CRUISE` | Normal drive speed | 25 |
| Turn | `TURN_ANGLE_LEFT` / `RIGHT` | Servo turn angles | 60 / 120 |
| Safety | `STOP_THRESHOLD` | Distance to stop (cm) | 20 |
| Sensor | `FILTER_ALPHA` | Filter smoothing | 0.5 |
| Logic | `MAX_LAPS` | Total laps before stop | 3 |

---

## 🧰 Libraries
```bash
adafruit_pca9685
adafruit_mpu6050
adafruit_vl53l0x
gpiozero
matplotlib
tkinter
numpy
json
csv
```

---

## 🏁 Run Modes
- **GUI Mode:**
  ```bash
  python3 autonomous_drive.py
  ```
- **Headless Mode:**
  Set `USE_GUI = 0` and press the hardware start button.

---

## 👨‍💻 Authors
**Team VivaLaVida** – WRO 2025 Future Engineers Project  
Developed by the VivaLaVida Robotics Team.

---

## 🧾 License
Released under the **MIT License** for educational and competition use.

