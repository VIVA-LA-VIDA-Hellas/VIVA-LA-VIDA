# VivaLaVida – WRO 2025 Future Engineers (Open Challenge)
**Mission:** Complete 3 autonomous laps on a 3×3 m track with randomized inner walls.  
**Robot:** 4-wheeled, single steering actuator, physically linked drive axle(s), Raspberry Pi + PCA9685 + MPU6050 + Ultrasonic/ToF.

---

## 1) Overview
This repository contains the autonomous navigation stack used by Team **VivaLaVida** for WRO 2025 **Future Engineers – Open Challenge**. The software implements:
- A **finite state machine (FSM)**: `IDLE → CRUISE → TURN_INIT → TURNING → POST_TURN → STOPPED`.
- **Sensor fusion** (ultrasonic and optional ToF) with robust filtering and spike rejection.
- **Yaw-based turning** via an **MPU6050** gyro and target-angle tracking with tolerance & safety guards.
- **Adaptive behavior** in narrow corridors (sum of side distances) using speed/threshold scaling.
- A **Tkinter GUI** for live telemetry, slider-based parameter tuning, and CSV export.
- **Headless mode** for competition (one Start button).

The design prioritizes stability, transparency, and WRO compliance. It is structured to make tuning explicit, repeatable, and explainable for judges.

---

## 2) Field & Rules Tie‑In (Open Challenge)
- **Task:** Drive **three (3) laps** autonomously. Direction and starting section vary per round.
- **Walls:** Randomized inner walls; **do not move** walls during a run.
- **Stop:** After 3 laps, stop as specified (see WRO rules) or the round ends when passing the finish section.
- **Robot constraints:** Size, mass, single steering actuator, physically connected drive axle(s), no wireless during runs.
- **Operation:** One **power switch** + one **Start button**. Headless mode supported.

> This implementation focuses on lane-keeping using **side distances** (for post‑1st‑turn correction),
front distance for **turn initiation**, and a **gyro‑based** termination of turns for deterministic 90° cornering.

---

## 3) Hardware
- **Controller:** Raspberry Pi‑class SBC
- **Motor/Servo PWM:** PCA9685
- **IMU:** MPU6050 (gyro Z used for yaw integration)
- **Distance sensors:** Ultrasonic (default) and optional VL53L0X ToF (front and/or sides)
- **GPIO:** Start button + LEDs (status); motor fwd/rev channels; steering servo
- **Power:** Isolated logic/motor supply recommended; follow WRO safety best practices

> The code initializes I²C and configures PCA9685 at 50 Hz for servo/motor channels.

---

## 4) Software
- **Language:** Python 3
- **Key libs:** `RPi.GPIO`, `gpiozero`, `adafruit-circuitpython-*`, `tkinter`, `matplotlib`, `numpy`
- **Threads:** 
  - `sensor_reader` – continuous sensor fusion & smoothing
  - `robot_loop` – FSM, yaw integration, actuation
- **GUI:** Start/Stop control, status indicator, plots (front/left/right/yaw), sliders, CSV export

---

## 5) Parameters (selected)
| Name | Purpose | Typical |
|---|---|---|
| `FRONT_TURN_TRIGGER` | Enter `TURN_INIT` when front < trigger | 90 cm |
| `TURN_DECISION_THRESHOLD` | Side threshold to consider “open” | 100 cm |
| `TURN_ANGLE_LEFT/RIGHT` | Servo setpoint for turns | 60° / 120° |
| `TARGET_TURN_ANGLE` | Desired yaw delta | 85° |
| `TURN_ANGLE_TOLERANCE` | Stop when within ±tol | 5° |
| `TURN_TIMEOUT` | Guard to end turn | 4.5 s |
| `TURN_LOCKOUT` | Min time between turns | 1.5 s |
| `SOFT_MARGIN` | Wall-following activation margin | 30 cm |
| `MAX_CORRECTION` | Max steering correction | 7° |
| `STOP_THRESHOLD` | Emergency stop front distance | 20 cm |

All parameters are adjustable via sliders in GUI mode and can be **saved/loaded** or overridden via `1st_mission_variables.json` at runtime.

---

## 6) Control Logic (high level)
- **CRUISE:** Straight motion; after 1st turn, apply timed wall-following corrections if side distance < soft margin.
- **TURN_INIT:** When `front < FRONT_TURN_TRIGGER` and lockout OK; keep straight (or gentle correction after 1st turn) while waiting for **exactly one** open side. Apply optional **direction lock** after the first decision.
- **TURNING:** Set servo to the commanded (left/right) angle; integrate gyro Z and **stop when target yaw** is reached within tolerance. Guards: timeout and max yaw.
- **POST_TURN:** Short straight stretch to stabilize, then return to CRUISE.
- **STOPPED:** Triggered by obstacle or max-laps; obstacle mode self‑retries after a wait window.

**Narrow Mode:** If `left+right` (valid readings) < threshold, scale speeds/thresholds to be conservative in tight corridors.

---

## 7) Filtering & Stability
- **Median + EMA** smoothing and **jump rejection** to mitigate spikes.
- Separate params for **ToF** vs **Ultrasonic** to reflect different noise/jitter characteristics.
- **Servo slew‑rate limiting** (`SERVO_SLEW_DPS`) can soften transients and reduce overshoot in tight fields.

---

## 8) Data & Telemetry
- Live plots: Front, Left, Right distances, and Yaw.
- CSV export includes time series, state, turns/laps, and a snapshot of current slider values for experiment traceability.

---

## 9) How to Run
1. Wire PCA9685, servo, motor channels, sensors, and the **Start** button.
2. Install Python deps on the Pi (`pip install` for the listed libs; system packages for `tk` & `matplotlib` backends as needed).
3. (Optional) Place `1st_mission_variables.json` next to the script to override defaults.
4. **GUI mode:** Run script → `Start Readings` → `Start Loop`.  
   **Headless:** Power on → press physical **START** button after boot.
5. Tune parameters using sliders; **Save/Load** presets; export **CSV** after test runs.

---

## 10) Repository Structure (suggested)
```
/docs
  VivaLaVida_FE_Mission1_TechDoc_WithCode.pdf
  diagrams/
    VivaLaVida_Route_States.png
    VivaLaVida_Decision_Flow.png
/src
  autonomous_drive.py
  1st_mission_variables.json (optional)
/data
  runs/*.csv
README.md
LICENSE
```

---

## 11) Compliance Checklist (Open Challenge)
- [x] Single steering actuator; mechanically linked drive axle(s)  
- [x] No wireless during runs; one power switch + one **Start** button  
- [x] Robot dimensions and mass within limits  
- [x] No wall moving; fully autonomous three laps  
- [x] Public GitHub with code, docs, photos, and a video (≥30s driving)  

---

## 12) Credits
Team **VivaLaVida** – WRO 2025 Future Engineers  
Sensors, libraries, and hardware as referenced above.

---

> For judges: This repository includes an engineering‑focused explanation of mobility, power/sense, and obstacle management strategies. The diagrams and document in `/docs` align with the WRO documentation rubric and help reproduce or extend the work.
