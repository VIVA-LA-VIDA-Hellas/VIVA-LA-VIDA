# First mission - Final proframm description
<a id="top"></a>

<!-- TOC -->
- [Project Overview](#project-overview)
- [Core Functionality](#core-functionality)
- [Control Logic (high level)](#control-logic-high-level)
- [Code Structure](#code-structure)
- [Hardware Setup](#hardware-setup)
- [Configuration](#configuration)
- [System Architecture Diagram](#system-architecture-diagram)
- [Mission Path Diagram](#mission-path-diagram)
<!-- /TOC -->

The program runs on a **Raspberry Pi** and offers two operation modes:
- **GUI Mode (Debug):** Visual interface with real-time plots and tuning sliders.
- **Headless Mode (Competition):** Fully autonomous execution via physical start button.

---

## Core Functionality <a id="core-functionality"></a>
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

## Control Logic (high level) <a id="control-logic-high-level"></a>
- **CRUISE:** Straight motion; after 1st turn, apply timed wall-following corrections if side distance < soft margin.
- **TURN_INIT:** When `front < FRONT_TURN_TRIGGER` and lockout OK; keep straight (or gentle correction after 1st turn) while waiting for **exactly one** open side. Apply optional **direction lock** after the first decision.
- **TURNING:** Set servo to the commanded (left/right) angle; integrate gyro Z and **stop when target yaw** is reached within tolerance. Guards: timeout and max yaw.
- **POST_TURN:** Short straight stretch to stabilize, then return to CRUISE.
- **STOPPED:** Triggered by obstacle or max-laps; obstacle mode self‑retries after a wait window.

**Narrow Mode:** If `left+right` (valid readings) < threshold, scale speeds/thresholds to be conservative in tight corridors.

---

## Code Structure <a id="code-structure"></a>

1. Imports
    - External Libraries
    - Internal Modules
2. Variables
    - Global Constants
    - Robot-Specific Variables
3. H/W Setup
    - Hardware Configuration
    - Pin Assignments
4. Robot Classes
    - Sensor Handling
    - Motor Control
    - State Management
5. Robot Loop
    - Main Control Loop
    - Decision Making
    - Movement Logic
6. GUI
    - Tkinter Setup
    - Sliders & Data Visualization
7. Main
    - Initialization
    - Robot Execution
      
---

## Hardware Setup <a id="hardware-setup"></a>
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

## Configuration <a id="configuration"></a>
Configuration values are defined in code but can be overridden using `1st_mission_variables.json`.
All GUI slider changes can be saved and reloaded.

---

## System Architecture Diagram <a id="system-architecture-diagram"></a>
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
 │  - Dashboard                             │
 │  - Sliders, plots, and CSV export        │
 │  - Physical start button (headless)      │
 └──────────────────────────────────────────┘
```

## Run Modes <a id="run-modes"></a>
- **Debugging (GUI) Mode:**
  Set `USE_GUI = 1` and press Start Readings & Start Loop button.
- **Competition (Headless) Mode:**
  Set `USE_GUI = 0` and press the hardware start button.

---
