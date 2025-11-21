# 2nd Mission – Final Program Description

<a id="top"></a>

<!-- TOC -->

* [Project Overview](#project-overview)
* [Core Functionality](#core-functionality)
* [Control Logic (high level)](#control-logic-high-level)
* [Code Structure](#code-structure)
* [Hardware Setup](#hardware-setup)
* [Configuration](#configuration)
* [Vision & Colour Processing](#vision--colour-processing)
* [System Architecture Diagram](#system-architecture-diagram)
* [Mission Path & Behaviour](#mission-path--behaviour)

<!-- /TOC -->

---

## Project Overview

This program runs on a **Raspberry Pi** and controls the robot for the **2nd Mission – WRO 2025 Future Engineers (VivaLaVida)**.

It is a **headless, competition-ready** script:

* Starts via a **hardware button**.
* Performs a **smart unpark** manoeuvre to exit the parking box.
* Uses **camera vision, IMU, ToF and ultrasonic sensors** to:

  * Detect **red/green traffic signs** (obstacles).
  * Detect **blue/orange diagonal lines** (turn markers).
  * Navigate the full track for **3 laps** with **4 line-based turns per lap**.

---

## Core Functionality <a id="core-functionality"></a>

* **Smart Unpark:**

  * Measures left/right ToF distances at startup and chooses the more open side.
  * Performs a **two-phase yaw-controlled manoeuvre** to exit the parking area:

    * Phase 1: ~45° steering into free space.
    * Phase 2: counter-turn to align with the driving lane.
  * Resets yaw and transitions into the main FSM loop.

* **Autonomous Navigation & Lap Control:**

  * Drives in **CRUISE** mode with IMU-based straightening.
  * Detects **blue/orange floor lines** (via HSV + Hough) to perform **90° turns**.
  * Counts turns and laps; after **`TOTAL_TURNS = 12`** (3 laps × 4 turns) it performs a final straight and stops.

* **Vision-Based Obstacle Avoidance:**

  * Detects **red and green signs** as obstacles using HSV masks and contour analysis.
  * Locks onto an obstacle when it is:

    * Large enough (area threshold).
    * Visually close (low in image, near robot).
  * Executes **two-phase avoidance**:

    * Short **back-off** with steering away from the sign.
    * **Forward follow** around the obstacle using:

      * Sign colour (Red → pass on right, Green → pass on left).
      * Sign **horizontal position** (x-offset) for more precise path shaping.

* **Emergency “Blue-Backward” Escape:**

  * Defines a “car box” area in the image near the robot.
  * When an obstacle is detected inside this region, it immediately:

    * Reverses at higher speed.
    * Steers away from the box to escape imminent collision.
  * Then resumes normal cruise.

* **IMU-Based Yaw Control:**

  * Uses MPU6050 gyro Z to:

    * Maintain a **running yaw estimate**.
    * **Keep straight** in CRUISE with proportional yaw correction and deadband.
    * Perform **yaw-based 90° turns** in TURN state:

      * Integrates yaw and stops turning near a **failsafe yaw** (~80°).
      * Applies small yaw offsets after the turn to avoid drifting.
  * Includes **bias estimation** and **soft decay** to reduce drift over time.

* **Wall & Collision Safety:**

  * Uses **side ultrasonic + ToF** to detect walls.
  * If a wall is too close on one side, it forces a collision-avoidance steering angle.
  * Uses **front ultrasonic** as a gate to allow line-based turns only when distance is within a safe window.

---

## Control Logic (high level) <a id="control-logic-high-level"></a>

Main states (FSM):

1. **SMART UNPARK (UnPark_L / UnPark_R)**

   * Runs once at start, chooses left or right based on ToF.
   * Performs two yaw-controlled phases and then resets yaw.
   * Enters main loop in **CRUISE** state.

2. **CRUISE**

   * Motors at **NORMAL_SPEED**.
   * Steering controlled by **IMU yaw correction**, with extra wall-safety overrides from side ToF/ultrasonic.
   * Continuously processes:

     * Red/green masks for obstacle detection.
     * Blue/orange masks + Hough for diagonal line detection.
   * Prioritisation:

     * If a valid obstacle is locked (close & stable) → **STATE_AVOID**.
     * Else, if a line is confirmed, front distance is OK, and turn cooldown is satisfied → **STATE_TURN** (as long as obstacle is not “closer” than the line).
   * Handles **turn gating**:

     * Cooling-off time after each turn.
     * Minimum interval between turns.
     * Temporary ignore window for lines after each turn.

3. **TURN**

   * Sets servo to **hard left or right** based on the global `direction` (from unpark).
   * Drives at **TURN_MOTOR_SPEED**.
   * Integrates yaw until **|yaw| ≥ TURN_FAILSAFE_MAX_DEG** (~80°):

     * Ends the turn.
     * Applies a small yaw reset offset.
     * Increments **turn count** and updates **lap count**.
   * If total turns reach **TOTAL_TURNS**:

     * Drives straight for a short time and stops.
   * Otherwise transitions to **STATE_POST_TURN** to reverse slightly.

4. **POST_TURN**

   * Reverses straight at **POST_TURN_BACK_SPEED**.
   * Uses **back ToF** AND/OR timeout:

     * If back distance ≥ `POST_TURN_BACK_CLEAR_CM` **or** timeout reached → stop reversing.
   * Returns to **CRUISE** and starts a short settling window where steering is forced straight.

5. **AVOID (Obstacle Avoidance)**

   * Two phases:

     * **back_off:** short reverse with steering away from the locked sign.
     * **forward:** move forward and bend around the sign.
   * While the locked obstacle is visible:

     * Computes a **base steering angle** from obstacle colour & area.
     * Applies additional **x-position steering** so the sign stays on the correct side of the image.
   * Once the locked colour has disappeared for several consecutive frames:

     * Considered “cleared”, and the FSM returns to **CRUISE**.

6. **BLUE-BACKWARD MODE (Emergency)**

   * Runs on top of the FSM when a sign enters the “car box” region.
   * Reverses for a fixed duration, then resets yaw and returns to CRUISE.

---

## Code Structure <a id="code-structure"></a>

1. **Imports & Environment Setup**

   * Ensures correct Python virtual environment.
   * Imports OpenCV, NumPy, Picamera2, PCA9685 driver, smbus2, board/busio, VL53L0X, gpiozero, etc. 

2. **Configuration Parameters**

   * Cropping percentages for active vision area.
   * Speed constants for different modes (cruise, turn, reverse, blue-backward, unpark).
   * HSV ranges for red, green, orange, blue.
   * Hough and Canny parameters for line detection.
   * Yaw control gains, limits, and bias update thresholds.
   * Turn/lap counting constants.

3. **Hardware Setup**

   * LEDs and button (status + start).
   * IMU (MPU6050) init, raw data read, gyro bias measurement.
   * PCA9685 servo and motor initialisation.
   * Picamera2 configuration.
   * ToF sensor bank with multiple XSHUT pins and custom I²C addresses.
   * Ultrasonic sensors via gpiozero.

4. **Utility Functions**

   * Distance reading utilities (`tof_cm`, `ultra_cm`).
   * Contour and shape filtering for obstacles.
   * Line detection and masking (`preprocess_edges`, `detect_line_and_mask`).
   * Hough helper functions for line length and y-projection.
   * IMU yaw-based steering helper (`imu_center_servo`).
   * Debug print helper `dbg()`.

5. **Smart Unpark Functions**

   * `UnPark_L()` and `UnPark_R()`:

     * Yaw-controlled forward turns in two stages.
     * Straightening and yaw reset.
     * Transition into CRUISE.

6. **Finite State Machine & Main Loop**

   * Runtime variables for:

     * FSM state, timers, yaw, drift control.
     * Obstacle lock, avoidance phase, colour streaks.
     * Turn/lap counters, cooldowns, settling timers.
   * Main loop:

     * IMU/yaw update + drift/bias adaptation.
     * Full camera processing for:

       * Cropped active ROI.
       * Lines (blue/orange) and diagonal line mask.
       * Obstacles (red/green) with morphological filtering.
     * ToF + ultrasonic readings.
     * FSM state handling as described above.
     * Final steering angle clamping and servo output.
     * ESC / `q` key support to exit.

7. **Shutdown**

   * On exit:

     * Stops camera and motors.
     * Centers steering.
     * Cleans up ultrasonic sensors and LEDs.

---

## Hardware Setup <a id="hardware-setup"></a>

| Component           | Function / Notes                                      |
| ------------------- | ----------------------------------------------------- |
| Raspberry Pi        | Main controller, runs `2nd_mission_v2.4(1).py`        |
| PCA9685             | PWM driver for motors and steering servo              |
| DC Motors           | Forward/reverse drive via `MOTOR_FWD` / `MOTOR_REV`   |
| Steering Servo      | Controlled on `SERVO_CHANNEL`, 60–120° working range  |
| VL53L0X ToF Sensors | 6 sensors: left, right, front, back, front_l, front_r |
| Ultrasonic Sensors  | Front, left, right; used for safety and walls         |
| MPU6050 (IMU)       | Gyro Z used for yaw tracking                          |
| Button (GPIO 20)    | Start trigger for headless operation                  |
| LEDs (red & green)  | Status indication during startup and run              |
| Picamera2 + Camera  | RGB image feed for obstacle and line detection        |

---

## Configuration <a id="configuration"></a>

All configuration is done via **constants at the top of the script**:

* **Cropping:**

  * `TOP_CROP_PCT`, `BOTTOM_CROP_PCT`, `LEFT_CROP_PCT`, `RIGHT_CROP_PCT`
    Define the **active area** used for vision, allowing quick tuning to camera mounting and FOV.

* **Speed & Behaviour:**

  * `NORMAL_SPEED`, `TURN_MOTOR_SPEED`, `BLUE_BACK_SPEED`, `POST_TURN_BACK_SPEED`, `UNPARK_STRAIGHT_SPEED`, etc.
  * `BLUE_BACK_DURATION`, `POST_BACK_FOLLOW_S`, `AVOID_BACK_DURATION`, `SETTLE_DURATION`.

* **Obstacle Detection & Locking:**

  * `MIN_AREA`, `MAX_AREA`, `OBSTACLE_LOCK_AREA_MIN`, `OBSTACLE_LOCK_Y_FRAC`.
  * `COLOR_HOLD_FRAMES`, `OBSTACLE_CLEAR_FRAMES`.
  * `OBSTACLE_LINE_MARGIN_PX` for priority between lines and obstacles.

* **Line Detection (Floor):**

  * HSV ranges: `ORANGE_LO/H I`, `BLUE_LO/H I`.
  * Hough parameters: `HOUGH_THRESHOLD`, `HOUGH_MIN_LENGTH`, `HOUGH_MAX_GAP`.
  * Trigger thresholds: `BLUE_MIN_LEN_PX`, `LINE_CENTER_BLUE_Y_MIN`, `LINE_CENTER_ORANGE_Y_MIN`.

* **Yaw / IMU:**

  * `YAW_KP_BASE`, `YAW_KP_STRONG`, `YAW_DEADBAND_*`, `SERVO_CORR_LIMIT_*`.
  * `DRIFT_GZ_THRESH`, `BIAS_ALPHA`, `SOFT_DECAY_RATE`.
  * Turn-related yaw thresholds: `TURN_FAILSAFE_MAX_DEG`, `YAW_RESET_AFTER_LEFT`, `YAW_RESET_AFTER_RIGHT`, `YAW_CLAMP_DEG`.

* **Turn & Lap Management:**

  * `TURNS_PER_LAP`, `TOTAL_LAPS`, `TOTAL_TURNS`.
  * `TURN_COOLDOWN_S`, `TURN_MIN_INTERVAL_S`.
  * `TURN_FRONT_MIN_CM`, `TURN_FRONT_MAX_CM`, `FRONT_TRIGGER`.

To retune the robot for a different venue or lighting:

1. Start by adjusting **HSV ranges** and **cropping percentages**.
2. Validate **line detection** (blue/orange) and **obstacle detection** (red/green) visually.
3. Fine-tune **yaw gains** and **speed values** only after vision and distances are reliable.

---

## Vision & Colour Processing <a id="vision--colour-processing"></a>

* Runs on **1280×800 RGB frames** from Picamera2.

* Main steps:

  1. Convert to **BGR** and **HSV**.
  2. **Crop** the active region (per-side percentages).
  3. For lines:

     * Apply HSV threshold for **orange and blue** lines.
     * Canny edges + HoughLinesP.
     * Measure projected y at centre and max length to decide if a turn should be triggered.
  4. For diagonal line exclusion:

     * Detect diagonal segments and build `line_mask_diag`.
     * Use this mask to **exclude diagonal line pixels** from red/green detection.
  5. For obstacles:

     * Create masks for:

       * **Red**: two HSV ranges + removal of orange and pink.
       * **Green**: single HSV range.
     * Morphological open/close to clean masks.
     * Find largest contour for each colour and validate shape (aspect ratio, extent).
     * Compute area and bounding box for size and closeness decisions.

* Additionally:

  * A **“car box”** zone near the bottom of the frame is used to trigger the emergency blue-backward escape when an obstacle is too close to the robot visually.

---

## System Architecture Diagram <a id="system-architecture-diagram"></a>

```text
 ┌──────────────────────────────────────────┐
 │              VISION MODULE               │
 │  - Camera capture (Picamera2)            │
 │  - HSV + masks (red/green/orange/blue)   │
 │  - Diagonal line masking                 │
 │  - Hough line detection                  │
 │  - Obstacle contour & box extraction     │
 └──────────────────────────────────────────┘
                    │
                    ▼
 ┌──────────────────────────────────────────┐
 │           SENSOR FUSION BLOCK            │
 │  - ToF readings (VL53L0X)                │
 │  - Ultrasonic distances                  │
 │  - Gyro (MPU6050) → yaw integration      │
 └──────────────────────────────────────────┘
                    │
                    ▼
 ┌──────────────────────────────────────────┐
 │           FSM / CONTROL CORE             │
 │  SMART UNPARK → CRUISE → TURN →          │
 │  POST_TURN → AVOID (+ blue-backward)     │
 │  - Obstacle lock / priority logic        │
 │  - Line trigger & turn cooldowns         │
 │  - Lap & turn counting                   │
 └──────────────────────────────────────────┘
                    │
                    ▼
 ┌──────────────────────────────────────────┐
 │          ACTUATION LAYER                 │
 │  - Servo steering (PCA9685)              │
 │  - Motor speed and direction             │
 │  - Status LEDs                           │
 └──────────────────────────────────────────┘
```

---

## Mission Path & Behaviour <a id="mission-path--behaviour"></a>

* Start:

  * Press hardware button → smart unpark left/right → enter CRUISE.
* Each lap:

  * Follow corridor, keep straight with yaw + wall safety.
  * When a **blue/orange line** is detected under correct conditions:

    * Perform a 90° yaw-based turn in the chosen global direction.
    * Reverse slightly, then resume cruise.
* Obstacles:

  * If a red/green sign appears:

    * Lock it when close and stable.
    * Execute avoidance: back-off then forward follow around it.
  * If the sign falls into the immediate “car box” region:

    * Trigger blue-backward escape to avoid collision.
* Finish:

  * After **12 turns** (3 laps) the robot goes straight for a fixed time and stops.

---

