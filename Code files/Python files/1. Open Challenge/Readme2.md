
Autonomous drive 1st Challenge - WRO 2025 FE
VivaLaVida

Purpose: 
        • Autonomous wall-following for WRO 2025 track 
        • Architecture: threaded sensor reader + finite-state machine (IDLE/CRUISE/TURN_INIT/TURNING/POST_TURN/STOPPED)
Sensors: 
        • Mix-and-match ToF (VL53L0X) & Ultrasonic (gpiozero) 
        • Filtering: median + spike rejection + optional EMA-style smoothing to stabilize cm readings
Orientation & turning: 
        • Gyro biasing + yaw integration guides turns 
        • Turn logic: front trigger, XOR side-open decision, optional direction lock, servo slew limiting
Environment handling: 
        • Narrow-corridor detector scales speeds/thresholds • 
Safety: 
        • Immediate stop on close front, timed obstacle retry, lap counting with final stop
UI & ops: 
        • Degugging dashboard with live plots/status, sliders for key params, JSON load/Save, CSV export, GUI or headless start via button

______________________________________________________________________________________________________________________
🚀 Code Structure:

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
______________________________________________________________________________________________________________________
