Autonomous drive 1st Mission WRO 2025
- VivaLaVida - 

v4.1 - remove camera, add logging
v4.2 - improved turn logic and timeout slider
v4.3 - graph and turn improvements
v4.5 - rewrite
v4.5 - new turn logic with PID clear after turn
v4.6 - new wall following rules
v4.7 - new GUI, rewrite
v4.8 - added gyroscope
v4.9 - added sliders, max lap, save/load slide values, csv export with data
v4.92 - check servo correction during cruise
v4.93 - add lap reset button (stop loop), slider name correction
v4.94 - Sensor readings filtering improvements, read in sequence. Lock turn side. Improve driving between walls with new rules.
v4.95 - Code cleaning, turn based on gyroscope
v4.96 - add/remove corrections
v4.98 - turn initiation stop logic (merge from v4.7-4.8 plus extra conditions), added tof
v4.98FSM - finite state machine (FSM)
v4.99 - option to run w/o GIO

v5.0 - Gold version
v5.1 - minor fixes
v5.2 - threaded, low-latency sensor reading, Exponential Moving Average (EMA) layer, turn initiation logic
v5.3:
     - fix stop loop to stop the motor (change at RobotState.IDLE), 
     - added timing and continuous reading in tof initialization and also changed tof settings on closing, 
     - added linear accelaration in RobotState.IDLE during start, 
     - def sensor_reader reduced sleep (0.005 from 0.01)
v5.4:
     - gpiozero, 
     - added SENSOR_DELAY variable, 
     - speed factor per state
     - last_turn_time set to a high value to enable initial turn
     - Removed logic to end turn based on front/side distance
v5.5:
     - add numners to plots
     - changes def turn_decision
v5.51:
     - fixed start readings button 
v5.52:
     - code cleaning
v5.53:
     - factor to change variables if in between narrow walls
v5.6:
     - code cleaning
     - bugs to fix: turn init speed probably not working, check speed sliders if ok, check BASE speeds if needed
v5.61:
     - efficiency improvements (Events + responsive waits, High-precision timing with monotonic_ns
v5.62:
     - stop due to obstacle (wait for clear front to continue)
     - code cleaning
v5.63:
     - Turn init state/speed fix
     - Variable to adjust servo speed
v5.64:
     - load variables from json file
v5.7:
     - code cleaning
v5.8:
     - performance improvements TBD
     - turning improvements (yaw calculation)
v5.9:
     - fix tof readings
v5.91:
     - minor improvements
v5.92:
     - change safe straight control logic
     - code cleaning
v5.93:
     - add front side ToF sensors
     















