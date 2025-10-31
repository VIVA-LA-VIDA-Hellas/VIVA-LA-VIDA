
# VivaLaVida WRO 2025 — Headless Functionality Checklist

**Legend:** ✅ = kept • ❌ = removed

## 1) Startup & configuration
- ❌ **1.1** Load defaults + JSON overrides
- ✅ **1.2** Global DEBUG prints toggle
- ✅ **1.3** Constants for speeds, margins, thresholds, laps, etc.

## 2) Hardware setup
- ✅ **2.1** GPIO mode, pins, START button input
- ✅ **2.2** Status LEDs (GREEN/RED)
- ✅ **2.3** PCA9685 init (50 Hz) for motors + steering servo
- ✅ **2.4** MPU6050 init

## 3) Sensors
- ✅ **3.1** Ultrasonic (front/left/right) via `gpiozero.DistanceSensor`
- ✅ **3.2** ToF (VL53L0X) support, XSHUT addressing, back ToF
- ✅ **3.3** Sensor reading thread with lock/events

## 4) Sensor filtering
- ✅ **4.1** Median over last N readings
- ✅ **4.2** EMA smoothing (alpha)
- ✅ **4.3** Spike/jump rejection (max jump)

## 5) Gyro handling
- ✅ **5.1** Startup bias calibration (averaging N samples)
- ✅ **5.2** Yaw integration (deg) with simple LPF on gyro Z
- ✅ **5.3** `snap90()` + `norm180()` helpers

## 6) Steering & drive
- ❌ **6.1** Servo command with slew limiter (max dps)
- ✅ **6.2** Motor PWM (forward/reverse channels)

## 7) Environment modes
- ✅ **7.1** Narrow corridor logic (sum L+R with hysteresis; scales speed/thresholds)

## 8) Wall following (straight driving)
- ✅ **8.1** Safe straight control using soft margin & max correction
- ✅ **8.2** Time-boxed correction windows (duration, renew)

## 9) Turn detection & decision
- ✅ **9.1** Front trigger distance to enter TURN_INIT
- ✅ **9.2** Side-open XOR decision (only one side open)
- ✅ **9.3** Turn direction lock after first valid direction
- ✅ **9.4** Turn lockout time between turns

## 10) Turning execution
- ✅ **10.1** Entry skew compensation: dynamic target = ±TARGET − entry_skew
- ✅ **10.2** Turn stop conditions: angle tolerance, timeout, max angle
- ✅ **10.3** Post-turn straight drive for X seconds

## 11) Safety / stops
- ✅ **11.1** Emergency front stop
- ✅ **11.2** Obstacle wait & auto-retry after `OBSTACLE_WAIT_TIME`
- ✅ **11.3** Hard stop on user stop / laps end

## 12) Laps & bookkeeping
- ✅ **12.1** Turn count, lap count, max laps + final post-lap drive
- ❌ **12.2** Status text (for logs)

## 13) Threading & timing
- ✅ **13.1** `readings_event`, `loop_event`, `sensor_tick` events
- ✅ **13.2** Loop delays

## 14) Headless bootstrap
- ✅ **14.1** LED ready indication
- ✅ **14.2** Wait for START button press, then run loop
- ✅ **14.3** KeyboardInterrupt cleanup

## 15) Telemetry & data
- ❌ **15.1** Deques for plotting
- ❌ **15.2** CSV export of telemetry
- ✅ **15.3** DEBUG prints with key events

## 16) GUI-only (remove for headless)
- ❌ **16.1** Tkinter window, buttons, status circle
- ❌ **16.2** Matplotlib plots & live annotations
- ❌ **16.3** Sliders to tune parameters live
- ❌ **16.4** Save/Load sliders via file dialogs
