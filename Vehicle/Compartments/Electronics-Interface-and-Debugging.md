# Interface and Debugging

The robot includes a set of simple but effective user-interface components designed to support fast testing, clear diagnostics, and reliable operation during competition. These elements help the operator understand the robot’s current state without needing a laptop or external monitor.

---

## 1 Purpose of On-Robot Interface Elements

The interface system provides:

- **Immediate visual feedback** on mode and state  
- **Simple on-device control** without SSH or remote tools  
- **Operational clarity** during competition setups  
- **Fast debugging** when diagnosing navigation or sensor issues  

These features significantly reduce testing time and improve repeatability.

---

## 2 LED Indicators (Status + Debug)

Several LEDs are connected directly to Raspberry Pi GPIO pins.

### Roles of the LEDs:

- **State indication:** cruising, turning, obstacle detected  
- **Lap progression:** LED patterns can represent which lap (1–3) the robot is currently completing  
- **Error signaling:** fast blinking or fixed patterns can indicate sensor failure, IMU freeze, or ToF timeout  
- **Startup sequence:** green LED ready state, red LED indicates system initialization  

### Engineering Notes:

- LEDs are driven through **current-limiting resistors** to protect GPIO pins  
- GPIO selection avoids pins used by I²C/SPI/UART or reserved functions  
- LEDs can be multiplexed or repurposed easily in software for new states  

This lightweight indicator system makes the robot significantly easier to operate without external equipment.

---

## 3 Physical Buttons (Start/Stop Control)

The robot includes tactile push-buttons wired to safe GPIO inputs.

### Button Functions:

- **Start/Run button:** launches the main autonomous routine  
- **Stop/Safe-reset button:** immediately halts motors and resets state  
- **Extra “NOT USED” footprint:** left intentionally for future upgrades (e.g., mode switching, manual override, calibration triggers)

### Engineering Notes:

- Buttons use **pull-up or pull-down resistors** for stable logic levels  
- Debouncing is handled in software (edge detection + delay filter)  
- Buttons allow full control even if SSH, Wi-Fi or GUI is unavailable  

Physical buttons ensure reliable interaction during practice and competition rounds.

---

## 4 Benefits of This Interface Design

- **Zero-laptop operation:** robot can be tested on field with no external devices  
- **Fast feedback:** operator instantly knows what the robot perceives or is doing  
- **Low complexity:** minimal components, low power, high reliability  
- **Future-proofing:** extra pads allow additional buttons, LEDs, or display modules  

---

## 5 Alternative Interface Options Considered

### 1. OLED or LCD Display
- **Pros:** rich debug information, text menus  
- **Cons:** more wiring/space; not needed for WRO where setup time is short  

### 2. Bluetooth or Wi-Fi App-Based Control
- **Pros:** flexible, remote debugging  
- **Cons:** wireless latency, potential interference during competition  

### 3. Rotary encoders or DIP switches
- **Pros:** physical mode selection  
- **Cons:** unnecessary complexity for a small autonomous robot  

### 4. RGB LED bar or NeoPixel strip
- **Pros:** expressive color-coded states  
- **Cons:** requires more power and precise timing  

---

## Summary

The robot’s interface system uses **simple LEDs and buttons** to provide reliable, low-cost, competition-friendly debugging and control. The design supports ease of use, quick iteration, and clear feedback while maintaining spare capacity for future enhancements.

---
