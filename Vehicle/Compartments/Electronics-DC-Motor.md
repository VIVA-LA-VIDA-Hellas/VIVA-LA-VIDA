4. DC Drive Motor – Metal Gearmotor 25 mm, 210 RPM, 9–12 V
For acceleration we use a 25 mm metal gearmotor (210 rpm at 12 V).
Key specs: 9–12 V rated, 34:1 steel gearbox, 4 mm shaft, ~70 mA no-load current, about 2.1 A stall current and around 4.5 kg·cm stall torque.
Why this motor fits the robot:
•	The voltage range (9–12 V) matches our 3S Li-ion battery almost perfectly, so we can drive it directly from Vbat through the DRV8871 without extra converters. 
•	The stall current (~2.1 A) is comfortably inside the DRV8871’s peak rating and below our 4 A system fuse. We can set the driver’s current limit near 2 A to protect both the motor and the PCB while still giving good acceleration.
•	A speed of around 200 rpm at 12 V gives a practical linear speed (roughly 0.6–0.8 m/s with our wheel size). Using PWM we can slow down precisely near obstacles or during turns, while still having enough top speed to complete three laps in time.
•	Mechanically, the 25 mm diameter and 4 mm shaft fit our chassis, wheels and couplers with minimal custom work, leading to a compact and robust drivetrain.
On the PCB, the motor connects to the DRV8871 (U10) through the Motor connector (TB1). The driver is powered from Vbat and controlled by Raspberry Pi GPIOs. This gives us a clean, single motor power channel that is easy to understand and debug.
