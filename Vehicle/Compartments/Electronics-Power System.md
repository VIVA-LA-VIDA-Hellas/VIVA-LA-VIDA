2. Power System – 3S Li-ion Pack, BMS and LM2596 Buck
For power we use 3 18650 Li-ion cells in series (3S), a BMS, a main switch, a 4 A fuse and an LM2596 buck converter that generates a stable 5 V rail.
•	A 3S pack gives us about 9–12.6 V, which is ideal both for the drive motor and for feeding the buck converter. The capacity of three 18650 cells is more than enough to drive the robot for a match (and for testing before it).
•	The BMS (battery management system) takes care of safety: it protects against over-charge, over-discharge and short circuits, and it balances the cells. That keeps the pack healthy and safe over many charge cycles.
•	The 4 A fuse and the main power switch protects the wiring and the PCB if something goes wrong.
•	The LM2596 provides a stable and efficient 5 V rail for the Raspberry Pi, the sensors, the PCA9685 and the servos.
•	An external power jack lets us power or charge the robot without opening the chassis or disconnecting the battery, which makes maintenance much easier.
