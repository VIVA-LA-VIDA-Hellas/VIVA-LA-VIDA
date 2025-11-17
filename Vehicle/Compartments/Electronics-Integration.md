8. System Integration and Design Philosophy
A few general choices guide the whole design:
•	I²C as the main sensor bus: Most smart devices (ToFs, IMU, PCA9685, OLED, colour sensor) share the I²C bus. This keeps wiring clean and makes it easy to add or remove modules.
•	Separation of 5 V and 3.3 V domains: The Raspberry Pi logic is 3.3 V, while many peripherals need 5 V power. We protect the Pi using level-safe connections and resistor dividers so the GPIOs stay within their limits.
•	Clearly labelled connectors: Headers for ultrasonics, ToF sensors, LIDAR and the motor are marked clearly so building and repairing the robot at a competition is quick.
•	Expansion pads: Spare pads for extra signals mean we can extend the robot later without redesigning the board.
