6. Sensors – Why This Combination
The sensor set is designed to give the robot all the information it needs to drive autonomously, without drowning us in complexity.
a) Time-of-Flight (ToF) Sensors
•	Several ToF modules are connected via I²C and powered from a dedicated 5 V sensor rail.
•	They provide very accurate short-range distance (millimetre resolution), which is perfect for wall-following and keeping precise gaps to obstacles between about 2 and 100 cm.
•	They also handle dark or angled surfaces better than many ultrasonic sensors.
b) Ultrasonic Sensors
•	We include several ultrasonic sensors, with resistor dividers on the echo lines so their 5 V signals are safe for the Pi’s 3.3 V GPIOs.
•	Ultrasonics have longer range and a wider cone than ToF, so they are great for early detection of walls and obstacles further away.
•	By combining ToF + ultrasonic, the robot gets short-range precision and long-range awareness, plus redundancy if one type of sensor is affected by noise or surface reflections.
c) IMU – GY-521 (MPU6050)
•	The IMU gives us gyroscope and accelerometer data over I²C.
•	We mainly use it for yaw estimation: keeping the robot driving straight even if the wheels slip, and performing smoother, repeatable turns.
•	Because we don’t use wheel encoders, the IMU is a key part of having robust navigation on different floor surfaces.
d) Optional / “Not Used” Sensors
•	The PCB also has footprints for a colour sensor, small OLED display, extra button, buzzer, DIP switch and LIDAR.
•	At the moment these are marked as “NOT USED”, but they give us space to experiment with colour-based tasks, on-board debugging displays or more advanced mapping in future versions, without redesigning the board.
