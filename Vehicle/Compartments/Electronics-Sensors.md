## 6. Sensors – Why This Combination

The sensor set is designed to give the robot all the information it needs to drive autonomously, as it provides real time data related to distances around it.

---
### Distance metering sensors

### a) Time-of-Flight (ToF) Sensors

- Several ToF modules are connected via **I²C** and powered from a dedicated **5 V sensor rail**.  
- They provide very accurate short-range distance (millimetre resolution), perfect for close to wall triggers while keeping precise gaps to obstacles between **2–100 cm**.  
- They also handle distance metering to dark surfaces better than many ultrasonic sensors and there is no issue if the sensor reading direction is facing a wall under an ungle. 
- There are six (6) ToF Sensors positioned on the robot:
           Front
  Left Front   Right Front
  Left               Right
           Back

---

### b) Ultrasonic Sensors

- Several ultrasonic sensors are included, with **resistor dividers** on the echo lines so their 5V signals are safe for the Pi’s 3.3 V GPIOs.  
- Ultrasonics have **longer range** and a **wider cone** than ToF, so they are great for early detection of walls and obstacles further away.
- There are three (3) Ultrasonic Sensors positioned on the robot:
           Front
  Left               Right

  
- By combining **ToF + ultrasonic**, the robot gets:
  - short-range precision  
  - long-range awareness  
  - redundancy when one type is affected by noise or reflections  

---

### Acceleration and rotation sensors

### c) IMU – GY-521 (MPU6050)

- Provides **gyroscope** and **accelerometer** data over I²C.  
- Mainly used for **yaw estimation**, keeping the robot driving straight even with wheel slip, and enabling smooth, repeatable turns.  
- Because the robot does **not use wheel encoders**, the IMU is essential for robust navigation on different floor surfaces.

---

### d) Optional / “Not Used” Sensors

- The PCB includes footprints for:
  - colour sensor  
  - small OLED display  
  - extra button  
  - buzzer  
  - DIP switch  
  - LIDAR  
- These are currently marked as **NOT USED**, but leave space for future upgrades—such as colour-based tasks, on-board debugging, or advanced mapping—without redesigning the PCB.
