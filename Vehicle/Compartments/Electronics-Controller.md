1. Controller – Raspberry Pi 5
We chose the Raspberry Pi 5 as the main controller because it offers a strong balance between processing power, I/O flexibility, and library support.
•	It’s powerful enough to run Linux, Python, OpenCV, sensor fusion and logging at the same time, which we need for camera processing and more advanced control.
•	Its 40-pin header gives us everything we need in one place: I²C for the IMU, ToF sensors, PCA9685, OLED and colour sensor, and GPIOs for PWM outputs, LEDs and buttons.
•	Almost every module we use already has a ready-made Python library (VL53L0X, HC-SR04, MPU6050, PCA9685, etc.), so we spend our time tuning the robot instead of fighting low-level drivers.
•	The Pi 5 runs from 5 V, which matches the 5 V rail from our DC-DC converter and keeps the power design straightforward.
