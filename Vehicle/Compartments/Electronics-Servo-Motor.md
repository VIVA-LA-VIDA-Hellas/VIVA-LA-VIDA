5. Steering Servo – TowerPro MG996R
Steering is done with a TowerPro MG996R metal-gear servo (standard size).
Typical specs:
•	Operating voltage: 4.8–6.0 V
•	Stall torque: ≈9.4 kg·cm at 4.8 V, ≈11 kg·cm at 6 V
•	Speed: about 0.15–0.17 s per 60°
•	Running current: 0.5–0.9 A, stall current ≈ 2.5 A
Why we chose it:
•	It offers high torque, even at 5 V, so it can turn the wheels under load and hold the steering angle even if the robot is pushed against a wall.
•	It uses the standard RC servo interface (50–60 Hz, 1–2 ms pulses). The PCA9685 generates these signals with 12-bit resolution, which lets us set steering angles very precisely without worrying about timing on the Pi.
•	It’s common and robust: metal gears, ball bearings and standard mounting holes make it easy to integrate mechanically and strong enough for repeated sharp turns.
•	Our software assumes a centre angle of 90° and uses roughly 50–130° for strong left and right turns. The MG996R comfortably supports this range, so we get both tight turns around corners and fine corrections from the IMU controller.
On the PCB, the MG996R is connected to PCA9685 channel 0 with power from the 5 V servo rail and ground on the common plane. That keeps its high current separate from the 3.3 V logic, while staying on the same timing domain as any future servos.
