'''accel,gyro,dis,colour sensors'''
import time
import RPi.GPIO as GPIO

import vl53l0x_module
import tcs34725_module
import mpu6050_module
import pca9685_module
import oled_module

GPIO.setmode(GPIO.BCM)
#  GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


vl53l0x_module.setup_vl53l0x()
tcs34725_module.setup_tcs34725()
mpu6050_module.setup_mpu6050()


try:
    while True:     
            
        d1, d2, d3, d4 = vl53l0x_module.get_distances()
        print(f" D1={d1} | D2={d2} | D3={d3} | D4={d4}")    
        text = f"D1:{d1} D2:{d2}\nD3:{d3} D4:{d4}"
        oled_module.display_text(text, size="small")
        # TCS34725
        r, g, b = tcs34725_module.get_rgb()
        print(f"\n R={r}  |  G={g}  |  B={b}\n")

        # MPU6050
        mpu_data = mpu6050_module.get_mpu_data()
        print("A   (g): X={:.2f}, Y={:.2f}, Z={:.2f}".format(*mpu_data['accel']))
        print("G (°/s): X={:.2f}, Y={:.2f}, Z={:.2f}\n".format(*mpu_data['gyro']))
        
        pca9685_module.set_servo_angle(0, 48)
        pca9685_module.set_motor(1, 2,50)
        


except KeyboardInterrupt:
    print("Stopping...")
finally:
    GPIO.cleanup()

