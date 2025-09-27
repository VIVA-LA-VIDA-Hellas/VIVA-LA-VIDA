from adafruit_pca9685 import PCA9685

# Example motor and servo control functions using a shared PCA9685 instance

def set_motor_speed(pca, motor_channel_fwd, motor_channel_rev, speed):
    """
    Set motor speed using two channels (forward/reverse) on PCA9685.
    speed: -100 to 100 (negative for reverse)
    """
    # Map speed to PWM duty cycle (0-65535)
    duty_cycle = int(min(max(abs(speed), 0), 100) / 100 * 65535)
    if speed > 0:
        pca.channels[motor_channel_fwd].duty_cycle = duty_cycle
        pca.channels[motor_channel_rev].duty_cycle = 0
    elif speed < 0:
        pca.channels[motor_channel_fwd].duty_cycle = 0
        pca.channels[motor_channel_rev].duty_cycle = duty_cycle
    else:
        pca.channels[motor_channel_fwd].duty_cycle = 0
        pca.channels[motor_channel_rev].duty_cycle = 0


def set_servo_angle(pca, servo_channel, angle):
    """
    Set servo angle on PCA9685 channel.
    angle: 0 to 180 degrees
    """
    # Restrict angle to min/max range
    min_angle = 50
    max_angle = 130
    angle = max(min_angle, min(max_angle, angle))
    # Map angle to pulse length (for 50Hz, 1ms-2ms typical)
    pulse_min = 1000  # 1ms
    pulse_max = 2000  # 2ms
    pulse = int(pulse_min + (pulse_max - pulse_min) * ((angle - min_angle) / (max_angle - min_angle)))
    pca.channels[servo_channel].duty_cycle = int(pulse * 65535 / 20000)  # 20ms period
