import time

# Patch missing attribute in adafruit_platformdetect
import adafruit_platformdetect.constants.chips as chips
if not hasattr(chips, 'JH71x0'):
    chips.JH71x0 = 'JH71x0'

import board
import busio
from adafruit_pca9685 import PCA9685

# Use your default I2C or specify alternate pins if needed:
i2c = busio.I2C(board.I2C_2_SCL, board.I2C_2_SDA)
# Alternatively, if your default works, use:
# i2c = board.I2C()

pca = PCA9685(i2c)
pca.frequency = 50  # Set to 50 Hz (20ms period)

def speed_to_pwm(speed):
    """
    Convert a speed (-1.0 to 1.0) into a PCA9685 duty_cycle count.
    
    - For speed = -1.0: pulse width ~1.0 ms (full reverse)
    - For speed = 0.0: pulse width ~1.5 ms (neutral)
    - For speed = 1.0: pulse width ~2.0 ms (full forward)
    """
    counts_per_ms = 4096 / 20.0  # ~204.8 counts per ms

    pulse_min = 1.0      # ms (full reverse)
    pulse_neutral = 1.5  # ms (neutral)
    pulse_max = 2.0      # ms (full forward)

    count_min = int(pulse_min * counts_per_ms)
    count_neutral = int(pulse_neutral * counts_per_ms)
    count_max = int(pulse_max * counts_per_ms)

    if speed < 0:
        pwm_count = count_neutral + int((count_neutral - count_min) * speed)
    else:
        pwm_count = count_neutral + int((count_max - count_neutral) * speed)

    pwm_count = max(min(pwm_count, count_max), count_min)
    return pwm_count

motor_channel = 15

print("Enter a speed value between -1.0 (full reverse) and 1.0 (full forward).")
try:
    while True:
        user_input = input("Enter speed: ")
        try:
            speed = float(user_input)
            if speed < -1.0 or speed > 1.0:
                print("Speed must be between -1.0 and 1.0.")
                continue
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            continue

        pwm_value = speed_to_pwm(speed)
        pca.channels[motor_channel].duty_cycle = pwm_value
        print("Speed set to {}, PWM count: {}".format(speed, pwm_value))
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pca.deinit()
