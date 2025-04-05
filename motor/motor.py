import time
import board
import busio
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Convert a speed command (-1.0 to 1.0) to the corresponding PWM count.
def speed_to_pwm(speed):
    """
    Convert a speed (-1.0 to 1.0) into a PCA9685 duty_cycle count.
    
    - For speed = -1.0: pulse width should be ~1.0 ms.
    - For speed = 0.0: pulse width ~1.5 ms (neutral).
    - For speed = 1.0: pulse width ~2.0 ms.
    """
    # Calculate counts per ms. Total period is 20ms and resolution is 4096.
    counts_per_ms = 4096 / 20.0  # ~204.8 counts per ms

    # Define pulse widths in ms
    pulse_min = 1.0    # ms (full reverse)
    pulse_neutral = 1.5  # ms (neutral)
    pulse_max = 2.0    # ms (full forward)

    # Calculate corresponding counts for min, neutral, and max
    count_min = int(pulse_min * counts_per_ms)
    count_neutral = int(pulse_neutral * counts_per_ms)
    count_max = int(pulse_max * counts_per_ms)

    if speed < 0:
        # Map speed from [-1.0, 0] to [count_min, count_neutral]
        pwm_count = count_neutral + int((count_neutral - count_min) * speed)
    else:
        # Map speed from [0, 1.0] to [count_neutral, count_max]
        pwm_count = count_neutral + int((count_max - count_neutral) * speed)
    
    pwm_count = max(min(pwm_count, count_max), count_min)
    return pwm_count

# CONTROL THIS MOTOR
# Set motor channel (0-15 for PCA9685)
motor_channel = 0

# Main loop: receive command and update PWM signal
print("Enter a speed value between -1.0 (full reverse) and 1.0 (full forward).")
try:
    while True:
        # ***Replace this with input from control system later
        user_input = input("Enter speed: ")
        try:
            speed = float(user_input)
            if speed < -1.0 or speed > 1.0:
                print("Speed must be between -1.0 and 1.0.")
                continue
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue

        pwm_value = speed_to_pwm(speed)
        pca.channels[motor_channel].duty_cycle = pwm_value

        print(f"Speed set to {speed}, PWM count: {pwm_value}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pca.deinit()
