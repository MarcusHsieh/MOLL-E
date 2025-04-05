import time
import smbus

# PCA9685 registers and address
PCA9685_ADDRESS = 0x40
MODE1   = 0x00
MODE2   = 0x01
PRE_SCALE = 0xFE
LED0_ON_L = 0x06

# Create an SMBus instance for I2C bus 1 (Jetson Nano)
bus = smbus.SMBus(1)

def reset():
    """Reset the PCA9685 device."""
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, 0x00)
    time.sleep(0.005)

def set_pwm_freq(freq):
    """
    Set the PWM frequency for the PCA9685.
    For a 25MHz oscillator, the prescale value is calculated as:
    
      prescale = round(25e6 / (4096 * freq)) - 1
    """
    prescaleval = 25000000.0  # 25 MHz
    prescaleval /= 4096.0
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = int(round(prescaleval))

    # Read the current MODE1 register value
    oldmode = bus.read_byte_data(PCA9685_ADDRESS, MODE1)
    # Enter sleep mode to set prescale
    newmode = (oldmode & 0x7F) | 0x10   # set sleep bit
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, newmode)
    # Write the prescale value
    bus.write_byte_data(PCA9685_ADDRESS, PRE_SCALE, prescale)
    # Restore the old mode and wait for oscillator to stabilize
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, oldmode)
    time.sleep(0.005)
    # Restart the device
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, oldmode | 0x80)

def set_pwm(channel, on, off):
    """
    Set the PWM output for a specific channel.
    The PCA9685 has four registers per channel:
      LEDn_ON_L, LEDn_ON_H, LEDn_OFF_L, LEDn_OFF_H
    """
    base = LED0_ON_L + 4 * channel
    bus.write_byte_data(PCA9685_ADDRESS, base, on & 0xFF)
    bus.write_byte_data(PCA9685_ADDRESS, base+1, (on >> 8) & 0xFF)
    bus.write_byte_data(PCA9685_ADDRESS, base+2, off & 0xFF)
    bus.write_byte_data(PCA9685_ADDRESS, base+3, (off >> 8) & 0xFF)

def speed_to_pwm(speed):
    """
    Convert a speed value (-1.0 to 1.0) into a 12-bit PWM count.
    
    Calibrated mapping:
      -1.0 --> ~1.1 ms pulse (full reverse)
       0.0 --> ~1.6 ms pulse (neutral)
       1.0 --> ~2.1 ms pulse (full forward)
       
    With 4096 counts over a 20 ms period (50Hz), each ms is roughly 204.8 counts.
    """
    # Calibrated pulse widths (in milliseconds)
    pulse_min = 1.1      # full reverse pulse width
    pulse_neutral = 1.6  # neutral pulse width
    pulse_max = 2.1      # full forward pulse width

    counts_per_ms = 4096 / 20.0  # ~204.8 counts per ms

    count_min = int(pulse_min * counts_per_ms)
    count_neutral = int(pulse_neutral * counts_per_ms)
    count_max = int(pulse_max * counts_per_ms)

    if speed < 0:
        # For negative speeds, interpolate from neutral down to count_min
        pwm_count = count_neutral + int((count_neutral - count_min) * speed)
    else:
        # For positive speeds, interpolate from neutral up to count_max
        pwm_count = count_neutral + int((count_max - count_neutral) * speed)

    # Clamp the pwm_count to the valid range
    pwm_count = max(min(pwm_count, count_max), count_min)
    return pwm_count

# Initialize PCA9685
reset()
set_pwm_freq(50)  # Set frequency to 50Hz (20ms period)

# Choose the motor channel (0 to 15)
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
        set_pwm(motor_channel, 0, pwm_value)
        print("Speed set to {}, PWM count: {}".format(speed, pwm_value))
except KeyboardInterrupt:
    print("Exiting...")
