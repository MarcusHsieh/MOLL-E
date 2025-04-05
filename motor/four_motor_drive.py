import time
import smbus

# PCA9685 registers and address
PCA9685_ADDRESS = 0x40
MODE1      = 0x00
MODE2      = 0x01
PRE_SCALE  = 0xFE
LED0_ON_L  = 0x06

# Create an SMBus instance for I2C bus 1
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
    pulse_min     = 1.1      # full reverse pulse width
    pulse_neutral = 1.6      # neutral pulse width
    pulse_max     = 2.1      # full forward pulse width

    counts_per_ms = 4096 / 20.0  # ~204.8 counts per ms

    count_min     = int(pulse_min * counts_per_ms)
    count_neutral = int(pulse_neutral * counts_per_ms)
    count_max     = int(pulse_max * counts_per_ms)

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

# Left side: channels 12 (top left) and 13 (bottom left)
# Right side: channels 14 (top right) and 15 (bottom right)
left_top    = 12
left_bottom = 13
right_top   = 14
right_bottom= 15

def set_left_speed(speed):
    """Set both left motors to a given speed (-1.0 to 1.0)."""
    pwm_val = speed_to_pwm(speed)
    set_pwm(left_top, 0, pwm_val)
    set_pwm(left_bottom, 0, pwm_val)

def set_right_speed(speed):
    """Set both right motors to a given speed (-1.0 to 1.0)."""
    pwm_val = speed_to_pwm(speed)
    set_pwm(right_top, 0, pwm_val)
    set_pwm(right_bottom, 0, pwm_val)

print("Tank Drive Commands:")
print("  f - Forward (prompt for magnitude 0.0 to 1.0)")
print("  b - Backward (prompt for magnitude 0.0 to 1.0)")
print("  r - Rotate right (left forward, right backward; prompt for magnitude 0.0 to 1.0)")
print("  l - Rotate left (left backward, right forward; prompt for magnitude 0.0 to 1.0)")
print("  v - Set both sides to a specific speed (-1.0 to 1.0)")
print("  s - Stop (neutral)")
print("  q - Quit")

try:
    while True:
        cmd = input("\nEnter command (f, b, r, l, v, s, q): ").strip().lower()
        if cmd == 'q':
            break

        if cmd == 's':
            # Stop motors (neutral)
            set_left_speed(0.0)
            set_right_speed(0.0)
            print("Motors stopped (neutral).")
            continue

        if cmd == 'v':
            # Set both sides to a given speed (-1.0 to 1.0)
            try:
                speed = float(input("Enter speed value (-1.0 to 1.0): ").strip())
            except ValueError:
                print("Invalid input. Using 0.0 as default.")
                speed = 0.0
            if speed < -1.0 or speed > 1.0:
                print("Speed out of range. Clamping to [-1.0, 1.0].")
                speed = max(min(speed, 1.0), -1.0)
            left_speed = speed
            right_speed = speed
        else:
            # For drive commands f, b, r, l, prompt for magnitude (0.0 to 1.0)
            try:
                mag = float(input("Enter speed magnitude (0.0 to 1.0): ").strip())
            except ValueError:
                print("Invalid magnitude. Defaulting to 0.5.")
                mag = 0.5
            if mag < 0.0 or mag > 1.0:
                print("Magnitude out of range. Using 0.5.")
                mag = 0.5

            if cmd == 'f':
                # Forward: both sides positive
                left_speed = mag
                right_speed = mag
            elif cmd == 'b':
                # Backward: both sides negative
                left_speed = -mag
                right_speed = -mag
            elif cmd == 'r':
                # Rotate right: left motors forward, right motors backward
                left_speed = mag
                right_speed = -mag
            elif cmd == 'l':
                # Rotate left: left motors backward, right motors forward
                left_speed = -mag
                right_speed = mag
            else:
                print("Invalid command. Please try again.")
                continue

        # Set motor speeds accordingly
        set_left_speed(left_speed)
        set_right_speed(right_speed)
        print("Left speed: {} (PWM: {}), Right speed: {} (PWM: {})".format(
            left_speed, speed_to_pwm(left_speed),
            right_speed, speed_to_pwm(right_speed)))
except KeyboardInterrupt:
    print("\nExiting...")

set_left_speed(0.0)
set_right_speed(0.0)
