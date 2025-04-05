#!/usr/bin/env python3
import time
import smbus
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

    oldmode = bus.read_byte_data(PCA9685_ADDRESS, MODE1)
    newmode = (oldmode & 0x7F) | 0x10   # set sleep bit
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, newmode)
    bus.write_byte_data(PCA9685_ADDRESS, PRE_SCALE, prescale)
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, oldmode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDRESS, MODE1, oldmode | 0x80)

def set_pwm(channel, on, off):
    """
    Set the PWM output for a specific channel.
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
    """
    pulse_min     = 1.1      # full reverse pulse width (ms)
    pulse_neutral = 1.6      # neutral pulse width (ms)
    pulse_max     = 2.1      # full forward pulse width (ms)

    counts_per_ms = 4096 / 20.0  # ~204.8 counts per ms (50Hz period)

    count_min     = int(pulse_min * counts_per_ms)
    count_neutral = int(pulse_neutral * counts_per_ms)
    count_max     = int(pulse_max * counts_per_ms)

    if speed < 0:
        pwm_count = count_neutral + int((count_neutral - count_min) * speed)
    else:
        pwm_count = count_neutral + int((count_max - count_neutral) * speed)

    pwm_count = max(min(pwm_count, count_max), count_min)
    return pwm_count

# Motor channel assignments (adjust according to your wiring)
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

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        # Initialize PCA9685
        reset()
        set_pwm_freq(50)  # Set frequency to 50Hz (20ms period)
        self.get_logger().info("PCA9685 initialized at 50Hz.")
        
        # Subscribe to velocity commands (Twist messages)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_filtered',
            self.cmd_callback,
            10)
        self.get_logger().info("Subscribed to /cmd_vel_filtered.")

    def cmd_callback(self, msg):
        # Here we assume a simple tank drive:
        # For example, we can combine linear and angular components.
        # You may adjust the mixing based on your robot's kinematics.
        left_speed  = msg.linear.x + msg.angular.z
        right_speed = msg.linear.x - msg.angular.z
        set_left_speed(left_speed)
        set_right_speed(right_speed)
        self.get_logger().info(f"Set left speed: {left_speed:.2f}, right speed: {right_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motor controller node...")
    set_left_speed(0.0)
    set_right_speed(0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
