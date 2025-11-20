#!/usr/bin/env python3
"""
Example Hardware Motor Driver for Raspberry Pi 5
This node subscribes to motor speed commands and controls GPIO pins
to drive motors via an L298N motor driver board.

Hardware connections (adjust pin numbers as needed):
- GPIO 12 (Pin 32): Left Motor PWM
- GPIO 16 (Pin 36): Left Motor Direction 1
- GPIO 20 (Pin 38): Left Motor Direction 2
- GPIO 13 (Pin 33): Right Motor PWM
- GPIO 19 (Pin 35): Right Motor Direction 1
- GPIO 26 (Pin 37): Right Motor Direction 2

Installation:
    pip3 install lgpio

Usage:
    python3 hardware_motor_driver.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# Try to import lgpio (for Raspberry Pi 5)
try:
    import lgpio
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: lgpio not available. Running in simulation mode.")
    print("To install: pip3 install lgpio")


class HardwareMotorDriver(Node):
    def __init__(self):
        super().__init__('hardware_motor_driver')
        
        # Motor control parameters
        self.declare_parameter('left_motor_pwm_pin', 12)
        self.declare_parameter('left_motor_dir1_pin', 16)
        self.declare_parameter('left_motor_dir2_pin', 20)
        self.declare_parameter('right_motor_pwm_pin', 13)
        self.declare_parameter('right_motor_dir1_pin', 19)
        self.declare_parameter('right_motor_dir2_pin', 26)
        self.declare_parameter('pwm_frequency', 1000)  # Hz
        
        # Get parameters
        self.left_pwm = self.get_parameter('left_motor_pwm_pin').value
        self.left_dir1 = self.get_parameter('left_motor_dir1_pin').value
        self.left_dir2 = self.get_parameter('left_motor_dir2_pin').value
        self.right_pwm = self.get_parameter('right_motor_pwm_pin').value
        self.right_dir1 = self.get_parameter('right_motor_dir1_pin').value
        self.right_dir2 = self.get_parameter('right_motor_dir2_pin').value
        self.pwm_freq = self.get_parameter('pwm_frequency').value
        
        # Initialize GPIO if available
        self.gpio_handle = None
        if GPIO_AVAILABLE:
            try:
                self.gpio_handle = lgpio.gpiochip_open(0)
                self.setup_gpio()
                self.get_logger().info('GPIO initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize GPIO: {str(e)}')
                self.gpio_handle = None
        else:
            self.get_logger().warn('Running in simulation mode (GPIO not available)')
        
        # Subscribe to motor speed topics
        self.left_motor_sub = self.create_subscription(
            Float32,
            'left_motor_speed',
            self.left_motor_callback,
            10
        )
        
        self.right_motor_sub = self.create_subscription(
            Float32,
            'right_motor_speed',
            self.right_motor_callback,
            10
        )
        
        self.get_logger().info('Hardware Motor Driver started')
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        if not self.gpio_handle:
            return
            
        # Configure PWM pins as outputs
        lgpio.gpio_claim_output(self.gpio_handle, self.left_pwm)
        lgpio.gpio_claim_output(self.gpio_handle, self.right_pwm)
        
        # Configure direction pins as outputs
        lgpio.gpio_claim_output(self.gpio_handle, self.left_dir1)
        lgpio.gpio_claim_output(self.gpio_handle, self.left_dir2)
        lgpio.gpio_claim_output(self.gpio_handle, self.right_dir1)
        lgpio.gpio_claim_output(self.gpio_handle, self.right_dir2)
        
        # Initialize all pins to low
        lgpio.gpio_write(self.gpio_handle, self.left_dir1, 0)
        lgpio.gpio_write(self.gpio_handle, self.left_dir2, 0)
        lgpio.gpio_write(self.gpio_handle, self.right_dir1, 0)
        lgpio.gpio_write(self.gpio_handle, self.right_dir2, 0)
        
    def left_motor_callback(self, msg):
        """Control left motor based on speed command"""
        speed = msg.data
        self.set_motor_speed('left', speed)
        
    def right_motor_callback(self, msg):
        """Control right motor based on speed command"""
        speed = msg.data
        self.set_motor_speed('right', speed)
        
    def set_motor_speed(self, motor, speed):
        """
        Set motor speed and direction
        
        Args:
            motor (str): 'left' or 'right'
            speed (float): Speed value from -1.0 (full reverse) to 1.0 (full forward)
        """
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))
        
        # Select appropriate pins
        if motor == 'left':
            pwm_pin = self.left_pwm
            dir1_pin = self.left_dir1
            dir2_pin = self.left_dir2
        else:  # right
            pwm_pin = self.right_pwm
            dir1_pin = self.right_dir1
            dir2_pin = self.right_dir2
        
        # Convert speed to duty cycle (0-100%)
        duty_cycle = int(abs(speed) * 100)
        
        if self.gpio_handle:
            try:
                # Set direction
                if speed > 0:
                    # Forward
                    lgpio.gpio_write(self.gpio_handle, dir1_pin, 1)
                    lgpio.gpio_write(self.gpio_handle, dir2_pin, 0)
                elif speed < 0:
                    # Reverse
                    lgpio.gpio_write(self.gpio_handle, dir1_pin, 0)
                    lgpio.gpio_write(self.gpio_handle, dir2_pin, 1)
                else:
                    # Stop
                    lgpio.gpio_write(self.gpio_handle, dir1_pin, 0)
                    lgpio.gpio_write(self.gpio_handle, dir2_pin, 0)
                
                # Set PWM
                lgpio.tx_pwm(self.gpio_handle, pwm_pin, self.pwm_freq, duty_cycle)
                
            except Exception as e:
                self.get_logger().error(f'Error controlling {motor} motor: {str(e)}')
        else:
            # Simulation mode - just log the command
            direction = 'FORWARD' if speed > 0 else 'REVERSE' if speed < 0 else 'STOP'
            self.get_logger().info(
                f'{motor.upper()} motor: {direction} at {duty_cycle}% duty cycle'
            )
    
    def destroy_node(self):
        """Clean up GPIO resources"""
        if self.gpio_handle:
            try:
                # Stop all motors
                self.set_motor_speed('left', 0.0)
                self.set_motor_speed('right', 0.0)
                
                # Close GPIO handle
                lgpio.gpiochip_close(self.gpio_handle)
                self.get_logger().info('GPIO cleanup completed')
            except Exception as e:
                self.get_logger().error(f'Error during GPIO cleanup: {str(e)}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareMotorDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
