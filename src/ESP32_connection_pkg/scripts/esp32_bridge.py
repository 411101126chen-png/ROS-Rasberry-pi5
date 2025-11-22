#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import json

class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout_sec', 1.0) # Stop if no command for 1 second

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        # Initialize Serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port} at {self.baud_rate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.ser = None

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for safety timeout
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self.check_timeout)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        
        if self.ser and self.ser.is_open:
            # Format: L:linear_x,A:angular_z\n
            # Example: L:0.5,A:0.1
            command = f"L:{msg.linear.x:.2f},A:{msg.angular.z:.2f}\n"
            try:
                self.ser.write(command.encode('utf-8'))
                # self.get_logger().debug(f'Sent: {command.strip()}')
            except Exception as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def check_timeout(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            if self.ser and self.ser.is_open:
                # Send stop command
                try:
                    self.ser.write(b"L:0.00,A:0.00\n")
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
