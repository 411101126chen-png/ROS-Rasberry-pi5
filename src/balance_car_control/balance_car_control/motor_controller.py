#!/usr/bin/env python3
"""
Motor Controller Node for Balance Car
Controls the DC motors based on velocity commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('motor_update_rate', 50.0)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.motor_update_rate = self.get_parameter('motor_update_rate').value
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for motor speeds (left and right)
        self.left_motor_pub = self.create_publisher(Float32, 'left_motor_speed', 10)
        self.right_motor_pub = self.create_publisher(Float32, 'right_motor_speed', 10)
        
        # Current motor speeds
        self.left_speed = 0.0
        self.right_speed = 0.0
        
        # Timer for motor updates
        self.timer = self.create_timer(1.0 / self.motor_update_rate, self.update_motors)
        
        self.get_logger().info('Motor Controller Node started')
        
    def cmd_vel_callback(self, msg):
        """
        Process velocity commands and calculate motor speeds
        Uses differential drive kinematics
        """
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Simple differential drive calculation
        # Assumes wheel separation and converts to motor speeds
        wheel_separation = 0.2  # meters (adjust based on your robot)
        
        # Calculate motor speeds
        self.left_speed = (linear_vel - angular_vel * wheel_separation / 2.0)
        self.right_speed = (linear_vel + angular_vel * wheel_separation / 2.0)
        
        # Clamp speeds to max_speed
        self.left_speed = max(-self.max_speed, min(self.max_speed, self.left_speed))
        self.right_speed = max(-self.max_speed, min(self.max_speed, self.right_speed))
        
    def update_motors(self):
        """Publish motor speeds"""
        left_msg = Float32()
        right_msg = Float32()
        
        left_msg.data = self.left_speed
        right_msg.data = self.right_speed
        
        self.left_motor_pub.publish(left_msg)
        self.right_motor_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
