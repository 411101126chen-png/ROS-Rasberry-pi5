#!/usr/bin/env python3
"""
Balance Controller Node
Maintains balance and follows visual targets detected by the vision system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Parameters
        self.declare_parameter('balance_kp', 0.5)
        self.declare_parameter('balance_ki', 0.01)
        self.declare_parameter('balance_kd', 0.1)
        self.declare_parameter('target_tracking_enabled', True)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.balance_kp = self.get_parameter('balance_kp').value
        self.balance_ki = self.get_parameter('balance_ki').value
        self.balance_kd = self.get_parameter('balance_kd').value
        self.target_tracking_enabled = self.get_parameter('target_tracking_enabled').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.target_sub = self.create_subscription(
            Point,
            'vision/target_position',
            self.target_callback,
            10
        )
        
        self.target_detected_sub = self.create_subscription(
            Bool,
            'vision/target_detected',
            self.target_detected_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # State variables
        self.current_pitch = 0.0
        self.target_position = None
        self.target_detected = False
        self.pitch_error_sum = 0.0
        self.last_pitch_error = 0.0
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Balance Controller Node started')
        
    def imu_callback(self, msg):
        """Process IMU data to get current pitch angle"""
        # Extract pitch from IMU orientation quaternion
        # Simplified - in real implementation, convert quaternion to euler angles
        self.current_pitch = msg.orientation.y
        
    def target_callback(self, msg):
        """Receive target position from vision system"""
        self.target_position = msg
        
    def target_detected_callback(self, msg):
        """Update target detection status"""
        self.target_detected = msg.data
        
    def control_loop(self):
        """Main control loop for balance and target tracking"""
        cmd = Twist()
        
        # Balance control (PID controller for pitch)
        target_pitch = 0.0  # Upright position
        pitch_error = target_pitch - self.current_pitch
        
        self.pitch_error_sum += pitch_error
        pitch_error_derivative = pitch_error - self.last_pitch_error
        self.last_pitch_error = pitch_error
        
        # PID calculation for balance
        balance_output = (
            self.balance_kp * pitch_error +
            self.balance_ki * self.pitch_error_sum +
            self.balance_kd * pitch_error_derivative
        )
        
        # Set base linear velocity from balance controller
        cmd.linear.x = balance_output
        
        # Add target tracking if enabled and target is detected
        if self.target_tracking_enabled and self.target_detected and self.target_position:
            # Target position x represents horizontal position (-1 to 1, 0 is center)
            # Negative x means target is on the left, positive means right
            target_x = self.target_position.x
            
            # Simple proportional controller for turning toward target
            angular_gain = 2.0
            cmd.angular.z = -angular_gain * target_x
            
            # Move forward if target is centered
            if abs(target_x) < 0.1:  # Target is centered
                cmd.linear.x += 0.1  # Add forward motion
        
        # Clamp velocities to maximum values
        cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
