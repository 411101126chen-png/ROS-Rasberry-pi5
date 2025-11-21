import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class VisionCommander(Node):
    def __init__(self):
        super().__init__('vision_commander')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for 30Hz loop
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        # Open video capture (default to /dev/video0)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device /dev/video0')
        else:
            self.get_logger().info('Video device /dev/video0 opened successfully')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # --- Custom Vision Logic Block ---
        # Analyze 'frame' here.
        # Example logic:
        # 1. Process image (e.g., line following, object detection)
        # 2. Determine desired linear_x and angular_z
        
        # Placeholder logic (to be replaced by user):
        # For now, we will just send 0.0 to keep it safe, 
        # or implement the example logic from the prompt if specific conditions were met.
        # The prompt says: "If judge forward, set linear.x = 0.2; if judge turn left, set angular.z = 0.5"
        # Since I don't have real input, I will default to stop (0.0) but leave code commented for the example.
        
        target_linear_x = 0.0
        target_angular_z = 0.0
        
        # [USER TODO]: Implement your vision logic here
        # Example:
        # if some_condition_forward:
        #     target_linear_x = 0.2
        # elif some_condition_left:
        #     target_angular_z = 0.5
        
        # ---------------------------------

        # Publish command
        msg = Twist()
        msg.linear.x = float(target_linear_x)
        msg.angular.z = float(target_angular_z)
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
