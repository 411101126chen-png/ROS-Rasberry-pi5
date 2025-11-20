#!/usr/bin/env python3
"""
Camera Node
Captures video from webcam and publishes images to ROS topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        self.camera_device = self.get_parameter('camera_device').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        # CV Bridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_device)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.camera_device}')
            return
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Timer for capturing and publishing frames
        self.timer = self.create_timer(1.0 / self.camera_fps, self.capture_and_publish)
        
        self.get_logger().info(f'Camera Node started - Device: {self.camera_device}, '
                             f'Resolution: {self.image_width}x{self.image_height}, '
                             f'FPS: {self.camera_fps}')
        
    def capture_and_publish(self):
        """Capture frame from camera and publish as ROS Image message"""
        ret, frame = self.cap.read()
        
        if ret:
            try:
                # Convert OpenCV image to ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'camera_frame'
                
                # Publish the image
                self.image_pub.publish(image_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')
        else:
            self.get_logger().warn('Failed to capture frame from camera')
            
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
