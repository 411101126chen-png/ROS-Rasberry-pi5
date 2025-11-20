#!/usr/bin/env python3
"""
Vision Processor Node
Processes camera images to detect and track objects
Publishes target position for the balance controller
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        # Parameters
        self.declare_parameter('detection_method', 'color')  # 'color', 'contour', or 'face'
        self.declare_parameter('target_color_lower', [0, 100, 100])  # HSV lower bound
        self.declare_parameter('target_color_upper', [10, 255, 255])  # HSV upper bound
        self.declare_parameter('min_target_area', 500)
        self.declare_parameter('publish_debug_image', True)
        
        self.detection_method = self.get_parameter('detection_method').value
        self.target_color_lower = np.array(self.get_parameter('target_color_lower').value)
        self.target_color_upper = np.array(self.get_parameter('target_color_upper').value)
        self.min_target_area = self.get_parameter('min_target_area').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Face detection cascade (for face tracking method)
        self.face_cascade = None
        if self.detection_method == 'face':
            try:
                self.face_cascade = cv2.CascadeClassifier(
                    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                )
            except:
                self.get_logger().warn('Face cascade not loaded, falling back to color detection')
                self.detection_method = 'color'
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.target_position_pub = self.create_publisher(Point, 'vision/target_position', 10)
        self.target_detected_pub = self.create_publisher(Bool, 'vision/target_detected', 10)
        
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, 'vision/debug_image', 10)
        
        self.get_logger().info(f'Vision Processor Node started - Method: {self.detection_method}')
        
    def image_callback(self, msg):
        """Process incoming images to detect targets"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect target based on selected method
            if self.detection_method == 'color':
                target_center, debug_image = self.detect_color_target(cv_image)
            elif self.detection_method == 'face':
                target_center, debug_image = self.detect_face_target(cv_image)
            else:
                target_center, debug_image = self.detect_contour_target(cv_image)
            
            # Publish target position
            if target_center is not None:
                # Normalize coordinates to [-1, 1] range
                # x: -1 (left) to 1 (right), y: -1 (top) to 1 (bottom)
                height, width = cv_image.shape[:2]
                normalized_x = (target_center[0] - width / 2) / (width / 2)
                normalized_y = (target_center[1] - height / 2) / (height / 2)
                
                point_msg = Point()
                point_msg.x = normalized_x
                point_msg.y = normalized_y
                point_msg.z = 0.0
                
                self.target_position_pub.publish(point_msg)
                
                # Publish detection status
                detected_msg = Bool()
                detected_msg.data = True
                self.target_detected_pub.publish(detected_msg)
            else:
                # No target detected
                detected_msg = Bool()
                detected_msg.data = False
                self.target_detected_pub.publish(detected_msg)
            
            # Publish debug image if enabled
            if self.publish_debug_image and debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_color_target(self, image):
        """Detect target based on color in HSV space"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for target color
        mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
        
        # Apply morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_center = None
        debug_image = image.copy()
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_target_area:
                # Calculate center of contour
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    target_center = (cx, cy)
                    
                    # Draw on debug image
                    cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, target_center, 10, (0, 0, 255), -1)
                    cv2.putText(debug_image, f'Target ({cx}, {cy})', 
                              (cx + 15, cy), cv2.FONT_HERSHEY_SIMPLEX, 
                              0.5, (0, 0, 255), 2)
        
        return target_center, debug_image
    
    def detect_face_target(self, image):
        """Detect face as target"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        
        target_center = None
        debug_image = image.copy()
        
        if len(faces) > 0:
            # Use the largest face
            largest_face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = largest_face
            
            # Calculate center
            cx = x + w // 2
            cy = y + h // 2
            target_center = (cx, cy)
            
            # Draw on debug image
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug_image, target_center, 10, (0, 0, 255), -1)
            cv2.putText(debug_image, 'Face Detected', 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 0), 2)
        
        return target_center, debug_image
    
    def detect_contour_target(self, image):
        """Detect target based on largest contour"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_center = None
        debug_image = image.copy()
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_target_area:
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    target_center = (cx, cy)
                    
                    cv2.drawContours(debug_image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(debug_image, target_center, 10, (0, 0, 255), -1)
        
        return target_center, debug_image

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
