# -*- coding: utf-8 -*-
"""A small visual simulator that publishes a synthetic moving blob as camera frames.

This node helps testing the follow_node without a real camera. It publishes
sensor_msgs/Image on topic /camera/image_raw and (optionally) shows a cv2 window.

Parameters:
- rate: publish rate (Hz)
- width, height: frame size
- no_gui: if True, do not open cv2.imshow
"""
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
import cv2

try:
    from cv_bridge import CvBridge
    _HAS_CVBRIDGE = True
except Exception:
    CvBridge = None
    _HAS_CVBRIDGE = False


class ImageSimulator(Node):
    def __init__(self):
        super().__init__('image_simulator')
        # Declare rate as float default. ROS2 parameter overrides may supply
        # an integer (e.g. -p rate:=5) which rclpy treats as INTEGER and will
        # raise InvalidParameterTypeException when the declared default is a
        # DOUBLE. To be robust, try declaring a double first and fall back to
        # declaring an integer if the override has a different type.
        try:
            self.declare_parameter('rate', 10.0)
        except Exception:
            # fallback to int default to accept integer overrides
            try:
                self.declare_parameter('rate', 10)
            except Exception:
                # last resort: declare without default (will error later if missing)
                self.declare_parameter('rate')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('no_gui', False)
        rate = float(self.get_parameter('rate').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.no_gui = bool(self.get_parameter('no_gui').value)
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge() if _HAS_CVBRIDGE else None
        self.t0 = time.time()
        self.timer = self.create_timer(1.0/float(rate), self.timer_cb)
        self.get_logger().info(f'image_simulator started (w={self.width}, h={self.height}, rate={rate}, no_gui={self.no_gui})')

    def make_frame(self, t):
        H = self.height
        W = self.width
        frame = np.zeros((H, W, 3), dtype=np.uint8)
        # background grid for nicer visuals
        for y in range(0, H, 40):
            cv2.line(frame, (0, y), (W, y), (25, 25, 25), 1)
        for x in range(0, W, 40):
            cv2.line(frame, (x, 0), (x, H), (25, 25, 25), 1)
        # moving blob (simulate person) - sinusoidal path
        cx = int(W*0.5 + (W*0.35) * math.sin(t*0.7))
        cy = int(H*0.5 + (H*0.2) * math.cos(t*0.4))
        radius = int(min(W,H)*0.12)
        cv2.circle(frame, (cx, cy), radius, (0,180,0), -1)
        # add a bright head
        cv2.circle(frame, (cx, cy - int(radius*0.6)), int(radius*0.4), (0,255,200), -1)
        # draw center vertical line
        cv2.line(frame, (W//2,0), (W//2,H), (120,120,120), 1)
        return frame

    def timer_cb(self):
        t = time.time() - self.t0
        frame = self.make_frame(t)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_sim'
        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header = header
                self.pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'cv_bridge publish failed: {e}')
        else:
            # fallback: construct sensor_msgs/Image manually (RGB8)
            img_msg = Image()
            img_msg.header = header
            img_msg.height = frame.shape[0]
            img_msg.width = frame.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.step = frame.shape[1] * 3
            img_msg.data = frame.tobytes()
            self.pub.publish(img_msg)

        if not self.no_gui:
            try:
                cv2.imshow('image_simulator', frame)
                cv2.waitKey(1)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
