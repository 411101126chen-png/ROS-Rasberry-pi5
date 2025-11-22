import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow')
        self.get_logger().info('follow node started')

        # Parameters to tune the follower behavior without touching code
        self.target_distance = self.declare_parameter('target_distance', 0.6).value
        self.linear_gain = self.declare_parameter('linear_gain', 0.8).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 0.35).value
        self.angular_gain = self.declare_parameter('angular_gain', 2.5).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 1.5).value
        self.front_angle_deg = self.declare_parameter('front_angle_deg', 40.0).value
        self._front_angle_rad = math.radians(self.front_angle_deg)
        
        # 訂閱 /scan 話題
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._last_cmd = Twist()

    def scan_callback(self, msg):
        closest_range = None
        closest_angle = 0.0
        angle = msg.angle_min

        for rng in msg.ranges:
            if not math.isfinite(rng):
                angle += msg.angle_increment
                continue
            if abs(angle) > self._front_angle_rad:
                angle += msg.angle_increment
                continue

            if closest_range is None or rng < closest_range:
                closest_range = rng
                closest_angle = angle
            angle += msg.angle_increment

        twist = Twist()
        if closest_range is None:
            # 沒有追蹤目標時就停車
            self._publish_cmd(twist, warn=True)
            return

        distance_error = closest_range - self.target_distance
        twist.linear.x = self._saturate(self.linear_gain * distance_error, self.max_linear_speed)
        twist.angular.z = self._saturate(-self.angular_gain * closest_angle, self.max_angular_speed)
        self._publish_cmd(twist)

    def _publish_cmd(self, twist: Twist, warn: bool = False):
        if warn and (self._last_cmd.linear.x != 0.0 or self._last_cmd.angular.z != 0.0):
            self.get_logger().warn('No valid target detected, stopping the robot.')
        self.cmd_pub.publish(twist)
        self._last_cmd = twist

    @staticmethod
    def _saturate(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
