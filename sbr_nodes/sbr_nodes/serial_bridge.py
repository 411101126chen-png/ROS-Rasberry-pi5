import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Serial connection
        self.ser = None
        self.connect_serial()
        
        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Timer for reconnection monitoring (every 1 second)
        self.create_timer(1.0, self.check_connection)

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'Connected to {self.port} at {self.baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self.ser = None

    def check_connection(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial disconnected. Attempting to reconnect...')
            self.connect_serial()

    def cmd_vel_callback(self, msg):
        if self.ser and self.ser.is_open:
            try:
                # Format: "<linear_x>,<angular_z>\n"
                # Example: "0.25,-0.10\n"
                payload = f"{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
                self.ser.write(payload.encode('utf-8'))
                # self.get_logger().debug(f'Sent: {payload.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
                self.ser.close() # Force reconnection attempt on next timer tick
        else:
            # self.get_logger().warn('Serial not connected, cannot send command')
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
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
