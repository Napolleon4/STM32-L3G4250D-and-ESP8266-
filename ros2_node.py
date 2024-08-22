import geometry_msgs.msg
import geometry_msgs.msg._twist
import rclpy
from rclpy.node import Node
import socket
import struct
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def _init_(self):
        super()._init_('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/gyro_values', 10)
        
        # UDP Setup
        self.udp_ip = "192.168.1.101"
        self.udp_port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"Listening on IP: {self.udp_ip}, PORT: {self.udp_port}")
        
        # Create a timer to periodically check for UDP data
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Check for UDP data
        try:
            self.sock.settimeout(0.1)  # Short timeout to avoid blocking
            data, addr = self.sock.recvfrom(1024)
            if len(data) == 2:  # Check if the received data is 2 bytes long (int16_t)
                value = struct.unpack('<h', data)[0]  # '<h' format code is for little-endian int16_t
                self.get_logger().info(f"Received message: {value}")
                
                # Publish the received value
                msg = Twist()
                msg.angular.z= float(value) 
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: "{value}"')
            else:
                self.get_logger().info(f"Received data of incorrect length: {len(data)} bytes")
        except socket.timeout:
            pass  # Timeout occurred; no data to read

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
