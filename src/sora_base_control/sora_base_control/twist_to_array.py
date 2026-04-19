#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import struct

class TwistUDPBridge(Node):
    def __init__(self):
        super().__init__('twist_udp_bridge')
        # CHANGE '192.168.x.x' to your Windows PC's exact IP address on the network!
        self.target_ip = "192.168.1.91"  
        self.target_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info(f"UDP Bridge broadcasting /cmd_vel to {self.target_ip}:{self.target_port}")

    def cb(self, msg):
        # Pack the 3 floats (x, y, z) into exactly 24 bytes of raw binary
        data = struct.pack('ddd', msg.linear.x, msg.linear.y, msg.angular.z)
        self.sock.sendto(data, (self.target_ip, self.target_port))

def main(args=None):
    rclpy.init(args=args)
    node = TwistUDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()