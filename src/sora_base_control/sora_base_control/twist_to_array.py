#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        # Publish a Point instead of an Array!
        self.pub = self.create_publisher(Point, '/cmd_vel_point', 10)
        self.get_logger().info("Twist to Point bridge started for Isaac Sim!")

    def cb(self, msg):
        pt = Point()
        pt.x = msg.linear.x   # Forward
        pt.y = msg.linear.y   # Strafe
        pt.z = msg.angular.z  # Spin
        self.pub.publish(pt)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()