#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/cmd_vel_array', 10)
        self.get_logger().info("Twist to Float64MultiArray bridge started for Isaac Sim!")

    def cb(self, msg):
        arr = Float64MultiArray()
        # Pack only [Linear X, Linear Y, Angular Z]
        arr.data = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.pub.publish(arr)

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