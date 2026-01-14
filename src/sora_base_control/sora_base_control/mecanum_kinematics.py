import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # Robot geometry (meters)
        self.declare_parameter('wheel_radius', 0.097)
        self.declare_parameter('lx', 0.410 / 2.0)  # half length
        self.declare_parameter('ly', 0.305 / 2.0)  # half width

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.lx = self.get_parameter('lx').get_parameter_value().double_value
        self.ly = self.get_parameter('ly').get_parameter_value().double_value

        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            'velocity_controller/commands',
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        L = self.lx + self.ly

        # Standard mecanum inverse kinematics
        w_fl = (1.0 / self.r) * (vx - vy - L * wz)
        w_fr = (1.0 / self.r) * (vx + vy + L * wz)
        w_bl = (1.0 / self.r) * (vx + vy - L * wz)
        w_br = (1.0 / self.r) * (vx - vy + L * wz)

        cmd = Float64MultiArray()
        # Order must match controllers.yaml joint list
        cmd.data = [w_fr, w_fl, w_br, w_bl]

        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
