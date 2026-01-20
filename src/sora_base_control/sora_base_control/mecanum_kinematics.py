#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # Robot geometry parameters (meters)
        self.declare_parameter('wheel_radius', 0.097)
        self.declare_parameter('wheel_base_length', 0.410)  # distance between front and rear wheels
        self.declare_parameter('wheel_base_width', 0.305)   # distance between left and right wheels

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_base_length = self.get_parameter('wheel_base_length').get_parameter_value().double_value
        wheel_base_width = self.get_parameter('wheel_base_width').get_parameter_value().double_value
        
        # Calculate L for mecanum kinematics (half of wheel base diagonal)
        self.lx = wheel_base_length / 2.0
        self.ly = wheel_base_width / 2.0

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )

        self.get_logger().info('Mecanum kinematics node started')
        self.get_logger().info(f'Wheel radius: {self.r}m, lx: {self.lx}m, ly: {self.ly}m')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel (vx, vy, wz) to wheel velocities for mecanum drive.
        
        Wheel order: [FR, FL, BR, BL]
        """
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Sum of half-widths for mecanum kinematics
        L = self.lx + self.ly

        # Mecanum inverse kinematics
        # FR: front-right, FL: front-left, BR: back-right, BL: back-left
        w_fr = (1.0 / self.r) * (vx + vy + L * wz)
        w_fl = (1.0 / self.r) * (vx - vy - L * wz)
        w_br = (1.0 / self.r) * (vx - vy + L * wz)
        w_bl = (1.0 / self.r) * (vx + vy - L * wz)

        # Create command message
        cmd = Float64MultiArray()
        cmd.data = [w_fr, w_fl, w_br, w_bl]

        self.wheel_cmd_pub.publish(cmd)
        
        # Debug logging (can be commented out)
        # self.get_logger().info(f'Wheel velocities: FR={w_fr:.2f}, FL={w_fl:.2f}, BR={w_br:.2f}, BL={w_bl:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
