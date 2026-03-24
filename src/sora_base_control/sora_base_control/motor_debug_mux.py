import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class MotorDebugMux(Node):
    def __init__(self):
        super().__init__('motor_debug_mux')
        self.motor_names = ['FR', 'FL', 'BR', 'BL', 'All']
        self.selected = 0  # Start with FR
        self.last_y = 0

        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands_raw',
            self.cmd_callback,
            10
        )
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )

    def joy_callback(self, msg):
        y_button = msg.buttons[3]  # Y is usually button 3 on Xbox
        if y_button and not self.last_y:
            self.selected = (self.selected + 1) % len(self.motor_names)
            self.get_logger().info(f"Selected motor: {self.motor_names[self.selected]}")
        self.last_y = y_button

    def cmd_callback(self, msg):
        data = list(msg.data)
        if self.motor_names[self.selected] == 'All':
            masked = data
        else:
            masked = [0.0, 0.0, 0.0, 0.0]
            masked[self.selected] = data[self.selected]
        out = Float64MultiArray()
        out.data = masked
        self.pub_cmd.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDebugMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()