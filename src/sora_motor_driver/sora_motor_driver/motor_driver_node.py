import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from adafruit_motorkit import MotorKit

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.kit = MotorKit(address=0x60)  # Bonnet default I2C address
        self.max_throttle = 1.0  # Set max throttle (adjust as needed)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.listener_callback,
            10
        )
        self.get_logger().info('Motor driver node started (Adafruit Bonnet)')

    def listener_callback(self, msg):
        # Expecting [FR, FL, BR, BL]
        if len(msg.data) != 4:
            self.get_logger().error('Expected 4 wheel velocities, got %d' % len(msg.data))
            return

        # Scale velocities to [-max_throttle, max_throttle]
        throttles = [max(min(v, 1.0), -1.0) * self.max_throttle for v in msg.data]

        # Assign to motors: M1=FL, M2=FR, M3=BR, M4=BL
        self.kit.motor1.throttle = throttles[1]  # FL
        self.kit.motor2.throttle = throttles[0]  # FR
        self.kit.motor3.throttle = throttles[2]  # BR
        self.kit.motor4.throttle = throttles[3]  # BL

    def destroy_node(self):
        # Stop all motors on shutdown
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0
        self.kit.motor3.throttle = 0
        self.kit.motor4.throttle = 0
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()