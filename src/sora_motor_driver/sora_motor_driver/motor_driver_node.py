import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from smbus2 import SMBus

# I2C addresses
FRONT_ADDR = 0x2E
BACK_ADDR = 0x2D

# Motor mapping: [FR, FL, BR, BL]
# Each driver: A = right, B = left

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.bus = SMBus(1)  # I2C bus 1 on Raspberry Pi

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.listener_callback,
            10
        )
        self.get_logger().info('Motor driver node started')

    def listener_callback(self, msg):
        # Expecting [FR, FL, BR, BL]
        if len(msg.data) != 4:
            self.get_logger().error('Expected 4 wheel velocities, got %d' % len(msg.data))
            return

        # Map velocities to PWM values (-255 to 255)
        pwm = [int(max(min(v * 100, 255), -255)) for v in msg.data]

        # Front driver: FR (A), FL (B)
        self.set_motor(FRONT_ADDR, pwm[0], pwm[1])
        # Back driver: BR (A), BL (B)
        self.set_motor(BACK_ADDR, pwm[2], pwm[3])

    def set_motor(self, addr, right, left):
        # Protocol: [A_PWM, A_DIR, B_PWM, B_DIR]
        # Direction: 0 = forward, 1 = backward
        def encode(val):
            return abs(val), 0 if val >= 0 else 1

        a_pwm, a_dir = encode(right)
        b_pwm, b_dir = encode(left)
        try:
            self.bus.write_i2c_block_data(addr, 0x00, [a_pwm, a_dir, b_pwm, b_dir])
        except Exception as e:
            self.get_logger().error(f'I2C error to addr {hex(addr)}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.bus.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()