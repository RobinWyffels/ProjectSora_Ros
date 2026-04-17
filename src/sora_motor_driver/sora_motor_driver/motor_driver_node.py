import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import lgpio
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Open GPIO chip
        self.h = lgpio.gpiochip_open(4)  # on Raspberry Pi, GPIO 0-27 are on chip 4 (BCM numbering)

        # Pin definitions (BCM numbering)
        self.DIR_FR = 6   # MDD10A One M1 (FR)
        self.PWM_FR = 13
        self.DIR_FL = 19  # MDD10A One M2 (FL)
        self.PWM_FL = 26
        self.DIR_BR = 12  # MDD10A Two M1 (BR)
        self.PWM_BR = 16
        self.DIR_BL = 20  # MDD10A Two M2 (BL)
        self.PWM_BL = 21

        # Set pins as outputs
        for pin in [self.DIR_FR, self.PWM_FR, self.DIR_FL, self.PWM_FL,
                    self.DIR_BR, self.PWM_BR, self.DIR_BL, self.PWM_BL]:
            lgpio.gpio_claim_output(self.h, pin)

        # Initialize PWM (1000 Hz frequency)
        self.pwm_freq = 1000
        self.pwm_fr = self.PWM_FR
        self.pwm_fl = self.PWM_FL
        self.pwm_br = self.PWM_BR
        self.pwm_bl = self.PWM_BL

        # Start PWM at 0% duty cycle
        for pwm_pin in [self.pwm_fr, self.pwm_fl, self.pwm_br, self.pwm_bl]:
            lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, 0)

        self.max_throttle = 0.5  # Max throttle (adjust as needed)

        # Ramping parameters
        self.ramp_rate = 0.5  # throttle units per second (1.0 = full scale in 1s)
        self.timer_period = 0.02  # seconds (50 Hz)
        self.current_throttles = [0.0, 0.0, 0.0, 0.0]  # [FR, FL, BR, BL]
        self.target_throttles = [0.0, 0.0, 0.0, 0.0]

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.listener_callback,
            10
        )

        # Timer for ramping
        self.timer = self.create_timer(self.timer_period, self.ramp_timer_callback)

        self.get_logger().info('Motor driver node started (Cytron MDD10A, using lgpio)')

    def listener_callback(self, msg):
        # Expecting [FR, FL, BR, BL]
        if len(msg.data) != 4:
            self.get_logger().error('Expected 4 wheel velocities, got %d' % len(msg.data))
            return

        # Scale velocities to [-max_throttle, max_throttle]
        self.target_throttles = [max(min(v, 1.0), -1.0) * self.max_throttle for v in msg.data]

    def ramp_timer_callback(self):
        # Helper function to set motor direction and speed
        def set_motor(dir_pin, pwm_pin, throttle):
            if throttle >= 0:
                lgpio.gpio_write(self.h, dir_pin, 1)  # Forward
            else:
                lgpio.gpio_write(self.h, dir_pin, 0)  # Backward
            # Duty cycle: 0-100
            duty = int(abs(throttle) * 100)
            lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, duty)

        # Ramp each motor's throttle toward its target
        for i in range(4):
            delta = self.target_throttles[i] - self.current_throttles[i]
            max_delta = self.ramp_rate * self.timer_period
            if abs(delta) > max_delta:
                delta = max_delta if delta > 0 else -max_delta
            self.current_throttles[i] += delta

        # Assign to motors
        set_motor(self.DIR_FR, self.pwm_fr, self.current_throttles[0])  # FR
        set_motor(self.DIR_FL, self.pwm_fl, self.current_throttles[1])  # FL
        set_motor(self.DIR_BR, self.pwm_br, self.current_throttles[2])  # BR
        set_motor(self.DIR_BL, self.pwm_bl, self.current_throttles[3])  # BL

    def destroy_node(self):
        # Stop all motors and cleanup
        for pwm_pin in [self.pwm_fr, self.pwm_fl, self.pwm_br, self.pwm_bl]:
            lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, 0)
        lgpio.gpiochip_close(self.h)
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