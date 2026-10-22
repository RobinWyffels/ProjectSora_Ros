import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool, String
import lgpio
import time
import threading
import random


# Short PWM "chirp" tone sequences per mood: list of (freq_hz, duration_s, duty).
# Duty is kept low (a tone, not a drive command). Each chirp is kept well under
# a second so it can be aborted quickly if a drive command arrives.
MOOD_CHIRPS = {
    "curious": [
        [(1200, 0.08, 9), (1600, 0.08, 9), (2000, 0.10, 10)],
        [(1400, 0.07, 9), (1100, 0.07, 9), (1700, 0.12, 10)],
    ],
    "excited": [
        [(1800, 0.05, 10), (2200, 0.05, 10), (1900, 0.05, 10), (2500, 0.09, 10)],
        [(2000, 0.05, 10), (2400, 0.05, 10), (2800, 0.08, 10)],
    ],
    "happy": [
        [(1500, 0.09, 9), (2000, 0.09, 9), (2500, 0.13, 10)],
        [(1600, 0.08, 9), (1900, 0.08, 9), (2300, 0.08, 9), (2600, 0.12, 10)],
    ],
    "sad": [
        [(1000, 0.16, 8), (820, 0.18, 7), (640, 0.26, 6)],
        [(900, 0.18, 7), (700, 0.22, 6), (560, 0.30, 6)],
    ],
    "sleepy": [
        [(1000, 0.14, 6), (880, 0.20, 5), (760, 0.28, 5)],
        [(950, 0.16, 6), (820, 0.24, 5)],
    ],
}

# Random seconds between chirps per mood (min, max).
MOOD_INTERVAL = {
    "curious": (5.0, 12.0),
    "excited": (3.0, 7.0),
    "happy": (4.0, 9.0),
    "sad": (9.0, 18.0),
    "sleepy": (12.0, 22.0),
}


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.declare_parameter('enable_startup_beep', True)
        self.enable_startup_beep = (
            self.get_parameter('enable_startup_beep')
            .get_parameter_value()
            .bool_value
        )

        # Stiction compensation: any non-zero command is driven at least this
        # hard (in scaled throttle units, i.e. a fraction of max_throttle).
        # 0.0 keeps the original behaviour; raise it so slow commands still move.
        self.declare_parameter('min_move_throttle', 0.0)
        self.min_move_throttle = (
            self.get_parameter('min_move_throttle')
            .get_parameter_value()
            .double_value
        )

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
        # If the startup beep is disabled, treat it as already done so chirps
        # (which wait for it) are not blocked forever.
        self.startup_beep_done = not self.enable_startup_beep

        # --- Chirp / mood system -----------------------------------------
        self.declare_parameter('chirp_enabled', False)
        self.declare_parameter('mood', 'curious')
        self._chirp_enabled = self.get_parameter('chirp_enabled').value
        self._mood = self.get_parameter('mood').value
        if self._mood not in MOOD_CHIRPS:
            self._mood = 'curious'
        self._chirping = False          # True while a chirp is playing on the pins
        self._next_chirp = time.monotonic() + self._chirp_interval()

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self.listener_callback,
            10
        )

        # Latched so the current chirp/mood settings survive a late/restarted
        # web_control (and vice-versa).
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/demo/chirp_enable', self._on_chirp_enable, latched)
        self.create_subscription(String, '/demo/mood', self._on_mood, latched)

        # Timer for ramping
        self.timer = self.create_timer(self.timer_period, self.ramp_timer_callback)

        # Timer that occasionally fires a chirp (only when the motors are idle).
        self.chirp_timer = self.create_timer(0.3, self.chirp_tick)

        self.get_logger().info('Motor driver node started (Cytron MDD10A, using lgpio)')

        if self.enable_startup_beep:
            self.startup_beep_timer = self.create_timer(0.5, self.startup_beep_callback)
        else:
            self.startup_beep_timer = None

    def listener_callback(self, msg):
        # Expecting [FR, FL, BR, BL]
        if len(msg.data) != 4:
            self.get_logger().error('Expected 4 wheel velocities, got %d' % len(msg.data))
            return

        # Scale velocities to [-max_throttle, max_throttle]
        self.target_throttles = [max(min(v, 1.0), -1.0) * self.max_throttle for v in msg.data]

    def ramp_timer_callback(self):
        # While a chirp is sounding the motors are idle and the chirp worker
        # owns the PWM pins, so skip driving them this tick.
        if self._chirping:
            return

        # Helper function to set motor direction and speed
        def set_motor(dir_pin, pwm_pin, throttle):
            if throttle >= 0:
                lgpio.gpio_write(self.h, dir_pin, 1)  # Forward
            else:
                lgpio.gpio_write(self.h, dir_pin, 0)  # Backward
            # Apply stiction floor to non-zero commands so slow speeds still move.
            magnitude = abs(throttle)
            if magnitude > 1e-3:
                magnitude = max(magnitude, self.min_move_throttle)
            # Duty cycle: 0-100
            duty = int(magnitude * 100)
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

    # ------------------------------------------------------------------
    # Chirp / mood
    # ------------------------------------------------------------------
    def _chirp_interval(self):
        lo, hi = MOOD_INTERVAL.get(self._mood, (5.0, 12.0))
        return random.uniform(lo, hi)

    def _on_chirp_enable(self, msg):
        self._chirp_enabled = bool(msg.data)
        if self._chirp_enabled:
            self._next_chirp = time.monotonic() + self._chirp_interval()
        self.get_logger().info('chirps %s' % ('ON' if self._chirp_enabled else 'OFF'))

    def _on_mood(self, msg):
        m = (msg.data or '').strip().lower()
        if m in MOOD_CHIRPS:
            self._mood = m
            self.get_logger().info('mood -> %s' % m)

    def _motors_idle(self):
        eps = 1e-3
        return (all(abs(t) < eps for t in self.target_throttles) and
                all(abs(c) < eps for c in self.current_throttles))

    def chirp_tick(self):
        # Fire a chirp only when enabled, not already chirping, after the
        # startup beep, while the robot is standing still, and the interval
        # has elapsed. This guarantees chirps never interrupt motion.
        if not self._chirp_enabled or self._chirping or not self.startup_beep_done:
            return
        if not self._motors_idle() or time.monotonic() < self._next_chirp:
            return
        seq = random.choice(MOOD_CHIRPS.get(self._mood, MOOD_CHIRPS['curious']))
        self._chirping = True
        threading.Thread(target=self._play_chirp, args=(list(seq),), daemon=True).start()

    def _play_chirp(self, seq):
        pins = [self.pwm_fr, self.pwm_fl, self.pwm_br, self.pwm_bl]
        try:
            for freq, dur, duty in seq:
                # Abort instantly if chirps were turned off or a drive command
                # arrived (so motion is never delayed by more than one step).
                if not self._chirp_enabled or not self._motors_idle():
                    break
                for p in pins:
                    lgpio.tx_pwm(self.h, p, int(freq), int(duty))
                time.sleep(dur)
        finally:
            for p in pins:
                lgpio.tx_pwm(self.h, p, self.pwm_freq, 0)
            self._chirping = False
            self._next_chirp = time.monotonic() + self._chirp_interval()

    def startup_beep_callback(self):
        if self.startup_beep_done:
            return

        self.startup_beep_done = True

        tone_sequence = [
            (1000, 0.07, 8),
            (1200, 0.08, 8),
            (1400, 0.03, 8),
            (1500, 0.1, 8),
            (1200, 0.08, 8),
            (1500, 0.2, 10),
            (1000, 1.0, 0),
            (1000, 0.5, 10),
            (1300, 0.3, 10),
            (1000, 0.2, 0),
            (1000, 0.5, 10),
            (1300, 0.3, 10)
        ]

        pwm_pins = [self.pwm_fr, self.pwm_fl, self.pwm_br, self.pwm_bl]

        for pwm_pin in pwm_pins:
            lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, 0)

        try:
            for freq, duration, duty in tone_sequence:
                for pwm_pin in pwm_pins:
                    lgpio.tx_pwm(self.h, pwm_pin, freq, duty)
                time.sleep(duration)
        finally:
            for pwm_pin in pwm_pins:
                lgpio.tx_pwm(self.h, pwm_pin, self.pwm_freq, 0)

            if self.startup_beep_timer is not None:
                self.startup_beep_timer.cancel()
                self.startup_beep_timer = None

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