#!/usr/bin/env python3
"""Innovation Cup demo: reactive LiDAR wander controller driven by an Xbox pad.

Behaviour
---------
* On start-up the node sits in the ``STOPPED`` state and holds the robot
  still (it keeps publishing a zero ``/cmd_vel`` so the motor driver's
  watchdog never sees a stale command).
* Press **A** on the controller to begin roaming. The robot drives forward,
  wanders gently, and steers / rotates away from obstacles using the LiDAR.
* Press **B** at any time to stop in place. Press **A** again to resume.

The controller is purely reactive (no map, no odometry needed): it only
consumes ``/scan`` and ``/joy`` and emits ``geometry_msgs/Twist`` on
``/cmd_vel``, which the mecanum kinematics node converts to wheel commands.

Safety
------
* The LiDAR driver is configured with ``invalid_range_is_inf: false``, so
  invalid / no-return beams come back as ``0.0`` rather than ``inf``. Those
  are treated as "no information", never as an obstacle at zero distance.
* If no fresh scan has arrived within ``scan_timeout_s`` the robot stops,
  even while roaming, so a stalled LiDAR can never leave it driving blind.
"""

import math
import random
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan


# Simple state machine
STATE_STOPPED = "STOPPED"
STATE_ROAMING = "ROAMING"


class DemoWander(Node):
    def __init__(self) -> None:
        super().__init__("demo_wander")

        # --- Topics -------------------------------------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_topic", "/joy")

        # --- Controller buttons (Xbox pad via the `joy` node) -------------
        self.declare_parameter("start_button", 0)   # A
        self.declare_parameter("stop_button", 1)    # B

        # --- Motion limits (base frame) -----------------------------------
        self.declare_parameter("forward_speed", 0.35)    # m/s cruise speed
        self.declare_parameter("turn_speed", 1.0)        # rad/s in-place turn
        self.declare_parameter("reverse_speed", 0.18)    # m/s when backing off
        self.declare_parameter("control_rate", 15.0)     # Hz

        # --- Obstacle thresholds (metres) ---------------------------------
        # > slow_distance            : cruise straight (with gentle wander)
        # stop_distance..slow_distance: slow down and steer toward open side
        # backup_distance..stop_dist : rotate in place toward open side
        # < backup_distance          : too close, reverse while turning
        self.declare_parameter("slow_distance", 1.0)
        self.declare_parameter("stop_distance", 0.50)
        self.declare_parameter("backup_distance", 0.30)

        # --- Sector geometry (degrees) ------------------------------------
        # Front cone is +/- front_half_angle around straight ahead; the left
        # and right sectors extend side_span beyond the front cone.
        self.declare_parameter("front_half_angle_deg", 35.0)
        self.declare_parameter("side_span_deg", 60.0)

        # --- Wander behaviour ---------------------------------------------
        self.declare_parameter("wander_turn_max", 0.4)      # rad/s random drift
        self.declare_parameter("wander_change_period", 2.5)  # s between drifts
        self.declare_parameter("steer_gain", 1.2)           # side-clearance steering

        # --- Safety -------------------------------------------------------
        self.declare_parameter("scan_timeout_s", 0.5)

        gp = self.get_parameter
        self._scan_topic = gp("scan_topic").value
        self._cmd_vel_topic = gp("cmd_vel_topic").value
        self._joy_topic = gp("joy_topic").value
        self._start_button = int(gp("start_button").value)
        self._stop_button = int(gp("stop_button").value)

        self._forward_speed = float(gp("forward_speed").value)
        self._turn_speed = float(gp("turn_speed").value)
        self._reverse_speed = float(gp("reverse_speed").value)
        control_rate = max(1.0, float(gp("control_rate").value))

        self._slow_distance = float(gp("slow_distance").value)
        self._stop_distance = float(gp("stop_distance").value)
        self._backup_distance = float(gp("backup_distance").value)

        self._front_half = math.radians(float(gp("front_half_angle_deg").value))
        self._side_span = math.radians(float(gp("side_span_deg").value))

        self._wander_turn_max = float(gp("wander_turn_max").value)
        self._wander_change_period = float(gp("wander_change_period").value)
        self._steer_gain = float(gp("steer_gain").value)

        self._scan_timeout_s = float(gp("scan_timeout_s").value)

        # --- Runtime state ------------------------------------------------
        self._state = STATE_STOPPED
        self._scan: Optional[LaserScan] = None
        self._last_scan_time = self.get_clock().now()
        self._prev_buttons: List[int] = []

        # Turn direction while avoiding, with hysteresis to avoid dithering.
        # +1 = turn left (CCW), -1 = turn right (CW).
        self._avoid_turn_dir = 1.0

        # Gentle wander drift state.
        self._wander_turn = 0.0
        self._wander_elapsed = 0.0

        # --- ROS interfaces ----------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)

        self._dt = 1.0 / control_rate
        self._timer = self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            "demo_wander ready. Waiting for START (button %d / A). "
            "Press button %d / B to stop." % (self._start_button, self._stop_button)
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._last_scan_time = self.get_clock().now()

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)

        def rising_edge(index: int) -> bool:
            if index < 0 or index >= len(buttons):
                return False
            was = self._prev_buttons[index] if index < len(self._prev_buttons) else 0
            return buttons[index] == 1 and was == 0

        if rising_edge(self._start_button) and self._state != STATE_ROAMING:
            self._state = STATE_ROAMING
            self.get_logger().info("START pressed -> ROAMING")
        if rising_edge(self._stop_button) and self._state != STATE_STOPPED:
            self._state = STATE_STOPPED
            self._publish_stop()
            self.get_logger().info("STOP pressed -> STOPPED")

        self._prev_buttons = buttons

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._state != STATE_ROAMING:
            self._publish_stop()
            return

        # Watchdog: without a fresh scan we refuse to drive.
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
        if self._scan is None or age > self._scan_timeout_s:
            self._publish_stop()
            self.get_logger().warn(
                "No LiDAR scan for %.2fs; holding still." % age,
                throttle_duration_sec=2.0,
            )
            return

        front, left, right = self._sector_clearances(self._scan)
        cmd = self._compute_command(front, left, right)
        self._cmd_pub.publish(cmd)

    def _sector_clearances(self, scan: LaserScan):
        """Return (front, left, right) nearest valid range per sector (metres).

        An empty sector (no valid returns) reports ``range_max`` i.e. "open".
        Invalid beams (0.0, NaN, inf, or outside [range_min, range_max]) are
        ignored so a no-return is never mistaken for a zero-distance hit.
        """
        front = left = right = scan.range_max
        rmin = scan.range_min
        rmax = scan.range_max

        inner = self._front_half
        outer = self._front_half + self._side_span

        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment

            if not math.isfinite(r) or r <= rmin or r > rmax:
                continue

            # Wrap the beam angle into [-pi, pi].
            aw = (a + math.pi) % (2.0 * math.pi) - math.pi
            mag = abs(aw)

            if mag <= inner:
                front = min(front, r)
            elif inner < mag <= outer:
                if aw > 0.0:
                    left = min(left, r)
                else:
                    right = min(right, r)

        return front, left, right

    def _compute_command(self, front: float, left: float, right: float) -> Twist:
        cmd = Twist()

        # Pick / hold the turn direction toward the more open side.
        # Hysteresis: only re-evaluate the direction once we are actually
        # committed to avoiding, so the robot doesn't dither between sides.
        if front <= self._stop_distance:
            if front <= self._backup_distance or abs(self._avoid_turn_dir) < 0.5:
                self._avoid_turn_dir = 1.0 if left >= right else -1.0

        if front <= self._backup_distance:
            # Too close: back off while rotating toward the open side.
            cmd.linear.x = -self._reverse_speed
            cmd.angular.z = self._avoid_turn_dir * self._turn_speed
            self._reset_wander()
            return cmd

        if front <= self._stop_distance:
            # Blocked ahead: rotate in place toward the open side.
            cmd.linear.x = 0.0
            cmd.angular.z = self._avoid_turn_dir * self._turn_speed
            self._reset_wander()
            return cmd

        # Path is reasonably clear: cruise.
        if front >= self._slow_distance:
            speed = self._forward_speed
        else:
            # Linearly scale speed between stop_distance and slow_distance.
            frac = (front - self._stop_distance) / max(
                1e-3, self._slow_distance - self._stop_distance
            )
            speed = self._forward_speed * max(0.15, min(1.0, frac))

        # Proportional steering away from the nearer side wall.
        steer = self._steer_gain * (1.0 / max(left, 0.2) - 1.0 / max(right, 0.2))
        steer = max(-self._turn_speed, min(self._turn_speed, steer))

        # Gentle random wander so it roams instead of tracking a dead straight
        # line; disabled implicitly when steering/avoidance dominates.
        wander = self._update_wander()

        cmd.linear.x = speed
        cmd.angular.z = steer + wander
        # Release the avoidance commitment now that the way is clear.
        self._avoid_turn_dir = 0.0
        return cmd

    def _update_wander(self) -> float:
        self._wander_elapsed += self._dt
        if self._wander_elapsed >= self._wander_change_period:
            self._wander_elapsed = 0.0
            self._wander_turn = random.uniform(
                -self._wander_turn_max, self._wander_turn_max
            )
        return self._wander_turn

    def _reset_wander(self) -> None:
        self._wander_turn = 0.0
        self._wander_elapsed = 0.0

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def destroy_node(self) -> None:
        # Best effort: leave the robot stopped.
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
