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

import json
import math
import random
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Bool, Float64


# Simple state machine
STATE_STOPPED = "STOPPED"
STATE_ROAMING = "ROAMING"
STATE_MANUAL = "MANUAL"


class DemoWander(Node):
    def __init__(self) -> None:
        super().__init__("demo_wander")

        # --- Topics -------------------------------------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_topic", "/joy")

        # Start/stop can also come from a topic (e.g. the web control node),
        # not just the controller. A JSON status is published back so a UI can
        # show state, current mode and the showmode countdown.
        self.declare_parameter("command_topic", "/demo/command")
        self.declare_parameter("status_topic", "/demo/status")
        self.declare_parameter("mode_cmd_topic", "/demo/mode")
        self.declare_parameter("showmode_topic", "/demo/showmode")
        self.declare_parameter("showmode_interval_topic", "/demo/showmode_interval")
        self.declare_parameter("manual_topic", "/demo/manual_cmd")

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

        # --- Modes --------------------------------------------------------
        self.declare_parameter("mode", "curious")   # "curious" | "cute"

        # --- Showmode (auto-alternate modes on a timer) -------------------
        self.declare_parameter("showmode", False)
        self.declare_parameter("showmode_interval_s", 300.0)   # default 5 minutes

        # --- Manual drive -------------------------------------------------
        self.declare_parameter("manual_timeout_s", 0.5)  # zero out if commands stop

        # --- Cute mode ----------------------------------------------------
        # Stays near the start point and favours the starting orientation,
        # with lots of standing still plus little spins / strafes / wiggles.
        self.declare_parameter("cute_radius", 1.0)          # m, leash from start
        self.declare_parameter("cute_forward_speed", 0.14)  # m/s gentle scoots
        self.declare_parameter("cute_strafe_speed", 0.14)   # m/s side slides
        self.declare_parameter("cute_spin_speed", 1.0)      # rad/s little spins
        self.declare_parameter("cute_wiggle_speed", 0.9)    # rad/s wiggle amplitude

        # --- Safety -------------------------------------------------------
        self.declare_parameter("scan_timeout_s", 0.5)

        gp = self.get_parameter
        self._scan_topic = gp("scan_topic").value
        self._cmd_vel_topic = gp("cmd_vel_topic").value
        self._joy_topic = gp("joy_topic").value
        self._command_topic = gp("command_topic").value
        self._status_topic = gp("status_topic").value
        self._mode_cmd_topic = gp("mode_cmd_topic").value
        self._showmode_topic = gp("showmode_topic").value
        self._showmode_interval_topic = gp("showmode_interval_topic").value
        self._manual_topic = gp("manual_topic").value
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

        self._mode = (gp("mode").value or "curious").strip().lower()
        if self._mode not in ("curious", "cute"):
            self._mode = "curious"
        self._cute_radius = float(gp("cute_radius").value)
        self._cute_forward_speed = float(gp("cute_forward_speed").value)
        self._cute_strafe_speed = float(gp("cute_strafe_speed").value)
        self._cute_spin_speed = float(gp("cute_spin_speed").value)
        self._cute_wiggle_speed = float(gp("cute_wiggle_speed").value)

        self._showmode = bool(gp("showmode").value)
        self._showmode_interval = max(5.0, float(gp("showmode_interval_s").value))
        self._manual_timeout_s = float(gp("manual_timeout_s").value)

        self._scan_timeout_s = float(gp("scan_timeout_s").value)

        # --- Runtime state ------------------------------------------------
        self._state = STATE_STOPPED
        self._scan: Optional[LaserScan] = None
        self._last_scan_mono = time.monotonic()
        self._prev_buttons: List[int] = []

        # Turn direction while avoiding, with hysteresis to avoid dithering.
        # +1 = turn left (CCW), -1 = turn right (CW).
        self._avoid_turn_dir = 1.0

        # Gentle wander drift state.
        self._wander_turn = 0.0
        self._wander_elapsed = 0.0

        # Open-loop pose estimate (integrated from the commanded velocity),
        # used by cute mode to stay near the start and favour its orientation.
        self._px = 0.0
        self._py = 0.0
        self._yaw = 0.0

        # Cute-mode phase machine.
        self._cute_phase: Optional[str] = None
        self._cute_elapsed = 0.0
        self._cute_duration = 0.0
        self._cute_dir = 1.0

        # Showmode auto-switch timer (monotonic deadline, armed on START).
        self._showmode_deadline: Optional[float] = None

        # Manual drive state (monotonic timestamps, immune to clock steps).
        self._manual_cmd = Twist()
        self._last_manual_mono = time.monotonic()
        self._last_status_mono = time.monotonic()

        # --- ROS interfaces ----------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(
            LaserScan, self._scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)

        # Latched JSON status so a late-joining UI immediately sees current
        # state / mode / showmode countdown.
        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(String, self._status_topic, status_qos)
        self.create_subscription(String, self._command_topic, self._on_command, 10)
        self.create_subscription(Twist, self._manual_topic, self._on_manual, 10)
        # Settings are published latched by web_control; match transient_local
        # so a late-matched subscriber still receives the current values.
        self.create_subscription(String, self._mode_cmd_topic, self._on_mode, status_qos)
        self.create_subscription(Bool, self._showmode_topic, self._on_showmode, status_qos)
        self.create_subscription(
            Float64, self._showmode_interval_topic, self._on_showmode_interval, status_qos)
        self._publish_status()

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
        self._last_scan_mono = time.monotonic()

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)

        def rising_edge(index: int) -> bool:
            if index < 0 or index >= len(buttons):
                return False
            was = self._prev_buttons[index] if index < len(self._prev_buttons) else 0
            return buttons[index] == 1 and was == 0

        if rising_edge(self._start_button):
            self._start("controller")
        if rising_edge(self._stop_button):
            self._stop("controller")

        self._prev_buttons = buttons

    def _on_command(self, msg: String) -> None:
        cmd = (msg.data or "").strip().lower()
        if cmd in ("start", "go", "roam", "a"):
            self._start("command")
        elif cmd in ("stop", "halt", "b"):
            self._stop("command")
        else:
            self.get_logger().warn(
                "Unknown command %r (expected 'start' or 'stop')" % msg.data
            )

    def _on_mode(self, msg: String) -> None:
        m = (msg.data or "").strip().lower()
        if m not in ("curious", "cute"):
            self.get_logger().warn(
                "Unknown mode %r (expected 'curious' or 'cute')" % msg.data
            )
            return
        self._set_mode(m, "command")

    def _set_mode(self, m: str, source: str) -> None:
        if m != self._mode:
            self._mode = m
            # Treat "here" as the new home when switching modes.
            self._reset_pose()
            self._cute_phase = None
            self.get_logger().info("mode -> %s (%s)" % (m, source))
            self._publish_status()

    def _on_showmode(self, msg: Bool) -> None:
        self._showmode = bool(msg.data)
        self.get_logger().info("showmode %s" % ("ON" if self._showmode else "OFF"))
        if self._showmode and self._state == STATE_ROAMING:
            self._arm_showmode()
        elif not self._showmode:
            self._showmode_deadline = None
        self._publish_status()

    def _on_showmode_interval(self, msg: Float64) -> None:
        self._showmode_interval = max(5.0, float(msg.data))
        self.get_logger().info("showmode interval -> %.0fs" % self._showmode_interval)
        # Re-arm from now if a countdown is already running.
        if self._showmode_deadline is not None:
            self._arm_showmode()
        self._publish_status()

    def _on_manual(self, msg: Twist) -> None:
        self._manual_cmd = msg
        self._last_manual_mono = time.monotonic()
        if self._state != STATE_MANUAL:
            self._state = STATE_MANUAL
            self._showmode_deadline = None
            self.get_logger().info("-> MANUAL")
            self._publish_status()

    def _arm_showmode(self) -> None:
        self._showmode_deadline = time.monotonic() + self._showmode_interval

    def _showmode_remaining(self) -> float:
        if self._showmode_deadline is None or self._state != STATE_ROAMING:
            return -1.0
        return max(0.0, self._showmode_deadline - time.monotonic())

    def _reset_pose(self) -> None:
        self._px = 0.0
        self._py = 0.0
        self._yaw = 0.0

    def _start(self, source: str = "") -> None:
        if self._state != STATE_ROAMING:
            self._state = STATE_ROAMING
            self._reset_pose()
            self._cute_phase = None
            # Showmode auto-switch timer starts only on START (per spec).
            if self._showmode:
                self._arm_showmode()
            else:
                self._showmode_deadline = None
            self.get_logger().info("START (%s) -> ROAMING [mode=%s]" % (source, self._mode))
            self._publish_status()

    def _stop(self, source: str = "") -> None:
        if self._state != STATE_STOPPED:
            self._state = STATE_STOPPED
            self._showmode_deadline = None
            self._publish_stop()
            self.get_logger().info("STOP (%s) -> STOPPED" % source)
            self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps({
            "state": self._state,
            "mode": self._mode,
            "showmode": self._showmode,
            "interval": self._showmode_interval,
            "remaining": round(self._showmode_remaining(), 1),
        })
        self._status_pub.publish(msg)
        self._last_status_mono = time.monotonic()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        now = time.monotonic()

        # Periodically republish status so the UI countdown updates.
        if now - self._last_status_mono >= 0.5:
            self._publish_status()

        if self._state == STATE_MANUAL:
            # Drive the last manual command, zeroed if it has gone stale
            # (button released / connection lost).
            age = now - self._last_manual_mono
            self._cmd_pub.publish(self._manual_cmd if age <= self._manual_timeout_s else Twist())
            return

        if self._state != STATE_ROAMING:
            self._publish_stop()
            return

        # Showmode: once the countdown elapses, flip the mode and re-arm.
        if self._showmode and self._showmode_deadline is not None:
            if now >= self._showmode_deadline:
                new_mode = "cute" if self._mode == "curious" else "curious"
                self._set_mode(new_mode, "showmode")
                self._arm_showmode()

        # Watchdog: without a fresh scan we refuse to drive.
        age = now - self._last_scan_mono
        if self._scan is None or age > self._scan_timeout_s:
            self._publish_stop()
            self.get_logger().warn(
                "No LiDAR scan for %.2fs; holding still." % age,
                throttle_duration_sec=2.0,
            )
            return

        front, left, right = self._sector_clearances(self._scan)
        if self._mode == "cute":
            cmd = self._cute_command(front, left, right)
        else:
            cmd = self._compute_command(front, left, right)
        self._cmd_pub.publish(cmd)
        self._integrate_pose(cmd)

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _integrate_pose(self, cmd: Twist) -> None:
        """Dead-reckon the pose from the commanded velocity (open loop)."""
        vx = cmd.linear.x
        vy = cmd.linear.y
        wz = cmd.angular.z
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        self._px += (vx * c - vy * s) * self._dt
        self._py += (vx * s + vy * c) * self._dt
        self._yaw = self._wrap(self._yaw + wz * self._dt)

    def _avoidance_command(self, front: float, left: float, right: float):
        """Shared obstacle reaction. Returns a Twist, or None if the way is clear."""
        if front <= self._stop_distance:
            if front <= self._backup_distance or abs(self._avoid_turn_dir) < 0.5:
                self._avoid_turn_dir = 1.0 if left >= right else -1.0

        if front <= self._backup_distance:
            cmd = Twist()
            cmd.linear.x = -self._reverse_speed
            cmd.angular.z = self._avoid_turn_dir * self._turn_speed
            return cmd

        if front <= self._stop_distance:
            cmd = Twist()
            cmd.angular.z = self._avoid_turn_dir * self._turn_speed
            return cmd

        # Way is clear; release the avoidance commitment.
        self._avoid_turn_dir = 0.0
        return None

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
        """Curious mode: the default roaming/obstacle-avoiding behaviour."""
        avoid = self._avoidance_command(front, left, right)
        if avoid is not None:
            self._reset_wander()
            return avoid

        cmd = Twist()

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

    # ------------------------------------------------------------------
    # Cute mode
    # ------------------------------------------------------------------
    def _cute_command(self, front: float, left: float, right: float) -> Twist:
        """Cute mode: stay near the start, stand still often, do little moves."""
        avoid = self._avoidance_command(front, left, right)
        if avoid is not None:
            # Obstacle takes priority; re-pick a cute phase once clear.
            self._cute_phase = None
            return avoid

        self._cute_elapsed += self._dt
        if self._cute_phase is None or self._cute_elapsed >= self._cute_duration:
            self._pick_cute_phase()

        return self._execute_cute_phase(front, left, right)

    def _pick_cute_phase(self) -> None:
        self._cute_elapsed = 0.0
        self._cute_dir = random.choice([-1.0, 1.0])

        # If we have drifted beyond the leash, head back toward the start.
        if math.hypot(self._px, self._py) > self._cute_radius:
            self._cute_phase = "RETURN"
            self._cute_duration = random.uniform(0.8, 1.6)
            return

        # Otherwise pick a cute phase, weighted heavily toward standing still.
        phase = random.choice(
            ["IDLE", "IDLE", "IDLE", "WIGGLE", "SPIN", "STRAFE", "SCOOT", "NOD"]
        )
        self._cute_phase = phase
        self._cute_duration = {
            "IDLE": random.uniform(1.5, 3.0),
            "WIGGLE": random.uniform(1.0, 1.8),
            "SPIN": random.uniform(0.6, 1.3),
            "STRAFE": random.uniform(0.8, 1.5),
            "SCOOT": random.uniform(0.5, 1.1),
            "NOD": random.uniform(1.0, 1.6),
        }[phase]

    def _execute_cute_phase(self, front: float, left: float, right: float) -> Twist:
        cmd = Twist()
        phase = self._cute_phase
        two_pi = 2.0 * math.pi

        if phase == "IDLE":
            # Stand still, drifting gently back to the starting orientation.
            yaw_err = self._wrap(0.0 - self._yaw)
            if abs(yaw_err) > 0.12:
                cmd.angular.z = max(-0.6, min(0.6, 1.2 * yaw_err))

        elif phase == "WIGGLE":
            cmd.angular.z = self._cute_wiggle_speed * math.sin(
                two_pi * (self._cute_elapsed / 0.5)
            )

        elif phase == "SPIN":
            cmd.angular.z = self._cute_dir * self._cute_spin_speed

        elif phase == "STRAFE":
            left_open = left > self._stop_distance
            right_open = right > self._stop_distance
            d = self._cute_dir
            if d > 0 and not left_open and right_open:
                d = -1.0
            elif d < 0 and not right_open and left_open:
                d = 1.0
            if (d > 0 and left_open) or (d < 0 and right_open):
                cmd.linear.y = d * self._cute_strafe_speed
            # else: boxed in on both sides, just stay put

        elif phase == "SCOOT":
            cmd.linear.x = self._cute_forward_speed

        elif phase == "NOD":
            cmd.linear.x = self._cute_forward_speed * 0.6 * math.sin(
                two_pi * (self._cute_elapsed / 0.6)
            )

        elif phase == "RETURN":
            bearing = math.atan2(-self._py, -self._px)
            yaw_err = self._wrap(bearing - self._yaw)
            cmd.angular.z = max(-self._turn_speed, min(self._turn_speed, 1.5 * yaw_err))
            if abs(yaw_err) < 0.5 and front > self._stop_distance:
                cmd.linear.x = self._cute_forward_speed

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
