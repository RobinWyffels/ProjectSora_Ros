#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class CmdVelOdometry(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_odometry')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)

        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self._cmd_vel_timeout = Duration(seconds=self.get_parameter('cmd_vel_timeout_s').get_parameter_value().double_value)

        if update_rate <= 0.0:
            update_rate = 50.0

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        self._last_cmd_vel: Twist = Twist()
        self._last_cmd_time = self.get_clock().now()

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_update_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._timer = self.create_timer(1.0 / update_rate, self._on_timer)

        self.get_logger().info(
            f"Publishing open-loop odom on /odom and TF {self._odom_frame}->{self._base_frame} "
            f"(update_rate={update_rate:.1f}Hz, timeout={self._cmd_vel_timeout.nanoseconds/1e9:.2f}s)"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_time = self.get_clock().now()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self._last_update_time = now

        # Treat cmd_vel as base-frame velocities.
        vx = float(self._last_cmd_vel.linear.x)
        vy = float(self._last_cmd_vel.linear.y)
        wz = float(self._last_cmd_vel.angular.z)

        # If cmd_vel is stale, stop integrating motion.
        if (now - self._last_cmd_time) > self._cmd_vel_timeout:
            vx = 0.0
            vy = 0.0
            wz = 0.0

        # Integrate in odom frame.
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        v_odom_x = vx * cos_yaw - vy * sin_yaw
        v_odom_y = vx * sin_yaw + vy * cos_yaw

        self._x += v_odom_x * dt
        self._y += v_odom_y * dt
        self._yaw += wz * dt

        # Normalize yaw to [-pi, pi]
        self._yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi

        qz = math.sin(self._yaw * 0.5)
        qw = math.cos(self._yaw * 0.5)

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist = self._last_cmd_vel
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
