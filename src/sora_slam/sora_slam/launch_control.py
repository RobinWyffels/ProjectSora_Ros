import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LaunchControl(Node):
	def __init__(self):
		super().__init__("launch_control")

		self._lock = threading.Lock()
		self._procs: dict[str, subprocess.Popen] = {}

		# Friendly name -> ros2 launch command
		self._commands: dict[str, list[str]] = {
			"slam": [
				"ros2",
				"launch",
				"sora_slam",
				"sora_slam_toolbox.launch.py",
				"enable_rviz:=false",
				"start_robot_state_publisher:=false",
			],
			"teleop": [
				"ros2",
				"launch",
				"sora_base_control",
				"teleop.launch.py",
			],
		}

		self.create_subscription(String, "/launch_control/command", self._on_command, 10)
		self.get_logger().info(
			"LaunchControl ready on /launch_control/command (std_msgs/String). "
			"Use: 'start slam', 'stop slam', 'start teleop', 'stop teleop'."
		)

	def _on_command(self, msg: String) -> None:
		text = (msg.data or "").strip()
		if not text:
			return

		parts = text.split()
		if len(parts) < 2:
			self.get_logger().warn("Command format: 'start <name>' or 'stop <name>'")
			return

		verb = parts[0].lower()
		name = parts[1]

		if verb == "start":
			self.start(name)
		elif verb == "stop":
			self.stop(name)
		else:
			self.get_logger().warn(f"Unknown verb '{verb}' (use start/stop)")

	def start(self, name: str) -> None:
		with self._lock:
			if name not in self._commands:
				self.get_logger().error(
					f"Unknown launch '{name}'. Known: {sorted(self._commands.keys())}"
				)
				return

			existing = self._procs.get(name)
			if existing is not None and existing.poll() is None:
				self.get_logger().info(f"'{name}' already running")
				return

			cmd = self._commands[name]
			self.get_logger().info(f"Starting '{name}': {' '.join(cmd)}")

			proc = subprocess.Popen(
				cmd,
				env=os.environ.copy(),
				stdout=None,
				stderr=None,
				start_new_session=True,  # new process group (WSL/Linux)
			)
			self._procs[name] = proc

	def stop(self, name: str) -> None:
		with self._lock:
			proc = self._procs.get(name)
			if proc is None or proc.poll() is not None:
				self.get_logger().info(f"'{name}' not running")
				self._procs.pop(name, None)
				return

			self.get_logger().info(f"Stopping '{name}' (SIGINT)")
			try:
				os.killpg(proc.pid, signal.SIGINT)
			except ProcessLookupError:
				self._procs.pop(name, None)
				return

		timeout_s = 5.0
		t0 = time.time()
		while time.time() - t0 < timeout_s:
			if proc.poll() is not None:
				with self._lock:
					self._procs.pop(name, None)
				self.get_logger().info(f"Stopped '{name}'")
				return
			time.sleep(0.1)

		self.get_logger().warn(f"'{name}' did not exit; killing (SIGKILL)")
		try:
			os.killpg(proc.pid, signal.SIGKILL)
		except ProcessLookupError:
			pass
		with self._lock:
			self._procs.pop(name, None)

	def destroy_node(self) -> None:
		for name in list(self._procs.keys()):
			try:
				self.stop(name)
			except Exception:
				pass
		super().destroy_node()


def main() -> None:
	rclpy.init()
	node = LaunchControl()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()
