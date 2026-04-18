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

		self._lock = threading.RLock()
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
			"sensors": [
				"ros2",
				"launch",
				"sora_slam",
				"sora_robot_bringup.launch.py",
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
			"Use: 'start slam', 'stop slam', 'start sensors', 'stop sensors', 'start teleop', 'start nav <mapname>', 'stop nav', 'save map <name> [timeout_s]'."
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
			self.start(name, parts[2:])
		elif verb == "stop":
			self.stop(name)
		elif verb == "save":
			self.save(name, parts[2:])
		else:
			self.get_logger().warn(f"Unknown verb '{verb}' (use start/stop)")

	def save(self, name: str, args: list[str]) -> None:
		if name != "map":
			self.get_logger().warn("Save format: 'save map <name> [timeout_s]'")
			return

		if not args:
			self.get_logger().warn("Save format: 'save map <name> [timeout_s]'")
			return

		map_name = args[0]
		try:
			timeout_s = float(args[1]) if len(args) >= 2 else 30.0
		except ValueError:
			timeout_s = 30.0

		# Run map saving in a separate thread so we don't block the ROS callback thread.
		threading.Thread(
			target=self._save_map_worker,
			args=(map_name, timeout_s),
			daemon=True,
		).start()

	def _save_map_worker(self, map_name: str, timeout_s: float) -> None:
		maps_dir = os.path.expanduser("~/maps")
		try:
			os.makedirs(maps_dir, exist_ok=True)
		except Exception as e:
			self.get_logger().error(f"Failed to create maps dir '{maps_dir}': {e}")
			return

		# -f expects a base path without extension; it will create .yaml + .pgm.
		map_base_path = os.path.join(maps_dir, map_name)

		cmd = [
			"ros2",
			"run",
			"nav2_map_server",
			"map_saver_cli",
			"-f",
			map_base_path,
			"--ros-args",
			"-p",
			f"save_map_timeout:={timeout_s}",
		]

		self.get_logger().info(
			f"Saving map to '{map_base_path}' (timeout {timeout_s:.1f}s): {' '.join(cmd)}"
		)
		try:
			result = subprocess.run(cmd, env=os.environ.copy(), capture_output=True, text=True)
		except FileNotFoundError:
			self.get_logger().error("Failed to run map_saver_cli: 'ros2' not found in PATH")
			return
		except Exception as e:
			self.get_logger().error(f"Failed to run map_saver_cli: {e}")
			return

		if result.returncode == 0:
			self.get_logger().info(f"Map saved: {map_base_path}.yaml + {map_base_path}.pgm")
			return

		stderr = (result.stderr or "").strip()
		stdout = (result.stdout or "").strip()
		if stdout:
			self.get_logger().info(stdout)
		if stderr:
			self.get_logger().warn(stderr)
		self.get_logger().error(f"Map save failed (exit {result.returncode})")

	def start(self, name: str, args: list[str]) -> None:
		with self._lock:
			if name not in self._commands and name != "nav":
				self.get_logger().error(
					f"Unknown launch '{name}'. Known: {sorted(self._commands.keys()) + ['nav']}"
				)
				return

			existing = self._procs.get(name)
			if existing is not None and existing.poll() is None:
				self.get_logger().info(f"'{name}' already running")
				return

			if name == "nav":
				if not args:
					self.get_logger().warn("Start nav format: 'start nav <mapname>'")
					return
				map_name = args[0]
				map_path = os.path.expanduser(f"~/maps/{map_name}.yaml")
				if not os.path.exists(map_path):
					self.get_logger().error(f"Map file not found: {map_path}")
					return
					
				# Stop teleop if it is currently running
				teleop_proc = self._procs.get("teleop")
				if teleop_proc is not None and teleop_proc.poll() is None:
					self.get_logger().info("Stopping active 'teleop' before starting navigation...")
					self.stop("teleop")
					
				# Implicitly start the lidar and sensors if not already running
				sensors_proc = self._procs.get("sensors")
				if sensors_proc is None or sensors_proc.poll() is not None:
					self.get_logger().info("Starting sensors implicitly for navigation...")
					sensors_cmd = self._commands["sensors"]
					self._procs["sensors"] = subprocess.Popen(
						sensors_cmd, env=os.environ.copy(), stdout=None, stderr=None, start_new_session=True
					)
					
				# Path to custom Nav2 params file
				params_path = os.path.expanduser("~/sora_ws/src/ProjectSora_Ros/src/sora_slam/config/nav2_params.yaml")
				
				cmd = [
					"ros2",
					"launch",
					"nav2_bringup",
					"bringup_launch.py",
					f"map:={map_path}",
					"use_sim_time:=false",
					f"params_file:={params_path}",
				]
			else:
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
				
				# If we're stopping nav, let's also stop implicitly launched sensors if they are running
				if name == "nav" and "sensors" in self._procs:
					self.get_logger().info("Also stopping implicitly launched 'sensors'")
					self._stop_child("sensors")
					
				return

			self.get_logger().info(f"Stopping '{name}' (SIGINT)")
			self._send_sigint(proc, name)
			
			# If we're stopping nav, also stop implicitly launched sensors
			if name == "nav":
				sensors_proc = self._procs.get("sensors")
				if sensors_proc and sensors_proc.poll() is None:
					self.get_logger().info("Also stopping implicitly launched 'sensors'")
					self._send_sigint(sensors_proc, "sensors")

		self._wait_and_kill(name, proc)
		
		# Now ensure sensors is also cleanly killed if we were stopping nav
		if name == "nav":
			sensors_proc = self._procs.get("sensors")
			if sensors_proc:
				self._wait_and_kill("sensors", sensors_proc)
				
	def _send_sigint(self, proc, name: str) -> None:
		try:
			os.killpg(proc.pid, signal.SIGINT)
		except ProcessLookupError:
			self._procs.pop(name, None)

	def _wait_and_kill(self, name: str, proc) -> None:
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

	def _stop_child(self, name: str) -> None:
		proc = self._procs.get(name)
		if proc and proc.poll() is None:
			self._send_sigint(proc, name)
			self._wait_and_kill(name, proc)

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
