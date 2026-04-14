from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	foxglove_port = LaunchConfiguration("foxglove_port")
	foxglove_address = LaunchConfiguration("foxglove_address")

	foxglove = Node(
		package="foxglove_bridge",
		executable="foxglove_bridge",
		output="screen",
		parameters=[{
			"port": foxglove_port,
			"address": foxglove_address,
		}],
	)

	launch_control = Node(
		package="sora_slam",
		executable="launch_control",
		output="screen",
	)

	return LaunchDescription([
		DeclareLaunchArgument("foxglove_port", default_value="8765"),
		DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
		foxglove,
		launch_control,
	])
