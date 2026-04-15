from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	foxglove_port = LaunchConfiguration("foxglove_port")
	foxglove_address = LaunchConfiguration("foxglove_address")
	use_sim_time = LaunchConfiguration("use_sim_time")

	bringup_launch = PathJoinSubstitution(
		[FindPackageShare("robot_description"), "launch", "bringup.launch.py"]
	)
	bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(bringup_launch),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"use_rviz": "false",
		}.items(),
	)

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
		DeclareLaunchArgument("use_sim_time", default_value="false"),
		DeclareLaunchArgument("foxglove_port", default_value="8765"),
		DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
		bringup,
		foxglove,
		launch_control,
	])
