from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	foxglove_port = LaunchConfiguration("foxglove_port")
	foxglove_address = LaunchConfiguration("foxglove_address")
	use_sim_time = LaunchConfiguration("use_sim_time")
	bringup_use_ros2_control = LaunchConfiguration("bringup_use_ros2_control")
	bringup_use_joint_state_publisher = LaunchConfiguration("bringup_use_joint_state_publisher")
	connect_zed_tf = LaunchConfiguration("connect_zed_tf")
	zed_parent_frame = LaunchConfiguration("zed_parent_frame")
	zed_child_frame = LaunchConfiguration("zed_child_frame")
	zed_to_base_x = LaunchConfiguration("zed_to_base_x")
	zed_to_base_y = LaunchConfiguration("zed_to_base_y")
	zed_to_base_z = LaunchConfiguration("zed_to_base_z")
	zed_to_base_roll = LaunchConfiguration("zed_to_base_roll")
	zed_to_base_pitch = LaunchConfiguration("zed_to_base_pitch")
	zed_to_base_yaw = LaunchConfiguration("zed_to_base_yaw")
	# Only runs when use_sim_time is true, bridging Twist down to flat Float64MultiArray for Isaac Sim
	twist_bridge_node = Node(
		package="sora_base_control",
		executable="twist_to_array",
		name="twist_bridge",
		output="screen",
		condition=IfCondition(use_sim_time),
		parameters=[{"use_sim_time": use_sim_time}],
	)

	bringup_launch = PathJoinSubstitution(
		[FindPackageShare("robot_description"), "launch", "bringup.launch.py"]
	)
	bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(bringup_launch),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"use_rviz": "false",
			"use_ros2_control": bringup_use_ros2_control,
			"use_joint_state_publisher": bringup_use_joint_state_publisher,
		}.items(),
	)

	foxglove = Node(
		package="foxglove_bridge",
		executable="foxglove_bridge",
		output="screen",
		parameters=[{
			"port": foxglove_port,
			"address": foxglove_address,
			# Suppress errors from missing zed_interfaces on the Pi
			"topic_whitelist": [r"^(?!/zedm/zed_node/depth/depth_info$).*"],
			"service_whitelist": [r"^(?!/zedm/zed_node/(set_pose|set_roi|start_svo_rec)$).*"],
		}],
	)

	launch_control = Node(
		package="sora_slam",
		executable="launch_control",
		output="screen",
	)

	# If the ZED publishes odom->zedm_base_link (common default), this bridges it to the
	# robot model by making base_link a child of zedm_base_link (no TF conflicts).
	# Assumes the ZED "base" frame is collocated with the physical camera center.
	zed_tf_bridge = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		name="zed_tf_bridge",
		output="screen",
		condition=IfCondition(connect_zed_tf),
		# parent: ZED base frame (e.g. zedm_base_link), child: base_link
		# base_link->zed_camera_center in URDF is (0.160, 0.0, 0.069)
		arguments=[
			'--x', zed_to_base_x, '--y', zed_to_base_y, '--z', zed_to_base_z,
			'--yaw', zed_to_base_yaw, '--pitch', zed_to_base_pitch, '--roll', zed_to_base_roll,
			'--frame-id', zed_parent_frame,
			'--child-frame-id', zed_child_frame,
		],
	)

	sim_robot_bringup_launch = PathJoinSubstitution(
		[FindPackageShare("sora_slam"), "launch", "sora_robot_bringup.launch.py"]
	)

	sim_robot_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(sim_robot_bringup_launch),
		condition=IfCondition(use_sim_time),
		launch_arguments={
			"use_sim_time": use_sim_time,
			"use_hardware": "false",
			"enable_rviz": "false",
			"start_robot_state_publisher": "false",
		}.items(),
	)

	return LaunchDescription([
		DeclareLaunchArgument("use_sim_time", default_value="false"),
		DeclareLaunchArgument(
			"connect_zed_tf",
			default_value="false",
			description="Publish a static TF zedm_base_link->base_link so the robot model follows the ZED odom/map tree",
		),
		DeclareLaunchArgument(
			"zed_parent_frame",
			default_value="zedm_base_link",
			description="ZED frame that moves with odometry (commonly: zedm_base_link or zed_base_link)",
		),
		DeclareLaunchArgument(
			"zed_child_frame",
			default_value="base_link",
			description="Robot base frame to attach under the ZED odom tree (usually base_link)",
		),
		DeclareLaunchArgument(
			"zed_to_base_x",
			default_value="-0.160",
			description="Static TF translation x from zed_parent_frame to zed_child_frame (meters)",
		),
		DeclareLaunchArgument(
			"zed_to_base_y",
			default_value="0.0",
			description="Static TF translation y from zed_parent_frame to zed_child_frame (meters)",
		),
		DeclareLaunchArgument(
			"zed_to_base_z",
			default_value="-0.069",
			description="Static TF translation z from zed_parent_frame to zed_child_frame (meters)",
		),
		DeclareLaunchArgument(
			"zed_to_base_roll",
			default_value="0.0",
			description="Static TF roll from zed_parent_frame to zed_child_frame (radians)",
		),
		DeclareLaunchArgument(
			"zed_to_base_pitch",
			default_value="0.0",
			description="Static TF pitch from zed_parent_frame to zed_child_frame (radians)",
		),
		DeclareLaunchArgument(
			"zed_to_base_yaw",
			default_value="0.0",
			description="Static TF yaw from zed_parent_frame to zed_child_frame (radians)",
		),
		DeclareLaunchArgument(
			"bringup_use_ros2_control",
			default_value="true",
			description="Whether robot_description/bringup.launch.py starts ros2_control controllers",
		),
		DeclareLaunchArgument(
			"bringup_use_joint_state_publisher",
			default_value="false",
			description="Publish zero joint states so the robot model renders in a stable pose (requires joint_state_publisher package)",
		),
		DeclareLaunchArgument("foxglove_port", default_value="8765"),
		DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
		bringup,
		foxglove,
		launch_control,
		zed_tf_bridge,
		twist_bridge_node,
		sim_robot_bringup,
	])
