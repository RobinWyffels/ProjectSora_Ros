from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_hardware = LaunchConfiguration("use_hardware")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    enable_rviz = LaunchConfiguration("enable_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")

    # Frames can be overridden to avoid conflicts with other TF publishers (e.g. ZED on another machine).
    # LiDAR-only mapping: set odom_frame==base_frame so slam_toolbox publishes map->base_link directly.
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    robot_description_share = FindPackageShare("robot_description")
    urdf_file = PathJoinSubstitution([robot_description_share, "urdf", "sora.urdf"])

    ydlidar_share = FindPackageShare("ydlidar_ros2_driver")
    # Use the Tmini profile by default (stable front-facing scan for your setup).
    default_lidar_params = PathJoinSubstitution([ydlidar_share, "params", "Tmini.yaml"])

    sora_slam_share = FindPackageShare("sora_slam")
    default_slam_params = PathJoinSubstitution([sora_slam_share, "config", "slam_toolbox_params.yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(start_robot_state_publisher),
        parameters=[{
            "robot_description": Command(["cat ", urdf_file]),
            "use_sim_time": use_sim_time,
        }],
    )

    ydlidar_node = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_hardware),
        parameters=[lidar_params_file],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
            },
        ],
    )

    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 0.0,

        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
    )

    # Must exactly match the ZED node's base_frame (this is what the Jetson ZED node considers its base)
    zed_parent_frame = 'zedm_base_link' 
    
    # CRITICAL TF FIX: The Jetson publishes odom -> zedm_base_link. 
    # To prevent 'zedm_base_link' from having two parents (which breaks Foxglove tracking),
    # we must make the robot's base_link a CHILD of the camera's base frame!
    # The physical ZED mount is at x=0.160, z=0.069 from base_link. 
    # Therefore, base_link is at x=-0.160, z=-0.069 relative to the camera.
    zed_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_base_link',
        arguments=[
            '--x', '-0.160', '--y', '0', '--z', '-0.069',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', zed_parent_frame,
            '--child-frame-id', 'base_link'
        ],
        condition=IfCondition(LaunchConfiguration('use_hardware'))
    )

    # Reconnect the ZED camera properly to base_link during simulation
    sim_zed_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_zed_to_base_link',
        arguments=[
            '--x', '0.160', '--y', '0', '--z', '0.069',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', zed_parent_frame
        ],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    isaac_optical_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='isaac_optical_tf_node',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '-1.5708', '--pitch', '0', '--roll', '-1.5708',
            '--frame-id', 'zedm_base_link',
            '--child-frame-id', 'zedm_left_camera_optical_frame'
        ],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_hardware", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),

        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Start robot_state_publisher (set false if another bringup already publishes TF/URDF)",
        ),

        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=default_lidar_params,
            description="YDLidar driver params (Tmini.yaml by default).",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params,
            description="slam_toolbox params yaml.",
        ),

        DeclareLaunchArgument(
            "map_frame",
            default_value="slam_map",
            description="SLAM map frame (use slam_map to avoid TF conflicts with other map/odom publishers).",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom", 
            description="Odometry frame from Jetson ZED",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link", 
            description="Robot base frame.",
        ),

        robot_state_publisher,
        ydlidar_node,
        slam_toolbox,
        lifecycle_manager_slam,
        rviz_node,
        zed_tf_node,
        sim_zed_tf_node,
        isaac_optical_tf_node,
    ])

    # Simulations
    sim_slam = [
        "ros2", "launch", "sora_slam", "sora_slam_toolbox.launch.py",
        "enable_rviz:=false", "start_robot_state_publisher:=false",
        "use_sim_time:=true", "use_hardware:=false",
    ]
    sim_sensors = [
        "ros2", "launch", "sora_slam", "sora_robot_bringup.launch.py",
        "enable_rviz:=false", "start_robot_state_publisher:=false",
        "use_sim_time:=true", "use_hardware:=false",
    ]
    sim_teleop = [
        "ros2", "launch", "sora_base_control", "teleop.launch.py",
        "use_sim_time:=true",
    ]