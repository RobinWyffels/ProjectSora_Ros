from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    enable_rviz = LaunchConfiguration("enable_rviz")

    robot_description_share = FindPackageShare("robot_description")
    urdf_file = PathJoinSubstitution([robot_description_share, "urdf", "sora.urdf"])

    ydlidar_share = FindPackageShare("ydlidar_ros2_driver")
    default_lidar_params = PathJoinSubstitution([ydlidar_share, "params", "Tmini.yaml"])

    sora_slam_share = FindPackageShare("sora_slam")
    default_slam_params = PathJoinSubstitution([sora_slam_share, "config", "slam_toolbox_params.yaml"])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
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
        parameters=[lidar_params_file],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
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
            "bond_timeout": 0,

        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),

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

        robot_state_publisher,
        ydlidar_node,
        slam_toolbox,
        lifecycle_manager_slam,
        rviz_node,
    ])