"""Innovation Cup demo launch.

Brings up everything needed for the controller-triggered LiDAR wander:

  * YDLidar driver            -> publishes /scan
  * joy_node                  -> publishes /joy (Xbox controller)
  * mecanum_kinematics        -> /cmd_vel  -> /velocity_controller/commands
  * motor_driver_node         -> drives the Cytron MDD10A motor controllers
  * demo_wander               -> the reactive behaviour (waits for A, stops on B)

Start it with:

    ros2 launch sora_base_control demo.launch.py

Then connect the Xbox controller and press **A** to start roaming, **B** to stop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    joy_dev = LaunchConfiguration("joy_dev")
    lidar_params_file = LaunchConfiguration("lidar_params_file")

    ydlidar_share = FindPackageShare("ydlidar_ros2_driver")
    default_lidar_params = PathJoinSubstitution([ydlidar_share, "params", "Tmini.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device for the Xbox controller.",
        ),
        DeclareLaunchArgument(
            "lidar_params_file",
            default_value=default_lidar_params,
            description="YDLidar driver params (Tmini.yaml by default).",
        ),

        # LiDAR -> /scan
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[lidar_params_file],
        ),

        # Xbox controller -> /joy
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
                "use_sim_time": use_sim_time,
            }],
        ),

        # /cmd_vel -> wheel commands
        Node(
            package="sora_base_control",
            executable="mecanum_kinematics",
            name="mecanum_kinematics",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                # Wheel angular speed (rad/s) that maps to full motor output.
                # Keeps teleop top speed while giving proportional low-speed control.
                "max_wheel_speed": 6.0,
            }],
        ),

        # Wheel commands -> motors
        Node(
            package="sora_motor_driver",
            executable="motor_driver_node",
            name="motor_driver_node",
            output="screen",
            parameters=[{
                # Overcome stiction: any non-zero command moves at least this
                # much throttle (in the driver's scaled 0..max_throttle units).
                "min_move_throttle": 0.22,
            }],
        ),

        # Reactive wander behaviour (A = start, B = stop)
        Node(
            package="sora_base_control",
            executable="demo_wander",
            name="demo_wander",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
