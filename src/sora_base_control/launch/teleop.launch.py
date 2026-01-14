from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('sora_base_control')

    teleop_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'teleop_joy.yaml'
    ])

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # typical for Bluetooth Xbox; adjust if needed
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[teleop_config],
            remappings=[('cmd_vel', 'cmd_vel')],
            output='screen'
        ),

        Node(
            package='sora_base_control',
            executable='mecanum_kinematics',
            name='mecanum_kinematics',
            output='screen'
        ),
    ])
