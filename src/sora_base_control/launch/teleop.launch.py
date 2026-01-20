from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    pkg_share = FindPackageShare('sora_base_control')
    teleop_config = PathJoinSubstitution([pkg_share, 'config', 'teleop_joy.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
                'use_sim_time': use_sim_time
            }]
        ),

        # Teleop twist joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[teleop_config, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', '/cmd_vel')]
        ),

        # Mecanum kinematics converter
        Node(
            package='sora_base_control',
            executable='mecanum_kinematics',
            name='mecanum_kinematics',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
