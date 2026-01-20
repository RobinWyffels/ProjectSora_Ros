from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'sora.urdf'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_file,
            description='Path to robot URDF file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', LaunchConfiguration('model')])
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
