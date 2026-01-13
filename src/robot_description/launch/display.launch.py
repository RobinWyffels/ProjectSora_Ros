from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('robot_description')

    urdf_path = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'sora.urdf'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=urdf_path,
            description='Absolute path to robot URDF file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'rviz',
                'display.rviz'
            ])]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        )
    ])
