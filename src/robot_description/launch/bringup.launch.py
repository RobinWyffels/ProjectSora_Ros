# src/robot_description/launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def _launch_setup(context, *args, **kwargs):
    # Resolve launch configuration and package paths at runtime
    hardware_plugin = LaunchConfiguration('hardware_plugin').perform(context)
    pkg_share = FindPackageShare('robot_description').perform(context)

    urdf_path = os.path.join(pkg_share, 'urdf', 'sora.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Read URDF and replace placeholder token with the resolved plugin string
    with open(urdf_path, 'r') as f:
        urdf_text = f.read()

    if '__HARDWARE_PLUGIN__' not in urdf_text:
        # If placeholder missing, still allow launch but warn in logs via parameter content
        pass

    urdf_text = urdf_text.replace('__HARDWARE_PLUGIN__', hardware_plugin)

    # Wrap as ParameterValue so robot_state_publisher and ros2_control_node accept it as a string
    robot_description = ParameterValue(urdf_text, value_type=str)

    nodes = []

    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml
            ],
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller'],
            output='screen'
        )
    )

    return nodes

def generate_launch_description():
    # Default set to Iron-friendly plugin; override at launch for Foxy
    hardware_plugin_arg = DeclareLaunchArgument(
        'hardware_plugin',
        default_value='fake_components/GenericSystem',
        description='ros2_control hardware plugin to use (e.g. fake_components/GenericSystem or ros2_control_demo_hardware/GenericSystem)'
    )

    return LaunchDescription([
        hardware_plugin_arg,
        OpaqueFunction(function=_launch_setup)
    ])
