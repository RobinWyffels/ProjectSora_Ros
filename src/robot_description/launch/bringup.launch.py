# src/robot_description/launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

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

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    pkg_share = FindPackageShare('robot_description')
    
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'sora.urdf'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])
    controllers_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # Robot State Publisher
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        )
    )

    # Controller Manager
    nodes.append(
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_config, {'use_sim_time': use_sim_time}],
            output='screen',
            remappings=[
                ('/controller_manager/robot_description', '/robot_description'),
            ]
        )
    )

    # Joint State Broadcaster Spawner
    nodes.append(
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen'
        )
    )

    # Velocity Controller Spawner
    nodes.append(
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller', '-c', '/controller_manager'],
            output='screen'
        )
    )

    # RViz (optional)
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    return nodes

def generate_launch_description():
    hardware_plugin_arg = DeclareLaunchArgument(
        'hardware_plugin',
        default_value='mock_components/GenericSystem',
        description='ros2_control hardware plugin to use'
    )

    return LaunchDescription([
        hardware_plugin_arg,
        OpaqueFunction(function=_launch_setup)
    ])
