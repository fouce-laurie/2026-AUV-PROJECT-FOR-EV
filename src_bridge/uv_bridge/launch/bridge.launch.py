from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the micro-ROS bridge node"""

    # Declare arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Bridge node
    bridge_node = Node(
        package='uv_bridge',
        executable='microros_bridge',
        name='microros_bridge',
        output='screen',
        parameters=[],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[]
    )

    return LaunchDescription([
        log_level_arg,
        LogInfo(msg='Starting Micro-ROS Bridge Node...'),
        bridge_node
    ])
