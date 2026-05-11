from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_ai = LaunchConfiguration('enable_ai')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_ai',
            default_value='true',
            description='Start AI nodes during real-hardware tests.',
        ),
        Node(
            package='uv_hm',
            executable='uv_hmu',
            output='screen',
        ),
        Node(
            package='uv_vision',
            executable='uv_capture',
            output='screen',
        ),
        Node(
            package='uv_ai',
            executable='uv_detect_demo',
            condition=IfCondition(enable_ai),
            output='screen',
        ),
        Node(
            package='uv_ai',
            executable='uv_segment',
            condition=IfCondition(enable_ai),
            output='screen',
        ),
    ])
