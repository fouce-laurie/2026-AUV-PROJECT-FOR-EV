from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_ai = LaunchConfiguration('enable_ai')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    launch_agent = LaunchConfiguration('launch_agent')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_ai',
            default_value='true',
            description='Start AI nodes.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='micro-ROS agent serial port.',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='micro-ROS agent baudrate.',
        ),
        DeclareLaunchArgument(
            'launch_agent',
            default_value='true',
            description='Launch micro-ROS agent (disable if already running).',
        ),

        # micro-ROS agent
        ExecuteProcess(
            condition=IfCondition(launch_agent),
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial', '--dev', serial_port, '-b', baudrate,
            ],
            output='screen',
        ),

        # heartbeat node (解锁心跳)
        Node(
            package='uv_hm',
            executable='uv_hmu',
            output='screen',
        ),

        # camera
        Node(
            package='uv_vision',
            executable='uv_capture',
            output='screen',
        ),

        # AI nodes (optional)
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
