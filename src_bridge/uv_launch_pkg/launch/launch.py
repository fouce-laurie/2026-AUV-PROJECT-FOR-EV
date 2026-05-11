from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uv_hm',
            executable='uv_hmu',
            output='screen'
        ),
        
        Node(
            package='uv_vision',
            executable='uv_capture',
            output='screen'
        ),
        
        Node(
            package='uv_ai',  # 替换为你的包名
            executable='uv_detect_demo',  # 替换为你的节点可执行文件名
            output='screen', # 输出到终端
        ),
        

        Node(
            package='uv_ai',
            executable='uv_segment',
            output='screen'
        )
        

    ])
