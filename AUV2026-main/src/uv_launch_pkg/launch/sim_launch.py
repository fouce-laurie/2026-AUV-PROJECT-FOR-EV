import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_stonefish_paths():
    stonefish_share = get_package_share_directory('stonefish_ros2')
    workspace_root = os.path.abspath(
        os.path.join(stonefish_share, '..', '..', '..', '..')
    )

    # Support both layouts:
    # 1) <workspace>/stonefish_ros2
    # 2) <workspace>/Cruise/stonefish_ros2
    direct_dir = os.path.join(workspace_root, 'stonefish_ros2')
    stonefish_dir = os.path.join(workspace_root, 'src', 'stonefish_ros2')
    stonefish_source_dir = direct_dir if os.path.exists(direct_dir) else stonefish_dir

    return {
        'stonefish_source_dir': stonefish_source_dir,
        'simulation_data': os.path.join(stonefish_source_dir, 'Data'),
        'default_scenario': os.path.join(stonefish_source_dir, 'underwater_test.scn'),
        'sauvc_pool_scenario': os.path.join(stonefish_source_dir, 'sauvc_pool.scn'),
    }


def _resolve_scene_alias(context, scene_map):
    # scenario_desc has highest priority: explicit path always wins.
    explicit_path = LaunchConfiguration('scenario_desc').perform(context).strip()
    if explicit_path:
        resolved = explicit_path
    else:
        scene_alias = LaunchConfiguration('scene').perform(context).strip().lower()
        resolved = scene_map.get(scene_alias, scene_map['test'])

    return [SetLaunchConfiguration('resolved_scenario_desc', resolved)]


def generate_launch_description():
    stonefish_paths = _default_stonefish_paths()
    default_simulation_data = stonefish_paths['simulation_data']
    default_scenario = stonefish_paths['default_scenario']
    scene_map = {
        'test': stonefish_paths['default_scenario'],
        'sauvc_pool': stonefish_paths['sauvc_pool_scenario'],
    }
    launch_pkg_share = get_package_share_directory('uv_launch_pkg')
    workspace_root = os.path.abspath(os.path.join(launch_pkg_share, '..', '..', '..', '..'))
    direct_datas = os.path.join(workspace_root, 'src', 'datas')
    datas_dir_path = os.path.join(workspace_root, 'src', 'datas')
    datas_dir = direct_datas if os.path.exists(direct_datas) else datas_dir_path

    simulation_data = LaunchConfiguration('simulation_data')
    resolved_scenario_desc = LaunchConfiguration('resolved_scenario_desc')
    simulation_rate = LaunchConfiguration('simulation_rate')
    window_res_x = LaunchConfiguration('window_res_x')
    window_res_y = LaunchConfiguration('window_res_y')
    rendering_quality = LaunchConfiguration('rendering_quality')
    enable_ai = LaunchConfiguration('enable_ai')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_data',
            default_value=default_simulation_data,
            description='Path to Stonefish Data directory (meshes, textures).',
        ),
        DeclareLaunchArgument(
            'scene',
            default_value='test',
            description='Built-in scenario alias: test or sauvc_pool.',
        ),
        DeclareLaunchArgument(
            'scenario_desc',
            default_value='',
            description='Optional absolute path to .scn file. Overrides scene alias when provided.',
        ),
        OpaqueFunction(function=lambda context: _resolve_scene_alias(context, scene_map)),
        DeclareLaunchArgument(
            'simulation_rate',
            default_value='100.0',
            description='Simulation rate in Hz.',
        ),
        DeclareLaunchArgument(
            'window_res_x',
            default_value='1920',
            description='Simulator window width.',
        ),
        DeclareLaunchArgument(
            'window_res_y',
            default_value='1080',
            description='Simulator window height.',
        ),
        DeclareLaunchArgument(
            'rendering_quality',
            default_value='high',
            description='Rendering quality: low, medium or high.',
        ),
        DeclareLaunchArgument(
            'enable_ai',
            default_value='false',
            description='Start AI nodes in simulation.',
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='Start stereo depth processing node in simulation.',
        ),
        Node(
            package='stonefish_ros2',
            executable='stonefish_simulator',
            namespace='stonefish_ros2',
            name='stonefish_simulator',
            arguments=[
                simulation_data,
                resolved_scenario_desc,
                simulation_rate,
                window_res_x,
                window_res_y,
                rendering_quality,
            ],
            output='screen',
        ),
        Node(
            package='uv_hm',
            executable='uv_sim_bridge',
            output='screen',
        ),
        Node(
            package='uv_vision',
            executable='uv_depthimg_sim',
            condition=IfCondition(LaunchConfiguration('enable_depth')),
            output='screen',
        ),
        Node(
            package='uv_ai',
            executable='uv_detect_demo',
            condition=IfCondition(enable_ai),
            arguments=[
                '--weights', os.path.join(workspace_root, 'src', 'datas', 'AUV2025V1.pt'),
                '--weights2', os.path.join(workspace_root, 'src', 'datas', 'AUV2025V1.pt'),
                '--front-params', os.path.join(workspace_root, 'src', 'datas', 'front.npz'),
                '--down-params', os.path.join(workspace_root, 'src', 'datas', 'down.npz'),
            ],
            output='screen',
        ),
        Node(
            package='uv_ai',
            executable='uv_segment',
            condition=IfCondition(enable_ai),
            arguments=['--weights', os.path.join(workspace_root, 'src', 'datas', 'redline-7-2.pt')],
            output='screen',
        ),
    ])
