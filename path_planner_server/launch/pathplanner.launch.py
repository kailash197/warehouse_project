from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def resolve_config_file(context, config_file):
    """Determine which YAML file to use based on the use_sim_time argument."""
    use_sim_time = context.launch_configurations['use_sim_time']

    pkg_dir = get_package_share_directory('path_planner_server')
    config_dir = 'config'
    extension = 'yaml'

    if config_file == 'approach_service_server':
        pkg_dir = get_package_share_directory("attach_shelf")
        
    if config_file == 'helper_config':
        pkg_dir = get_package_share_directory("helper")

    if config_file == 'pathplanning_rviz':
        config_dir = 'rviz'
        extension = 'rviz'

    if use_sim_time.lower() == 'true':
        return os.path.join(pkg_dir, config_dir, f'{config_file}_sim.{extension}')
    else:
        return os.path.join(pkg_dir, config_dir, f'{config_file}_real.{extension}')

def resolve_command_topic(context):
    use_sim_time = context.launch_configurations['use_sim_time']
    topic = 'diffbot_base_controller/cmd_vel_unstamped' if use_sim_time.lower() == 'true' else 'cmd_vel'
    return topic

def add_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time')

    return [
        Node(
            package='helper',
            executable='helper_node',
            output='screen',
            parameters=[resolve_config_file(context, 'helper_config')],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            remappings=[
                ('cmd_vel', resolve_command_topic(context))
            ],
            parameters=[{'use_sim_time': use_sim_time}, resolve_config_file(context, 'controller')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, resolve_config_file(context, 'planner')],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            remappings=[
                ('cmd_vel', resolve_command_topic(context))
            ],
            parameters=[{'use_sim_time': use_sim_time}, resolve_config_file(context, 'recoveries')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, resolve_config_file(context, 'bt_navigator')]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[resolve_config_file(context, 'filters')]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[resolve_config_file(context, 'filters')]),
        
        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen',
            parameters=[resolve_config_file(context, 'approach_service_server')],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', resolve_config_file(context, 'pathplanning_rviz')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
                                'filter_mask_server', 'costmap_filter_info_server']
            }]
        )
    ]


def generate_launch_description():

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time: True or False'
    )
    dynamic_nodes = OpaqueFunction(function=add_nodes)

    return LaunchDescription([
        declare_use_sim_time_arg,
        dynamic_nodes
    ])
