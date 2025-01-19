from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def resolve_yaml_file(context, yaml_file):
    """Determine which YAML file to use based on the use_sim_time argument."""
    use_sim_time = context.launch_configurations['use_sim_time']

    pkg_dir = get_package_share_directory('path_planner_server')
    config_dir = 'config'

    if yaml_file == 'approach_service_server':
        pkg_dir = get_package_share_directory("attach_shelf")

    if use_sim_time.lower() == 'true':
        return os.path.join(pkg_dir, config_dir, f'{yaml_file}_sim.yaml')
    else:
        return os.path.join(pkg_dir, config_dir, f'{yaml_file}_real.yaml')

def resolve_command_topic(context):
    use_sim_time = context.launch_configurations['use_sim_time']
    topic = 'diffbot_base_controller/cmd_vel_unstamped' if use_sim_time.lower() == 'true' else 'cmd_vel'
    return topic

def add_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'pathplanning.rviz')

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            remappings=[
                ('cmd_vel', resolve_command_topic(context))
            ],
            parameters=[{'use_sim_time': use_sim_time}, resolve_yaml_file(context, 'controller')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, resolve_yaml_file(context, 'planner')],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            remappings=[
                ('cmd_vel', resolve_command_topic(context))
            ],
            parameters=[{'use_sim_time': use_sim_time}, resolve_yaml_file(context, 'recoveries')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, resolve_yaml_file(context, 'bt_navigator')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
            }]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ), 

        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen',
            parameters=[resolve_yaml_file(context, 'approach_service_server')],
        ),
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
