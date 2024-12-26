from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def resolve_yaml_file(context, yaml_file):
    """Determine which YAML file to use based on the use_sim_time argument."""
    path_planner_server_dir = get_package_share_directory('path_planner_server')
    use_sim_time = context.launch_configurations['use_sim_time']
    if use_sim_time.lower() == 'true':
        return os.path.join(path_planner_server_dir, 'config', f'{yaml_file}_sim.yaml')
    else:
        return os.path.join(path_planner_server_dir, 'config', f'{yaml_file}_real.yaml')

def add_nodes(context):
    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            remappings=[
                ('cmd_vel', 'diffbot_base_controller/cmd_vel_unstamped')
            ],
            parameters=[resolve_yaml_file(context, 'controller')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[resolve_yaml_file(context, 'planner')],
        ),

        Node(

            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[resolve_yaml_file(context, 'recoveries')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[resolve_yaml_file(context, 'bt_navigator')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ]


def generate_launch_description():

    path_planner_server_dir = get_package_share_directory('path_planner_server')
    rviz_config_file = os.path.join(path_planner_server_dir, 'rviz', 'pathplanning.rviz')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time: True or False'
    )

    dynamic_nodes = OpaqueFunction(function=add_nodes)


    return LaunchDescription([
        declare_use_sim_time_arg,
        dynamic_nodes,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
