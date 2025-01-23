import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def resolve_yaml_file(context, yaml_file):
    """Determine which YAML file to use based on the use_sim_time argument."""
    use_sim_time = context.launch_configurations['use_sim_time']

    pkg_dir = get_package_share_directory('helper')
    config_dir = 'config'

    if use_sim_time.lower() == 'true':
        return os.path.join(pkg_dir, config_dir, f'{yaml_file}_sim.yaml')
    else:
        return os.path.join(pkg_dir, config_dir, f'{yaml_file}_real.yaml')

def add_nodes(context):
    return [
        Node(
            package='helper',
            executable='helper_node',
            output='screen',
            parameters=[resolve_yaml_file(context, 'helper_config')],
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