import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    map_server_dir = get_package_share_directory('map_server')
    localization_server_dir = get_package_share_directory('localization_server')
    rviz_config_file = os.path.join(localization_server_dir, 'rviz', 'localization.rviz')
    amcl_yaml = os.path.join(localization_server_dir, 'config', 'amcl_config_sim.yaml')

    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file to load'
    )
    map_file = LaunchConfiguration('map_file')

    # Dynamically construct the map file path
    map_path = PathJoinSubstitution([map_server_dir, 'config', map_file])

    return LaunchDescription([
        declare_map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename':map_path}
                       ]),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}
                        ]),
         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
