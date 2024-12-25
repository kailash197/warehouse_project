import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation time (True or False)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename_sim = 'cartographer_sim.lua'
    configuration_basename_real = 'cartographer_real.lua'
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    rviz_config_file = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapping.rviz')

    return LaunchDescription([
        use_sim_time_arg,

        Node(
            package='cartographer_ros', 
            executable='cartographer_node',
            name='cartographer_node_sim',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename_sim],
            condition=IfCondition(use_sim_time)
        ),

        Node(
            package='cartographer_ros', 
            executable='cartographer_node',
            name='cartographer_node_real',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename_real],
            condition=UnlessCondition(use_sim_time)
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
