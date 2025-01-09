import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the command-line arguments for the parameters
    package_description = "attach_shelf"
    package_dir = get_package_share_directory(package_description)
    rviz_config_dir = os.path.join(package_dir, 'rviz', 'rb1_rviz.rviz')
    approach_shelf_service_config = os.path.join(package_dir, 'config', 'approach_service_server.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('degrees', default_value='-90', description='Number of degrees for the rotation of the robot after stopping'),
        DeclareLaunchArgument('obstacle', default_value='0.3', description='Distance (in meters) to the obstacle at which the robot will stop'),
        DeclareLaunchArgument('final_approach', default_value='true', description='value of the request of the service, attach_to_shelf'),

        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            output='screen',
            parameters=[approach_shelf_service_config]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    ])
