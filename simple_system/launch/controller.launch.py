from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'simple_system'
    package_share_directory = get_package_share_directory(package_name)
    config_file_path = os.path.join(package_share_directory, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[config_file_path],
        )
    ])
