#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('unicore_um982_driver')
    config_file = os.path.join(pkg_dir, 'config', 'unicore_driver_params.yaml')
    
    return LaunchDescription([
        Node(
            package='unicore_um982_driver',
            executable='unicore_um982_driver_node',
            name='unicore_um982_driver',
            parameters=[config_file],
            output='screen'
        )
    ])
