#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Load parameters from yaml file
    config_file = os.path.join(
        get_package_share_directory('app_control'),
        'config',
        'stanley_params.yaml'
    )

    # Stanley node
    # Parameters are loaded from yaml file by default
    # Can be overridden via command line: ros2 launch app_control stanley_launch.py k_e:=0.5
    stanley_node = Node(
        package='app_control',
        executable='stanley_node',
        name='stanley_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        stanley_node,
    ])
