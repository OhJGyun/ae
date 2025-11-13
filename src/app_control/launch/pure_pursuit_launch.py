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
        'pure_pursuit_params.yaml'
    )

    # Pure pursuit sim node (advanced version with PID control)
    # Parameters are loaded from yaml file by default
    # Can be overridden via command line: ros2 launch app_control pure_pursuit_launch.py max_speed:=7.0
    pure_pursuit_node = Node(
        package='app_control',
        executable='pure_pursuit_sim_node',
        name='pure_pursuit_sim_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        pure_pursuit_node,
    ])