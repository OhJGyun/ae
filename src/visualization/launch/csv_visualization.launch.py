#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the config file
    config = os.path.join(
        get_package_share_directory('visualization'),
        'config',
        'visualization.yaml'
    )

    # Create the csv_publisher node
    csv_publisher_node = Node(
        package='visualization',
        executable='csv_publisher',
        name='csv_publisher',
        output='screen',
        parameters=[config],
        emulate_tty=True,
    )

    return LaunchDescription([
        csv_publisher_node
    ])
