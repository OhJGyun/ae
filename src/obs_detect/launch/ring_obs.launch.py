#!/usr/bin/env python3
"""
Launch file for ring obstacle detection with boundary visualization (Simulation version)
Launches both obstacle detection and boundary visualization nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('obs_detect')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'obs_detect_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Default to true for simulation
        description='Use simulation time if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ring obstacle detection node
    ring_obs_node = Node(
        package='obs_detect',
        executable='ring_obs_node',
        name='ring_obs_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Add remappings if needed for simulation
        ]
    )

    # Boundary visualization node
    boundary_viz_node = Node(
        package='obs_detect',
        executable='boundary_viz_node',
        name='boundary_viz_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ring_obs_node,
        boundary_viz_node,
    ])
