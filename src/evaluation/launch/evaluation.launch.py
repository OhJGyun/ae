#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch argument for controller name
    controller_name_arg = DeclareLaunchArgument(
        'controller_name',
        default_value='unknown',
        description='Name of the controller being evaluated (e.g., pure_pursuit, mpc, disparity, map_controller)'
    )

    # ✅ Declare launch argument for obstacle map evaluation
    is_obstacle_map_arg = DeclareLaunchArgument(
        'is_obstacle_map',
        default_value='false',
        description='Set to true when evaluating on obstacle map (saves to separate CSV)'
    )

    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('evaluation'),
        'config',
        'sections.yaml'
    )

    # Evaluation node
    evaluation_node = Node(
        package='evaluation',
        executable='evaluation_node',
        name='evaluation',
        output='screen',
        parameters=[
            config_file,
            {
                'controller_name': LaunchConfiguration('controller_name'),
                'is_obstacle_map': LaunchConfiguration('is_obstacle_map')
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        controller_name_arg,
        is_obstacle_map_arg,
        evaluation_node,
    ])
