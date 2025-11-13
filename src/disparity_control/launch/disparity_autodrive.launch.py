import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the parameters file
    config = os.path.join(
        get_package_share_directory('disparity_control'),
        'config',
        'disparity_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='disparity_control',
            executable='disparity_autodrive_node',
            name='disparity_autodrive',
            output='screen',
            parameters=[config]
        )
    ])
