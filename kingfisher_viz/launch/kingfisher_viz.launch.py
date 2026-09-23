import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('kingfisher_viz'), 'config', 'kingfisher_viz.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the kingfisher_viz node'),

        # Name must match the node's own name in kingfisher_viz_node.py: the
        # node publishes on the private topic ~/marker, so the resolved topic
        # (/kingfisher_viz/marker) depends on the name set here.
        launch_ros.actions.Node(
            package='kingfisher_viz',
            executable='kingfisher_viz_node',
            name='kingfisher_viz',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'),
    ])
