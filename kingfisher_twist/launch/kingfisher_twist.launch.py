import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('kingfisher_twist'), 'config', 'kingfisher_twist.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the kingfisher_twist node'),

        launch_ros.actions.Node(
            package='kingfisher_twist',
            executable='kingfisher_twist_node',
            name='kingfisher_twist',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'),
    ])
