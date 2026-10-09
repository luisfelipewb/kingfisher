"""game_controller_node alone, for the machine the joystick is plugged into."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('kingfisher_teleop'), 'config', 'joy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='Robot namespace; namespace:=/ for the root namespace'),

        launch_ros.actions.Node(
            package='joy', executable='game_controller_node',
            name='game_controller_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[config],
            output='screen'),
    ])
