import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('wifi_monitor'), 'config', 'wifi_monitor.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the wifi_monitor node'),

        # respawn matches what the ROS 1 kingfisher_bringup/launch/base.launch
        # did; it was lost in the ROS 2 port.
        launch_ros.actions.Node(
            package='wifi_monitor',
            executable='wifi_monitor_node',
            name='wifi_monitor',
            parameters=[LaunchConfiguration('params_file')],
            respawn=True,
            respawn_delay=2.0,
            output='screen'),
    ])
