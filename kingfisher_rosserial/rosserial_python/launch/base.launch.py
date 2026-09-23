import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rosserial_python'), 'config', 'serial_node.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the rosserial bridge'),
        DeclareLaunchArgument(
            'port', default_value='/dev/arduino',
            description='Serial port for the Kingfisher MCU'),

        # respawn matches what the ROS 1 kingfisher_bringup/launch/base.launch
        # did; it was lost in the ROS 2 port.
        #
        # It matters more here than it used to. serial_node.py still has the
        # unguarded crash paths listed in KINGFISHER-PLAN.md 5.4 -- notably it
        # dies on the first log packet the firmware sends -- and it opens the
        # serial port in __init__, so it also fails outright if the MCU has not
        # enumerated yet at boot. respawn turns both into a retry rather than
        # the end of the run. It is a guard, not a fix.
        launch_ros.actions.Node(
            package='rosserial_python',
            executable='serial_node',
            name='kingfisher_serial',
            parameters=[
                LaunchConfiguration('params_file'),
                {'port': LaunchConfiguration('port')},
            ],
            respawn=True,
            respawn_delay=2.0,
            output='screen'),
    ])
