"""The nodes needed to drive the boat. Supersedes ROS 1 base.launch.

Self-contained: nodes are declared here against config/kingfisher.yaml
rather than including each package's own launch file. Those still exist
in their packages for debugging.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')

    return LaunchDescription([
        # MCU bridge. Topic names here come from the firmware (plan 5.1).
        # respawn was on the ROS 1 node and matters more now: serial_node.py
        # still has the unguarded crashes in plan 5.4 and opens the port in
        # __init__. A guard, not a fix.
        launch_ros.actions.Node(
            package='rosserial_python', executable='serial_node',
            name='kingfisher_serial',
            parameters=[config],
            respawn=True, respawn_delay=2.0,
            output='screen'),

        # cmd_vel -> cmd_drive
        launch_ros.actions.Node(
            package='kingfisher_twist', executable='kingfisher_twist_node',
            name='kingfisher_twist',
            parameters=[config],
            output='screen'),

        # link liveness
        launch_ros.actions.Node(
            package='wifi_monitor', executable='wifi_monitor_node',
            name='wifi_monitor',
            parameters=[config],
            respawn=True, respawn_delay=2.0,
            output='screen'),

        # thrust arrows for RViz
        launch_ros.actions.Node(
            package='kingfisher_viz', executable='kingfisher_viz_node',
            name='kingfisher_viz',
            parameters=[config],
            output='screen'),
    ])
