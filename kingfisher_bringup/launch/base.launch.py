"""The nodes needed to drive the boat. Supersedes ROS 1 base.launch.

Self-contained: nodes are declared here against config/kingfisher.yaml
rather than including each package's own launch file. Those still exist
in their packages for debugging.

Runs under the `namespace` launch argument (default kingfisher) and stamps
frames with `frame_prefix` (default kingfisher/). robot.launch.py passes
both through.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression

import launch_ros.actions
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('kingfisher_bringup'), 'config', 'kingfisher.yaml')
    # Leading '/' stripped, as in robot.launch.py: frame_prefix:=/ means no prefix.
    frame_prefix = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'.lstrip('/')"])

    nodes = [
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

        # thrust and wind arrows for RViz
        launch_ros.actions.Node(
            package='kingfisher_viz', executable='kingfisher_viz_node',
            name='kingfisher_viz',
            parameters=[config, {
                param: ParameterValue([frame_prefix, frame], value_type=str)
                for param, frame in (
                    ('base_frame', 'base_link'),
                    ('left_thruster_frame', 'left_thruster_link'),
                    ('right_thruster_frame', 'right_thruster_link'))}],
            output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace for the boat nodes and topics'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='kingfisher/',
            description='prefix for the TF frames the boat nodes stamp'),
        GroupAction([PushRosNamespace(LaunchConfiguration('namespace')), *nodes]),
    ])
