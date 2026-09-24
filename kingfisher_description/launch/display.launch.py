# Copyright 2026 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Show the Kingfisher description in RViz, without a simulator."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('kingfisher_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'kingfisher.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='kingfisher',
             parameters=[{'robot_description': robot_description,
                          'frame_prefix': 'kingfisher/'}]),
        # Zero angles for the propeller joints, which otherwise get no TF
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace='kingfisher'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
