"""joy_teleop, next to the robot, plus joy_node unless joy:=false.

With the stick on another machine, run joy.launch.py there and this with
joy:=false here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    pkg = get_package_share_directory('kingfisher_teleop')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='Robot namespace; namespace:=/ for the root namespace'),
        DeclareLaunchArgument(
            'joy', default_value='true',
            description='Also start joy_node; false when the stick is on another machine'),
        DeclareLaunchArgument(
            'teleop_config', default_value=os.path.join(pkg, 'config', 'teleop.yaml'),
            description='Parameter file for joy_teleop'),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'joy.launch.py')),
                launch_arguments={'namespace': namespace}.items()),
        ], condition=IfCondition(LaunchConfiguration('joy'))),

        launch_ros.actions.Node(
            package='joy_teleop', executable='joy_teleop',
            name='joy_teleop',
            namespace=namespace,
            parameters=[LaunchConfiguration('teleop_config')],
            output='screen'),
    ])
