"""Localization for the real boat: the SBG driver, odom/IMU/GPS, map -> odom and the site frames.

Runs on its own, next to kingfisher_bringup (which publishes base_link -> imu_link etc.).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

import launch_ros.actions
from launch_ros.actions import PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('kingfisher_localization')
    site_file = PathJoinSubstitution(
        [share, 'config', 'sites', [LaunchConfiguration('site'), '.yaml']])
    frame_prefix = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'.lstrip('/')"])

    def frame(name):
        return ParameterValue([frame_prefix, name], value_type=str)

    nodes = [
        # Raw sbg/* in NED, stamped imu_link_ned
        launch_ros.actions.Node(
            package='sbg_driver', executable='sbg_device',
            parameters=[os.path.join(share, 'config', 'sbg.yaml'), {
                'uartConf.portName': LaunchConfiguration('port'),
                'output.frame_id': frame('imu_link_ned'),
            }],
            output='screen'),

        # sbg/* -> sensors/imu/imu/data, sensors/gps/gps/fix, odom and odom -> base_link
        launch_ros.actions.Node(
            package='kingfisher_localization', executable='sbg_localization',
            parameters=[{
                'site_file': site_file,
                'base_frame': frame('base_link'),
                'imu_frame': frame('imu_link'),
                'gps_frame': frame('gps_link'),
            }],
            output='screen'),

        # map -> <site point>
        launch_ros.actions.Node(
            package='kingfisher_localization', executable='geo_tf_publisher',
            parameters=[{'site_file': site_file}],
            output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace, the same as the bringup'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='kingfisher/',
            description='prefix for the robot TF frames, the same as the bringup'),
        DeclareLaunchArgument(
            'port', default_value='/dev/sbg',
            description='SBG serial port (udev symlink)'),
        DeclareLaunchArgument(
            'site', default_value='lac_symphonie',
            description='config/sites/<site>.yaml: the map datum and the site frames'),

        GroupAction([PushRosNamespace(LaunchConfiguration('namespace')), *nodes]),

        # map = odom: the SBG position is absolute, so there is no drift to absorb
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            output='screen'),
    ])
