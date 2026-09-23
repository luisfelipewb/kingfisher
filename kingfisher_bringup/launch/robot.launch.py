"""Top-level bringup for the real boat. Supersedes ros1/launch/robot.launch.

Base only. Lidar and SBG localization land once ros-jazzy-lms1xx and
ros-jazzy-sbg-driver are installed. ROS 1 gps.launch (u-blox) and
imu.launch (UM6) are not ported; the SBG unit supersedes both.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch = os.path.join(get_package_share_directory('kingfisher_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/arduino',
            description='Serial port for the Kingfisher MCU'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'static_tfs.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'base.launch.py')),
            launch_arguments={'port': LaunchConfiguration('port')}.items()),
    ])
