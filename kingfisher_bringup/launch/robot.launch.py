"""Top-level bringup for the real boat. Supersedes ros1/launch/robot.launch.

Base plus sail calibration. Lidar and SBG localization land once
ros-jazzy-lms1xx and ros-jazzy-sbg-driver are installed. ROS 1 gps.launch
(u-blox) and imu.launch (UM6) are not ported; the SBG unit supersedes both.

The Phidgets drivers the sail node talks to (stepper, high-speed encoder,
digital inputs) are not launched here yet; they still come from
sawasp/phidgets_launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import launch_ros.actions


def generate_launch_description():
    bringup_share = get_package_share_directory('kingfisher_bringup')
    bringup_launch = os.path.join(bringup_share, 'launch')
    config = os.path.join(bringup_share, 'config', 'kingfisher.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/arduino',
            description='Serial port for the Kingfisher MCU'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'static_tfs.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'base.launch.py')),
            launch_arguments={'port': LaunchConfiguration('port')}.items()),

        # Sail calibration. Idle until /sail_calibration/calibrate is called;
        # needs the Phidgets drivers from sawasp/phidgets_launch to be up.
        launch_ros.actions.Node(
            package='kingfisher_sail', executable='kingfisher_sail',
            name='sail_calibration',
            parameters=[config],
            output='screen'),
    ])
