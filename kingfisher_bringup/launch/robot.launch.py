"""Top-level bringup for the real boat.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    bringup_share = get_package_share_directory('kingfisher_bringup')
    bringup_launch = os.path.join(bringup_share, 'launch')
    config = os.path.join(bringup_share, 'config', 'kingfisher.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'static_tfs.launch.py'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'base.launch.py'))),

        # Phidgets drivers on the VINT hub. They are components, so they
        # share one container process.
        launch_ros.actions.ComposableNodeContainer(
            name='phidget_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[
                # sail magnet switch on /digital_input00
                launch_ros.descriptions.ComposableNode(
                    package='phidgets_digital_inputs',
                    plugin='phidgets::DigitalInputsRosI',
                    name='phidgets_digital_inputs',
                    parameters=[config]),

                # sail encoder
                launch_ros.descriptions.ComposableNode(
                    package='phidgets_high_speed_encoder',
                    plugin='phidgets::HighSpeedEncoderRosI',
                    name='phidgets_high_speed_encoder',
                    parameters=[config]),

                # sail stepper
                launch_ros.descriptions.ComposableNode(
                    package='phidgets_stepper',
                    plugin='phidgets::StepperRosI',
                    name='phidgets_stepper',
                    parameters=[config]),
            ],
            output='screen'),

        # Sail calibration. Idle until /sail_calibration/calibrate is called.
        launch_ros.actions.Node(
            package='kingfisher_sail', executable='kingfisher_sail',
            name='sail_calibration',
            parameters=[config],
            output='screen'),
    ])
