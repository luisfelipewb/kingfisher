"""Top-level bringup for the real boat.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

import launch_ros.actions
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('kingfisher_bringup')
    bringup_launch = os.path.join(bringup_share, 'launch')
    config = os.path.join(bringup_share, 'config', 'kingfisher.yaml')
    urdf = os.path.join(
        get_package_share_directory('kingfisher_description'), 'urdf', 'kingfisher.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        # TF from the URDF. No namespace or frame_prefix: frames on the boat
        # are plain (base_link, imu_link, ...).
        launch_ros.actions.Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),

        # Merges the measured joints (source_list) into /joint_states at a
        # fixed rate; joints without a source, the propellers, are sent as 0.
        launch_ros.actions.Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[config],
            output='screen'),

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
                    parameters=[config],
                    remappings=[('joint_states', 'sail/joint_states')]),

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
