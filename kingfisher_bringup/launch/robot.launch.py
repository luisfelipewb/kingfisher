"""Top-level bringup for the real ASV.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

import launch_ros.actions
from launch_ros.actions import PushRosNamespace
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('kingfisher_bringup')
    bringup_launch = os.path.join(bringup_share, 'launch')
    config = os.path.join(bringup_share, 'config', 'kingfisher.yaml')
    urdf = os.path.join(
        get_package_share_directory('kingfisher_description'), 'urdf', 'kingfisher.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)
    namespace = LaunchConfiguration('namespace')
    frame_prefix = PythonExpression(["'", LaunchConfiguration('frame_prefix'), "'.lstrip('/')"])

    nodes = [
        # TF from the URDF. The URDF links are plain (base_link, imu_link, ...)
        # names are not prefixed.
        launch_ros.actions.Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': ParameterValue(frame_prefix, value_type=str),
            }],
            output='screen'),

        # Merges the measured joints (source_list) into joint_states at a
        # fixed rate; joints without a source, the propellers, are sent as 0.
        launch_ros.actions.Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            parameters=[config],
            output='screen'),

        # Phidgets drivers on the VINT hub. They are components, so they
        # share one container process.
        launch_ros.actions.ComposableNodeContainer(
            name='phidget_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[
                # sail magnet switch on digital_input00
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
                    parameters=[config, {'frame_id': ParameterValue(
                        [frame_prefix, 'sail_motor_link'], value_type=str)}],
                    remappings=[('joint_states', 'sail/joint_states')]),

                # sail stepper
                launch_ros.descriptions.ComposableNode(
                    package='phidgets_stepper',
                    plugin='phidgets::StepperRosI',
                    name='phidgets_stepper',
                    parameters=[config]),
            ],
            output='screen'),

        # Sail calibration. Idle until sail_calibration/calibrate is called.
        launch_ros.actions.Node(
            package='kingfisher_sail', executable='kingfisher_sail',
            name='sail_calibration',
            parameters=[config],
            output='screen'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='kingfisher',
            description='ROS namespace for the boat nodes and topics'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='kingfisher/',
            description='prefix for the TF frames (robot_state_publisher and node frame_ids)'),

        GroupAction([PushRosNamespace(namespace), *nodes]),

        # Outside the group: base.launch.py pushes the namespace itself, and
        # inside the group it would stack (/kingfisher/kingfisher).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_launch, 'base.launch.py')),
            launch_arguments={
                'namespace': namespace,
                'frame_prefix': LaunchConfiguration('frame_prefix'),
            }.items()),
    ])
