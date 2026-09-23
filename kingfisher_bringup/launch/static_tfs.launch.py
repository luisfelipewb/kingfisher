"""Calibrated boat geometry as static transforms.

From ros1/launch/static_tfs.launch. Jazzy's static_transform_publisher
takes named flags only, and tf2 rejects a leading slash on frame ids.

Replaced by robot_state_publisher in Stage B; these numbers are the
ground truth the URDF has to match.
"""

from launch import LaunchDescription

import launch_ros.actions

# name, parent, child, (x, y, z), (roll, pitch, yaw)
STATIC_TFS = [
    ('sbg',            'base_link', 'sbg',               (-0.1,   0.0,     0.06),  (0.0,     0.0, 0.0)),
    ('hazcam',         'base_link', 'camera_link',       (0.33,   0.0,     0.04),  (0.0,     0.0, 0.0)),
    ('lidar',          'base_link', 'laser',             (0.04,   0.0,     0.16),  (3.14159, 0.0, 0.0)),
    ('anemometer',     'base_link', 'anemometer_link',   (-0.54,  0.0,     0.32),  (3.14159, 0.0, 0.0)),
    ('sail_encoder',   'base_link', 'sail_encoder_link', (0.30,   0.0,     0.05),  (0.0,     0.0, 3.14159)),
    ('gps_a1',         'sbg',       'gps_a1',            (-0.04, -0.49,    0.29),  (0.0,     0.0, 0.0)),
    ('gps_a2',         'sbg',       'gps_a2',            (-0.04,  0.49,    0.29),  (0.0,     0.0, 0.0)),
    # ROS 1 named these gps_tl/gps_tr; they are the thrusters.
    ('thruster_left',  'base_link', 'thruster_left',     (-0.53,  0.3776, -0.16),  (0.0,     0.0, 0.0)),
    ('thruster_right', 'base_link', 'thruster_right',    (-0.53, -0.3776, -0.16),  (0.0,     0.0, 0.0)),
]


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{name}_static_broadcaster',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', str(z),
                '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                '--frame-id', parent, '--child-frame-id', child,
            ],
            output='screen')
        for name, parent, child, (x, y, z), (roll, pitch, yaw) in STATIC_TFS
    ])
