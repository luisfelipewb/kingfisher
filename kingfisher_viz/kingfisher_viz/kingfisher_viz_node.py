#!/usr/bin/python3
"""RViz arrows for the forces on the Kingfisher: thrusters, true and apparent wind, sail."""

import math

from geometry_msgs.msg import Point, Vector3Stamped, WrenchStamped
from kingfisher_msgs.msg import Drive
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Float32, Header
from visualization_msgs.msg import Marker, MarkerArray

RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=0.8, b=1.0, a=1.0)
BLUE = ColorRGBA(r=0.1, g=0.2, b=1.0, a=1.0)
ORANGE = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
MAGENTA = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)

# RViz warns "Scale of 0 in one of x/y/z" on a zero scale component, and an
# idle boat publishes cmd_drive 0.0 continuously. Keep the arrow length off
# zero; the sign is passed through untouched.
MIN_ARROW_LENGTH = 1e-3


def arrow_length(thrust):
    """Thrust arrow length, kept off zero."""
    if abs(thrust) < MIN_ARROW_LENGTH:
        return MIN_ARROW_LENGTH
    return thrust


def rotate_into(q, v):
    """Express the vector v in the frame whose orientation is the quaternion q."""
    x, y, z, w = q.x, q.y, q.z, q.w
    # Transpose of the rotation matrix of q
    return (
        (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y + z * w) * v[1] + 2 * (x * z - y * w) * v[2],
        2 * (x * y - z * w) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z + x * w) * v[2],
        2 * (x * z + y * w) * v[0] + 2 * (y * z - x * w) * v[1] + (1 - 2 * (x * x + y * y)) * v[2],
    )


class KingfisherViz(Node):
    """Publishes one MarkerArray per input message, holding that input's arrows."""

    def __init__(self):
        super().__init__('kingfisher_viz')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.thruster_frames = (
            self.declare_parameter('left_thruster_frame', 'left_thruster_link').value,
            self.declare_parameter('right_thruster_frame', 'right_thruster_link').value)
        # m per m/s, and m per N
        self.wind_scale = self.declare_parameter('wind_scale', 0.1).value
        self.force_scale = self.declare_parameter('force_scale', 0.25).value
        self.true_wind_origin = list(
            self.declare_parameter('true_wind_origin', [0.0, 0.0, 1.2]).value)

        # Cached for the true wind
        self.wind_direction = None
        self.orientation = None

        self.marker_pub = self.create_publisher(MarkerArray, '~/marker', 1)
        # (topic, type, QoS, marker builder)
        contributions = [
            ('cmd_drive', Drive, 1, self.drive_markers),
            ('sensors/anemometer/wind', Vector3Stamped, qos_profile_sensor_data,
             self.apparent_wind_markers),
            ('/vrx/debug/wind/speed', Float32, qos_profile_sensor_data, self.true_wind_markers),
            ('sail/wrench', WrenchStamped, qos_profile_sensor_data, self.sail_markers),
        ]
        for topic, msg_type, qos, build in contributions:
            self.create_subscription(
                msg_type, topic, lambda msg, build=build: self.publish(build(msg)), qos)
        self.create_subscription(
            Float32, '/vrx/debug/wind/direction', self.on_wind_direction, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, 'sensors/position/ground_truth_odometry', self.on_odometry,
            qos_profile_sensor_data)

    def on_wind_direction(self, msg):
        self.wind_direction = msg.data

    def on_odometry(self, msg):
        self.orientation = msg.pose.pose.orientation

    def publish(self, markers):
        if markers:
            self.marker_pub.publish(MarkerArray(markers=markers))

    def drive_markers(self, msg):
        stamp = self.get_clock().now().to_msg()
        left, right = self.thruster_frames
        return [self.thrust_arrow(Header(stamp=stamp, frame_id=left), 0, msg.left, RED),
                self.thrust_arrow(Header(stamp=stamp, frame_id=right), 1, msg.right, GREEN)]

    def apparent_wind_markers(self, msg):
        v = msg.vector
        s = self.wind_scale
        end = (s * v.x, s * v.y, s * v.z)
        return [self.arrow(msg.header, 'kf_wind', 0, (0.0, 0.0, 0.0), end, CYAN)]

    def true_wind_markers(self, msg):
        # Where the wind goes, ENU degrees; rotated into base_frame by the ground truth
        if self.wind_direction is None or self.orientation is None:
            return []
        theta = math.radians(self.wind_direction)
        world = (msg.data * math.cos(theta), msg.data * math.sin(theta), 0.0)
        wind = rotate_into(self.orientation, world)
        start = self.true_wind_origin
        end = [start[i] + self.wind_scale * wind[i] for i in range(3)]
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.base_frame)
        return [self.arrow(header, 'kf_wind', 1, start, end, BLUE)]

    def sail_markers(self, msg):
        f = msg.wrench.force
        s = self.force_scale
        origin = (0.0, 0.0, 0.0)
        return [self.arrow(msg.header, 'kf_sail', 0, origin, (s * f.x, 0.0, 0.0), ORANGE),
                self.arrow(msg.header, 'kf_sail', 1, origin, (0.0, s * f.y, 0.0), MAGENTA)]

    def thrust_arrow(self, header, marker_id, thrust, color):
        """Arrow along the thruster's x, from its origin, as long as the command."""
        m = Marker(header=header, ns='kf_drive', id=marker_id, type=Marker.ARROW,
                   action=Marker.ADD, color=color)
        m.pose.orientation.w = 1.0
        m.scale.x = arrow_length(thrust)
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.lifetime = Duration(seconds=3.0).to_msg()
        return m

    def arrow(self, header, ns, marker_id, start, end, color):
        """Arrow from start to end, in header's frame; vanishes 1 s after its source stops."""
        if math.dist(start, end) < MIN_ARROW_LENGTH:
            end = (start[0] + MIN_ARROW_LENGTH, start[1], start[2])
        # frame_locked: RViz uses the latest TF, not the stamp's. A sensor stamp
        # can be ahead of the TF of a moving frame (sail_link), and RViz drops
        # markers it cannot transform.
        m = Marker(header=header, ns=ns, id=marker_id, type=Marker.ARROW,
                   action=Marker.ADD, color=color, frame_locked=True)
        m.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                    Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
        m.pose.orientation.w = 1.0
        # Shaft diameter, head diameter, head length
        m.scale.x = 0.03
        m.scale.y = 0.06
        m.scale.z = 0.08
        m.lifetime = Duration(seconds=1.0).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = KingfisherViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
