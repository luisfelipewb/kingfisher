"""
Publish a site's points as static TF frames, ``map -> <name>``.

The site file (``config/sites/<site>.yaml``) holds the datum, which is the origin of ``map``,
and the points as lat/lon. A point without altitude is at the datum's altitude (z = 0).
"""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from kingfisher_localization.geo import load_site


class GeoTfPublisher(Node):
    """Publish map -> <point> static transforms once."""

    def __init__(self):
        super().__init__('geo_tf_publisher')
        site_file = self.declare_parameter('site_file', '').value
        map_frame = self.declare_parameter('map_frame', 'map').value

        enu, points = load_site(site_file)
        transforms = []
        for p in points:
            x, y, z = enu.local_from_latlon(p['latitude'], p['longitude'], p.get('altitude'))
            yaw = math.radians(p.get('yaw_deg', 0.0))
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = map_frame
            t.child_frame_id = p['name']
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            transforms.append(t)
            self.get_logger().info(f"{p['name']}: x={x:.2f} y={y:.2f} z={z:.2f}")

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)


def main(args=None):
    """Spin the geo TF publisher."""
    rclpy.init(args=args)
    node = GeoTfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
