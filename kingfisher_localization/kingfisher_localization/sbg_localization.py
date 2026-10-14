"""
SBG Ellipse (driver in NED) -> the localization contract, the same topics as the sim (VRX).

* ``sensors/imu/imu/data`` (sensor_msgs/Imu, ``imu_frame``): orientation of imu_link in ENU,
  rates and accelerations in FLU, on every ``sbg/imu_data``;
* ``sensors/gps/gps/fix`` (sensor_msgs/NavSatFix, ``gps_frame``), on every ``sbg/gps_pos``;
* ``odom`` (nav_msgs/Odometry) and its TF ``odom -> base_frame``, on every ``sbg/ekf_nav``.
  Position is ENU about the site datum (``map`` = ``odom``), twist is in ``base_frame`` (FLU).

Without a valid position (indoors) odom is still published: the position is held at the last
valid one (the datum if none) and the velocity is 0, both with a large variance.

The imu_link position in base_frame comes from TF (the bringup's robot_state_publisher), looked
up once: odom waits for it, the IMU and GPS topics don't.
"""

import math

from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sbg_driver.msg import SbgEkfNav, SbgEkfQuat, SbgGpsPos, SbgImuData
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from kingfisher_localization.geo import load_site

# Quaternions (w, x, y, z): ENU <- NED, and FRD <- FLU (pi about x)
Q_ENU_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)

# SbgGpsPosStatus.type -> NavSatStatus.status
GPS_STATUS = {0: NavSatStatus.STATUS_NO_FIX, 4: NavSatStatus.STATUS_SBAS_FIX,
              6: NavSatStatus.STATUS_GBAS_FIX, 7: NavSatStatus.STATUS_GBAS_FIX}


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rotate(q, v):
    """Rotate v by the unit quaternion q."""
    u = q[1:]
    t = [2.0 * c for c in cross(u, v)]
    return tuple(vi + q[0] * ti + ci for vi, ti, ci in zip(v, t, cross(u, t)))


def flu(v):
    """FRD vector -> FLU tuple."""
    return (v.x, -v.y, -v.z)


class SbgLocalization(Node):
    """Convert the SBG driver's raw NED messages to the ENU/FLU contract topics."""

    def __init__(self):
        super().__init__('sbg_localization')
        self._enu, _ = load_site(self.declare_parameter('site_file', '').value)
        self._odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        self._imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self._gps_frame = self.declare_parameter('gps_frame', 'gps_link').value
        # m^2 and (m/s)^2, for a position or velocity the SBG doesn't have
        self._unknown_variance = self.declare_parameter('unknown_variance', 1e6).value

        self._q = None                 # imu_link in ENU, from the last ekf_quat
        self._q_variance = (0.0,) * 3  # roll, pitch, yaw
        self._omega = (0.0, 0.0, 0.0)  # rad/s, FLU, from the last imu_data
        self._position = (0.0, 0.0, 0.0)  # base_frame in odom, the last valid one
        # m, imu_frame in base_frame, from TF; the axes are assumed aligned (as in the URDF)
        self._lever = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._imu_pub = self.create_publisher(Imu, 'sensors/imu/imu/data', 10)
        self._fix_pub = self.create_publisher(NavSatFix, 'sensors/gps/gps/fix', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(SbgEkfQuat, 'sbg/ekf_quat', self._on_quat, 10)
        self.create_subscription(SbgImuData, 'sbg/imu_data', self._on_imu, 10)
        self.create_subscription(SbgGpsPos, 'sbg/gps_pos', self._on_gps, 10)
        self.create_subscription(SbgEkfNav, 'sbg/ekf_nav', self._on_nav, 10)

    def _on_quat(self, msg):
        q = msg.quaternion
        self._q = qmul(qmul(Q_ENU_NED, (q.w, q.x, q.y, q.z)), Q_FRD_FLU)
        self._q_variance = tuple(a * a for a in (msg.accuracy.x, msg.accuracy.y, msg.accuracy.z))

    def _on_imu(self, msg):
        self._omega = flu(msg.gyro)
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self._imu_frame
        if self._q is None:
            imu.orientation_covariance[0] = -1.0
        else:
            imu.orientation = Quaternion(w=self._q[0], x=self._q[1], y=self._q[2], z=self._q[3])
            imu.orientation_covariance[0::4] = self._q_variance
        imu.angular_velocity = Vector3(x=self._omega[0], y=self._omega[1], z=self._omega[2])
        a = flu(msg.accel)
        imu.linear_acceleration = Vector3(x=a[0], y=a[1], z=a[2])
        self._imu_pub.publish(imu)

    def _on_gps(self, msg):
        fix = NavSatFix()
        fix.header.stamp = msg.header.stamp
        fix.header.frame_id = self._gps_frame
        fix.status.service = NavSatStatus.SERVICE_GPS
        if msg.status.status != 0:  # not SOL_COMPUTED
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix.status.status = GPS_STATUS.get(msg.status.type, NavSatStatus.STATUS_FIX)
        fix.latitude, fix.longitude = msg.latitude, msg.longitude
        # Above mean sea level, like the site datum (and the sim's navsat at the water)
        fix.altitude = msg.altitude
        acc = msg.position_accuracy  # NED: x north, y east
        fix.position_covariance[0::4] = (acc.y ** 2, acc.x ** 2, acc.z ** 2)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)

    def _lookup_lever(self):
        """Return imu_frame's position in base_frame, once TF has it, else None."""
        if self._lever is None:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base_frame, self._imu_frame, Time()).transform.translation
            except TransformException as ex:
                self.get_logger().warn(
                    f'no odom until TF has {self._imu_frame} in {self._base_frame}: {ex}',
                    throttle_duration_sec=5.0)
                return None
            self._lever = (t.x, t.y, t.z)
            self.get_logger().info(f'{self._imu_frame} in {self._base_frame}: {self._lever}')
            # Static: no need to keep listening to /tf
            self._tf_listener.unregister()
        return self._lever

    def _on_nav(self, msg):
        if self._q is None or self._lookup_lever() is None:
            return
        q, status = self._q, msg.status
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        big = self._unknown_variance
        if status.position_valid:
            imu = self._enu.local_from_latlon(msg.latitude, msg.longitude, msg.altitude)
            self._position = tuple(p - r for p, r in zip(imu, rotate(q, self._lever)))
            acc = msg.position_accuracy  # NED: x north, y east
            position_variance = (acc.y ** 2, acc.x ** 2, acc.z ** 2)
        else:
            position_variance = (big,) * 3
        p = odom.pose.pose.position
        p.x, p.y, p.z = self._position
        odom.pose.pose.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        odom.pose.covariance[0::7] = position_variance + self._q_variance

        if status.velocity_valid:
            v = msg.velocity  # NED
            # imu_link velocity, in the body; base_link = imu_link - omega x lever
            v_imu = rotate((q[0], -q[1], -q[2], -q[3]), (v.y, v.x, -v.z))
            v_base = tuple(a - b for a, b in zip(v_imu, cross(self._omega, self._lever)))
            acc = msg.velocity_accuracy
            h = max(acc.x, acc.y) ** 2
            velocity_variance = (h, h, acc.z ** 2)
        else:
            v_base = (0.0, 0.0, 0.0)
            velocity_variance = (big,) * 3
        t = odom.twist.twist
        t.linear = Vector3(x=v_base[0], y=v_base[1], z=v_base[2])
        t.angular = Vector3(x=self._omega[0], y=self._omega[1], z=self._omega[2])
        odom.twist.covariance[0::7] = velocity_variance + (0.0,) * 3
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self._base_frame
        tf.transform.translation = Vector3(x=p.x, y=p.y, z=p.z)
        tf.transform.rotation = odom.pose.pose.orientation
        self._tf.sendTransform(tf)


def main(args=None):
    """Spin the SBG localization node."""
    rclpy.init(args=args)
    node = SbgLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
