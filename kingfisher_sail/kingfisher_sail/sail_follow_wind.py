"""
Keeps the sail feathered in the apparent wind.

Turns ``sensors/anemometer/wind`` (geometry_msgs/Vector3Stamped in base_link, FLU, m/s,
pointing where the wind goes) into ``sail/cmd_position`` for sail_controller:

* the target is the direction the wind comes from, turned ``angle_of_attack_deg`` toward the
  bow, so lift pushes forward from either side; at 0 the sail's nose points into the wind;
* below ``min_wind_speed`` nothing is sent, and the sail holds;
* a target within ``deadband`` of the measured ``sail_joint`` angle is skipped, so the
  follower also takes the sail back after a jog or a preset.

Calibration, wrapping, the velocity limit and the teleop override are sail_controller's.
"""

from math import atan2, copysign, hypot, radians

from geometry_msgs.msg import Vector3Stamped
from kingfisher_sail.sail_controller import wrap
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def feather_angle(vx, vy, angle_of_attack=0.0):
    """Return the sail angle for the wind (vx, vy) in base_link, pointing where it goes."""
    wind = atan2(-vy, -vx)
    return wrap(wind - copysign(angle_of_attack, wind))


class SailFollowWind(Node):
    """Point the sail into the apparent wind."""

    def __init__(self):
        super().__init__('sail_follow_wind')

        # deg, the sail angle from the feathered position, toward the bow
        # (alpha, as in the sim's aero model)
        self._angle_of_attack = radians(self.declare_parameter('angle_of_attack_deg', 0.0).value)
        # m/s, below this the wind direction is ignored and the sail holds
        self._min_wind_speed = self.declare_parameter('min_wind_speed', 0.5).value
        # rad, a target closer than this to the measured sail angle is skipped
        self._deadband = self.declare_parameter('deadband', 0.05).value
        self._joint = self.declare_parameter('joint', 'sail_joint').value

        self._angle = None

        self._position_pub = self.create_publisher(Float64, 'sail/cmd_position', 10)
        self.create_subscription(JointState, 'joint_states', self._joint_states_callback, 10)
        self.create_subscription(
            Vector3Stamped, 'sensors/anemometer/wind', self._wind_callback, 10)

    def _joint_states_callback(self, msg):
        if self._joint in msg.name:
            self._angle = msg.position[msg.name.index(self._joint)]

    def _wind_callback(self, msg):
        vx, vy = msg.vector.x, msg.vector.y
        speed = hypot(vx, vy)
        if speed < self._min_wind_speed:
            self.get_logger().info(
                f'Wind {speed:.2f} m/s below min_wind_speed: holding',
                throttle_duration_sec=10.0)
            return
        target = feather_angle(vx, vy, self._angle_of_attack)
        if self._angle is not None and abs(wrap(target - self._angle)) < self._deadband:
            return
        self._position_pub.publish(Float64(data=target))


def main(args=None):
    """Spin the wind follower."""
    rclpy.init(args=args)
    node = SailFollowWind()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
