"""
Sail command interface, the same in sim and on the boat.

Turns ``sail/cmd_position`` and ``sail/cmd_velocity`` (std_msgs/Float64, rad and rad/s,
+ counter-clockwise seen from above) into ``phidgets_stepper/command``:

* a position is wrapped to [-pi, pi] and reached the short way from the measured
  ``sail_joint`` angle, with STEP at ``max_velocity``;
* a non-zero velocity runs the stepper (RUN), clamped to ``max_velocity``. A 0, or no
  velocity for ``velocity_timeout``, sends RUN at 0: the stepper ramps down and holds.
  STOP is not used, since the driver disengages the motor on every mode change;
* velocity wins: positions are ignored until ``position_holdoff`` after the last non-zero
  velocity;
* nothing is sent while ``sail/calibrated`` is false or has not arrived.
"""

from math import remainder, tau

from phidgets_msgs.msg import StepperCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


def wrap(angle):
    """Return angle in [-pi, pi]."""
    return remainder(angle, tau)


class SailController(Node):
    """Forward sail position and velocity commands to the stepper."""

    def __init__(self):
        super().__init__('sail_controller')

        # rad/s, the STEP velocity limit and the RUN clamp
        self._max_velocity = self.declare_parameter('max_velocity', 2.0).value
        # s without cmd_velocity before a jog stops
        self._velocity_timeout = self.declare_parameter('velocity_timeout', 0.25).value
        # s after the last non-zero cmd_velocity during which cmd_position is ignored
        self._position_holdoff = self.declare_parameter('position_holdoff', 1.0).value
        self._joint = self.declare_parameter('joint', 'sail_joint').value

        self._calibrated = False
        self._angle = None
        # The RUN velocity last sent; non-zero while jogging
        self._velocity = 0.0
        self._last_velocity_msg = None
        self._last_nonzero_velocity = None

        self._command_pub = self.create_publisher(StepperCommand, 'phidgets_stepper/command', 10)
        self.create_subscription(
            Bool, 'sail/calibrated', self._calibrated_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, 'joint_states', self._joint_states_callback, 10)
        self.create_subscription(Float64, 'sail/cmd_position', self._position_callback, 10)
        self.create_subscription(Float64, 'sail/cmd_velocity', self._velocity_callback, 10)
        self.create_timer(0.05, self._timeout_check)

    def _calibrated_callback(self, msg):
        if msg.data != self._calibrated:
            self.get_logger().info(
                'Calibrated' if msg.data else 'Not calibrated: commands ignored')
        self._calibrated = msg.data
        # Calibration owns the stepper now; forget the jog without commanding it
        if not msg.data:
            self._velocity = 0.0

    def _joint_states_callback(self, msg):
        if self._joint in msg.name:
            self._angle = msg.position[msg.name.index(self._joint)]

    def _position_callback(self, msg):
        if not self._ready('cmd_position'):
            return
        now = self.get_clock().now()
        if self._last_nonzero_velocity is not None and \
                self._seconds(now - self._last_nonzero_velocity) < self._position_holdoff:
            self.get_logger().warn(
                'cmd_position ignored: cmd_velocity has priority', throttle_duration_sec=1.0)
            return
        target = self._angle + wrap(msg.data - self._angle)
        self._send(StepperCommand.CONTROL_MODE_STEP, target=target, velocity=self._max_velocity)

    def _velocity_callback(self, msg):
        if not self._ready('cmd_velocity'):
            return
        now = self.get_clock().now()
        self._last_velocity_msg = now
        velocity = max(-self._max_velocity, min(self._max_velocity, msg.data))
        if velocity != 0.0:
            self._last_nonzero_velocity = now
        if velocity != self._velocity:
            self._run(velocity)

    def _timeout_check(self):
        if self._velocity != 0.0 and \
                self._seconds(self.get_clock().now() - self._last_velocity_msg) > \
                self._velocity_timeout:
            self.get_logger().info('No cmd_velocity: stopping')
            self._run(0.0)

    def _ready(self, topic):
        if not self._calibrated:
            reason = 'sail not calibrated'
        elif self._angle is None:
            reason = f'no {self._joint} on joint_states'
        else:
            return True
        self.get_logger().warn(f'{topic} ignored: {reason}', throttle_duration_sec=2.0)
        return False

    def _run(self, velocity):
        self._velocity = velocity
        self._send(StepperCommand.CONTROL_MODE_RUN, velocity=velocity)

    def _send(self, mode, target=0.0, velocity=0.0):
        cmd = StepperCommand()
        cmd.mode = mode
        cmd.target = float(target)
        cmd.velocity = float(velocity)
        self._command_pub.publish(cmd)

    @staticmethod
    def _seconds(duration):
        return duration.nanoseconds * 1e-9


def main(args=None):
    """Spin the sail controller."""
    rclpy.init(args=args)
    node = SailController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
