"""
Sail calibration node.

Exposes a ``sail/calibrate`` service (std_srvs/Trigger) that locates the magnet switch, moves
the sail to the true zero and zeroes both the stepper and the encoder. ``sail/calibrated``
(std_msgs/Bool, latched) is false from startup and during a calibration, and true after a
successful one; sail_controller only commands the stepper while it is true. Progress is logged.

Calibration sequence:
    1. Search: run fast in the positive direction until the switch is reached
       (skipped if the sail is known to sit on the switch).
    2. Clear: keep running in the positive direction at measuring speed until leaving the
       switch, so that both measuring passes cross the whole switch at constant speed.
    3. Negative pass: run slowly in the negative direction across the switch -> record p1
       where it is left.
    4. Positive pass: run slowly in the positive direction across the switch -> record p2
       where it is left.
    5. Center: step to (p1 + p2) / 2 and zero the stepper and the encoder there.
    6. Offset: wait offset_move_delay, step to zero_offset and zero both again
       (skipped when zero_offset is 0).
    7. Hold there, engaged.

A ``sail/measure_scale`` service (kingfisher_msgs/MeasureSailScale) checks the stepper
``rescale_factor`` and the encoder tick factor against real sail turns:
    1. Run at measuring speed in the positive direction.
    2. Record both positions at every entry into the switch, interpolated between joint
       samples at the entry time. The first entry is the reference.
    3. After ``laps`` more entries, stop, log each lap and suggest corrected constants.
``sail/calibrated`` is false from the start of a measurement: recalibrate after it.
"""

from enum import auto, Enum
from math import degrees, pi

from kingfisher_msgs.srv import MeasureSailScale
from phidgets_msgs.msg import StepperCommand, StepperState
from phidgets_msgs.srv import Trigger as ChannelTrigger
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class CalibrationState(Enum):
    """Steps of the calibration sequence."""

    IDLE = auto()
    SEARCHING_SWITCH = auto()
    CLEARING_SWITCH = auto()
    MEASURING_NEGATIVE_PASS = auto()
    MEASURING_POSITIVE_PASS = auto()
    MOVING_TO_CENTER = auto()
    ZEROING_CENTER = auto()
    WAITING_BEFORE_OFFSET_MOVE = auto()
    MOVING_TO_OFFSET = auto()
    ZEROING_OFFSET = auto()
    MEASURING_SCALE = auto()
    FETCHING_SCALE_PARAMETERS = auto()


SCALE_STATES = (CalibrationState.MEASURING_SCALE, CalibrationState.FETCHING_SCALE_PARAMETERS)
# Switch entries this soon after the start of a scale measurement are ignored, so that no
# interpolation uses a sample from before the sail reached constant speed (s)
SCALE_SETTLE_TIME = 1.0


class _EdgeTrack:
    """Joint samples of one source, interpolated at switch edge times."""

    def __init__(self):
        self.reset()

    def reset(self):
        # (time, position) of the last sample
        self.last = None
        # Edge times not yet followed by a sample
        self.pending = []
        # Position at each resolved edge
        self.edges = []

    def sample(self, t, position):
        if self.last is not None:
            t0, p0 = self.last
            while self.pending and self.pending[0] <= t:
                edge_time = self.pending.pop(0)
                fraction = (edge_time - t0) / (t - t0) if t > t0 else 1.0
                self.edges.append(p0 + (position - p0) * fraction)
        self.last = (t, position)

    def edge(self, t):
        self.pending.append(t)


class SailCalibration(Node):
    """Run the sail calibration sequence on request."""

    def __init__(self):
        super().__init__('sail_calibration')

        # Velocity used to search for the switch and to move to the final position (rad/s)
        self._search_velocity = self.declare_parameter('search_velocity', 2.0).value
        # Velocity used for the two passes that measure the switch edges (rad/s)
        self._measure_velocity = self.declare_parameter('measure_velocity', 0.2).value
        # Offset from the switch center to the true zero (rad)
        self._zero_offset = self.declare_parameter('zero_offset', -pi/2).value
        # Pause between zeroing at the switch center and moving to the offset (s)
        self._offset_move_delay = self.declare_parameter('offset_move_delay', 0.8).value
        self._position_tolerance = self.declare_parameter('position_tolerance', 1e-3).value
        self._encoder_channel = self.declare_parameter('encoder_channel', 0).value

        self.create_subscription(Bool, 'digital_input00', self._switch_callback, 1)
        self.create_subscription(JointState, 'phidgets_stepper/joint', self._joint_callback, 1)
        self.create_subscription(JointState, 'sail/joint_states', self._encoder_callback, 10)
        self.create_subscription(
            StepperState, 'phidgets_stepper/state', self._stepper_callback, 10)

        self._command_pub = self.create_publisher(StepperCommand, 'phidgets_stepper/command', 1)
        self._calibrated_pub = self.create_publisher(
            Bool, 'sail/calibrated',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._stepper_zero_client = self.create_client(Trigger, 'phidgets_stepper/zero')
        self._encoder_zero_client = self.create_client(
            ChannelTrigger, 'phidgets_high_speed_encoder/zero')
        self._stepper_parameters_client = self.create_client(
            GetParameters, 'phidgets_stepper/get_parameters')
        self._encoder_parameters_client = self.create_client(
            GetParameters, 'phidgets_high_speed_encoder/get_parameters')

        self.create_service(Trigger, 'sail/calibrate', self._calibrate_callback)
        self.create_service(MeasureSailScale, 'sail/measure_scale', self._measure_scale_callback)

        self._state = CalibrationState.IDLE
        self._last_joint = None
        # None until the first message: the driver only publishes on changes
        self._on_switch = None
        # Whether the current measuring pass has reached the switch yet
        self._pass_reached_switch = False
        self._p1 = None
        self._p2 = None
        self._center = None
        # Position the current STEP move is heading to
        self._target = None
        self._zero_futures = []
        self._offset_timer = None
        self._scale_laps = 0
        self._scale_start = None
        self._stepper_track = _EdgeTrack()
        self._encoder_track = _EdgeTrack()
        # Laps already logged
        self._scale_logged = 0
        self._parameter_futures = []
        self._calibrated_pub.publish(Bool(data=False))

    # ---------------------------------------------------------------- service

    def _calibrate_callback(self, request, response):
        if self._state != CalibrationState.IDLE:
            response.success = False
            response.message = self._busy_message()
            return response

        error = self._check_ready()
        if error:
            self.get_logger().warn(f'Calibration not started: {error}')
            response.success = False
            response.message = error
            return response

        self._p1 = self._p2 = self._center = self._target = None
        # Before the first stepper command, so that sail_controller stops commanding
        self._calibrated_pub.publish(Bool(data=False))
        self.get_logger().info('Calibration started')
        if self._on_switch:
            self._start_clearing('Already on the switch')
        else:
            self._run(self._search_velocity)
            self._set_state(CalibrationState.SEARCHING_SWITCH, 'Searching for the switch')

        response.success = True
        response.message = 'Calibration started'
        return response

    def _measure_scale_callback(self, request, response):
        if self._state != CalibrationState.IDLE:
            response.success = False
            response.message = self._busy_message()
            return response

        error = 'laps must be at least 1' if request.laps < 1 else self._check_ready()
        if error:
            self.get_logger().warn(f'Scale measurement not started: {error}')
            response.success = False
            response.message = error
            return response

        self._scale_laps = request.laps
        self._scale_start = self._now()
        self._stepper_track.reset()
        self._encoder_track.reset()
        self._scale_logged = 0
        self._calibrated_pub.publish(Bool(data=False))
        self._run(self._measure_velocity)
        self._set_state(
            CalibrationState.MEASURING_SCALE,
            f'Scale measurement started: {self._scale_laps} laps '
            f'at {self._measure_velocity} rad/s')

        response.success = True
        response.message = 'Scale measurement started'
        return response

    def _busy_message(self):
        if self._state in SCALE_STATES:
            return 'Scale measurement in progress'
        return 'Calibration already in progress'

    def _check_ready(self):
        """Return an error message if calibration cannot start, else None."""
        if self._last_joint is None:
            return 'No joint state received on phidgets_stepper/joint'
        if not self._stepper_zero_client.service_is_ready():
            return 'Service phidgets_stepper/zero not available'
        if not self._encoder_zero_client.service_is_ready():
            return 'Service phidgets_high_speed_encoder/zero not available'
        return None

    # ------------------------------------------------------- sensor callbacks

    def _joint_callback(self, msg):
        self._last_joint = msg
        if self._state == CalibrationState.MEASURING_SCALE:
            self._stepper_track.sample(self._now(), msg.position[0])
            self._check_laps()

    def _encoder_callback(self, msg):
        if self._state == CalibrationState.MEASURING_SCALE:
            self._encoder_track.sample(self._now(), msg.position[0])
            self._check_laps()

    def _switch_callback(self, msg):
        # The digital input reads False while the magnet is over the switch
        was_on_switch = self._on_switch
        self._on_switch = not msg.data

        if self._state == CalibrationState.SEARCHING_SWITCH:
            if self._on_switch:
                self._start_clearing('Found the switch')
            elif was_on_switch is None:
                # The input is only published on changes, so a first reading of 'off' means
                # the sail started on the switch and has just left it.
                self._start_pass(
                    CalibrationState.MEASURING_NEGATIVE_PASS, -self._search_velocity,
                    'Left the switch (started on it), starting negative pass')

        elif self._state == CalibrationState.CLEARING_SWITCH and not self._on_switch:
            self._start_pass(
                CalibrationState.MEASURING_NEGATIVE_PASS, -self._measure_velocity,
                'Cleared the switch, starting negative pass')

        elif self._state == CalibrationState.MEASURING_NEGATIVE_PASS:
            if self._pass_completed():
                self._p1 = self._joint_position()
                self._start_pass(
                    CalibrationState.MEASURING_POSITIVE_PASS, self._measure_velocity,
                    f'Negative pass done, p1={self._p1:.4f}, starting positive pass')

        elif self._state == CalibrationState.MEASURING_POSITIVE_PASS:
            if self._pass_completed():
                self._p2 = self._joint_position()
                self._center = (self._p1 + self._p2) / 2
                self._start_move(
                    CalibrationState.MOVING_TO_CENTER, self._center,
                    f'Positive pass done, p2={self._p2:.4f}, '
                    f'moving to center={self._center:.4f}')

        elif self._state == CalibrationState.MEASURING_SCALE and self._on_switch \
                and not was_on_switch:
            self._scale_edge()

    def _stepper_callback(self, msg):
        if self._state == CalibrationState.MOVING_TO_CENTER and self._at_target(msg):
            self._start_zeroing(
                CalibrationState.ZEROING_CENTER, 'Zeroing stepper and encoder at switch center')
        elif self._state == CalibrationState.MOVING_TO_OFFSET and self._at_target(msg):
            self._start_zeroing(
                CalibrationState.ZEROING_OFFSET, 'Zeroing stepper and encoder at offset')

    def _at_target(self, stepper_state):
        error = abs(self._joint_position() - self._target)
        return error < self._position_tolerance and not stepper_state.is_moving

    # ----------------------------------------------------------- calibration

    def _start_clearing(self, message):
        self._run(self._measure_velocity)
        self._set_state(CalibrationState.CLEARING_SWITCH, message)

    def _start_pass(self, state, velocity, message):
        self._pass_reached_switch = False
        self._run(velocity)
        self._set_state(state, message)

    def _pass_completed(self):
        """Return True once the current pass has entered and then left the switch."""
        if self._on_switch:
            self._pass_reached_switch = True
            return False
        return self._pass_reached_switch

    def _start_move(self, state, target, message):
        self._target = target
        self._step_to(target, self._search_velocity)
        self._set_state(state, message)

    def _start_zeroing(self, state, message):
        self._set_state(state, message)

        encoder_request = ChannelTrigger.Request()
        encoder_request.channel = self._encoder_channel

        self._zero_futures = [
            ('stepper', self._stepper_zero_client.call_async(Trigger.Request())),
            ('encoder', self._encoder_zero_client.call_async(encoder_request)),
        ]
        for _, future in self._zero_futures:
            future.add_done_callback(self._zero_done_callback)

    def _zero_done_callback(self, _future):
        if self._state not in (CalibrationState.ZEROING_CENTER, CalibrationState.ZEROING_OFFSET):
            return
        if not all(future.done() for _, future in self._zero_futures):
            return

        errors = []
        for name, future in self._zero_futures:
            result = future.result()
            if result is None or not result.success:
                message = result.message if result is not None else 'no response'
                errors.append(f'{name} zero failed ({message})')
        self._zero_futures = []

        if errors:
            self._finish('Calibration failed: ' + '; '.join(errors), success=False)
        elif self._state == CalibrationState.ZEROING_CENTER and self._zero_offset != 0.0:
            self._offset_timer = self.create_timer(
                self._offset_move_delay, self._offset_timer_callback)
            self._set_state(
                CalibrationState.WAITING_BEFORE_OFFSET_MOVE,
                f'Zeroed at switch center, moving to offset in {self._offset_move_delay}s')
        else:
            self._finish('Calibration complete', success=True)

    def _offset_timer_callback(self):
        self.destroy_timer(self._offset_timer)
        self._offset_timer = None
        # The switch center is now 0, so the offset is the absolute target
        self._start_move(
            CalibrationState.MOVING_TO_OFFSET, self._zero_offset,
            f'Moving to offset={self._zero_offset:.4f}')

    def _finish(self, message, success):
        # The stepper stays engaged and holds where the last step move left it
        self._set_state(CalibrationState.IDLE, message)
        self._calibrated_pub.publish(Bool(data=success))

    # ------------------------------------------------------ scale measurement

    def _scale_edge(self):
        """Record a switch entry on both tracks."""
        now = self._now()
        tracks = (self._stepper_track, self._encoder_track)
        if not self._stepper_track.pending and not self._stepper_track.edges:
            # The reference entry needs a sample from each source at constant speed
            settled = self._scale_start + SCALE_SETTLE_TIME
            if any(track.last is None or track.last[0] < settled for track in tracks):
                self.get_logger().info('Reached the switch before settling, waiting one lap')
                return
            self.get_logger().info('Reference entry into the switch')
        for track in tracks:
            track.edge(now)

    def _check_laps(self):
        """Log the laps resolved on both tracks, and finish after the last one."""
        resolved = min(len(self._stepper_track.edges), len(self._encoder_track.edges))
        while self._scale_logged < resolved - 1:
            k = self._scale_logged
            stepper = self._stepper_track.edges[k + 1] - self._stepper_track.edges[k]
            encoder = self._encoder_track.edges[k + 1] - self._encoder_track.edges[k]
            self._scale_logged += 1
            self.get_logger().info(
                f'Lap {self._scale_logged}/{self._scale_laps}: '
                f'stepper {stepper:.4f} rad ({degrees(stepper):.2f} deg), '
                f'encoder {encoder:.4f} rad ({degrees(encoder):.2f} deg)')
        if self._scale_logged >= self._scale_laps:
            self._finish_scale()

    def _finish_scale(self):
        self._run(0.0)
        stepper = self._lap_stats('stepper', self._stepper_track.edges)
        encoder = self._lap_stats('encoder', self._encoder_track.edges)
        gap = (self._stepper_track.edges[-1] - self._stepper_track.edges[0]) - \
            (self._encoder_track.edges[-1] - self._encoder_track.edges[0])
        self.get_logger().info(
            f'Stepper - encoder over {self._scale_laps} laps: {degrees(gap):.3f} deg '
            '(non-zero means missed steps)')

        requests = [
            (self._stepper_parameters_client, 'phidgets_stepper', 'rescale_factor', stepper),
            (self._encoder_parameters_client, 'phidgets_high_speed_encoder',
             f'joint{self._encoder_channel}_tick2rad', encoder),
        ]
        self._parameter_futures = []
        for client, node, name, correction in requests:
            if correction is None:
                continue
            if not client.service_is_ready():
                self.get_logger().warn(
                    f'{node}/get_parameters not available: multiply {name} by '
                    f'{correction:.6f}')
                continue
            future = client.call_async(GetParameters.Request(names=[name]))
            self._parameter_futures.append((node, name, correction, future))
            future.add_done_callback(self._parameters_done_callback)

        if self._parameter_futures:
            self._set_state(
                CalibrationState.FETCHING_SCALE_PARAMETERS, 'Fetching the current constants')
        else:
            self._finish_scale_measurement()

    def _lap_stats(self, name, edges):
        """Log the lap statistics of one source and return its correction factor."""
        laps = [b - a for a, b in zip(edges, edges[1:])]
        mean = sum(laps) / len(laps)
        spread = max(laps) - min(laps)
        if mean <= 0.0:
            self.get_logger().error(f'{name}: mean lap {mean:.4f} rad is not positive')
            return None
        error = mean - 2 * pi
        self.get_logger().info(
            f'{name}: mean lap {mean:.5f} rad ({degrees(mean):.3f} deg), '
            f'error {degrees(error):+.3f} deg per lap, spread {degrees(spread):.3f} deg. '
            f'If only the gear ratio is off, the real one is {mean / (2 * pi):.5f} '
            'times the configured one')
        if spread > abs(error):
            self.get_logger().warn(
                f'{name}: the spread between laps is larger than the error; '
                'look for slip before changing the constants')
        return 2 * pi / mean

    def _parameters_done_callback(self, _future):
        if self._state != CalibrationState.FETCHING_SCALE_PARAMETERS:
            return
        if not all(future.done() for *_, future in self._parameter_futures):
            return

        for node, name, correction, future in self._parameter_futures:
            result = future.result()
            value = result.values[0] if result is not None and result.values else None
            if value is None or value.type != ParameterType.PARAMETER_DOUBLE:
                self.get_logger().warn(
                    f'Could not read {node} {name}: multiply it by {correction:.6f}')
                continue
            self.get_logger().info(
                f'{node} {name}: {value.double_value!r} -> suggested '
                f'{name}: {value.double_value * correction:.17g}')
        self._parameter_futures = []
        self._finish_scale_measurement()

    def _finish_scale_measurement(self):
        self._set_state(
            CalibrationState.IDLE,
            'Scale measurement complete. The zero is no longer trusted: recalibrate')

    # ---------------------------------------------------------------- helpers

    def _set_state(self, state, message):
        self._state = state
        self.get_logger().info(message)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _joint_position(self):
        return self._last_joint.position[0]

    def _send_command(self, mode, target=0.0, velocity=0.0):
        cmd = StepperCommand()
        cmd.mode = mode
        cmd.target = float(target)
        cmd.velocity = float(velocity)
        self._command_pub.publish(cmd)

    def _run(self, velocity):
        self._send_command(StepperCommand.CONTROL_MODE_RUN, velocity=velocity)

    def _step_to(self, position, velocity):
        self._send_command(StepperCommand.CONTROL_MODE_STEP, target=position, velocity=velocity)


def main(args=None):
    """Spin the sail calibration node."""
    rclpy.init(args=args)
    node = SailCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
