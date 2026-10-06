"""
Sail calibration node.

Exposes a ``~/calibrate`` service (std_srvs/Trigger) that locates the magnet switch, moves
the sail to the true zero and zeroes both the stepper and the encoder. Progress is published
as plain text on ``~/status``.

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
    7. Disengage the stepper.
"""

from enum import auto, Enum
from math import pi

from phidgets_msgs.msg import StepperCommand, StepperState
from phidgets_msgs.srv import Trigger as ChannelTrigger
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
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
        self.create_subscription(
            StepperState, 'phidgets_stepper/state', self._stepper_callback, 10)

        self._command_pub = self.create_publisher(StepperCommand, 'phidgets_stepper/command', 1)
        self._status_pub = self.create_publisher(String, '~/status', 10)

        self._stepper_zero_client = self.create_client(Trigger, 'phidgets_stepper/zero')
        self._encoder_zero_client = self.create_client(
            ChannelTrigger, 'phidgets_high_speed_encoder/zero')

        self.create_service(Trigger, '~/calibrate', self._calibrate_callback)

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

    # ---------------------------------------------------------------- service

    def _calibrate_callback(self, request, response):
        if self._state != CalibrationState.IDLE:
            response.success = False
            response.message = 'Calibration already in progress'
            return response

        error = self._check_ready()
        if error:
            self._publish_status(f'Calibration not started: {error}')
            response.success = False
            response.message = error
            return response

        self._p1 = self._p2 = self._center = self._target = None
        self._publish_status('Calibration started')
        if self._on_switch:
            self._start_clearing('Already on the switch')
        else:
            self._run(self._search_velocity)
            self._set_state(CalibrationState.SEARCHING_SWITCH, 'Searching for the switch')

        response.success = True
        response.message = 'Calibration started'
        return response

    def _check_ready(self):
        """Return an error message if calibration cannot start, else None."""
        if self._last_joint is None:
            return 'No joint state received on /phidgets_stepper/joint'
        if not self._stepper_zero_client.service_is_ready():
            return 'Service /phidgets_stepper/zero not available'
        if not self._encoder_zero_client.service_is_ready():
            return 'Service /phidgets_high_speed_encoder/zero not available'
        return None

    # ------------------------------------------------------- sensor callbacks

    def _joint_callback(self, msg):
        self._last_joint = msg

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
            self._finish('Calibration failed: ' + '; '.join(errors))
        elif self._state == CalibrationState.ZEROING_CENTER and self._zero_offset != 0.0:
            self._offset_timer = self.create_timer(
                self._offset_move_delay, self._offset_timer_callback)
            self._set_state(
                CalibrationState.WAITING_BEFORE_OFFSET_MOVE,
                f'Zeroed at switch center, moving to offset in {self._offset_move_delay}s')
        else:
            self._finish('Calibration complete')

    def _offset_timer_callback(self):
        self.destroy_timer(self._offset_timer)
        self._offset_timer = None
        # The switch center is now 0, so the offset is the absolute target
        self._start_move(
            CalibrationState.MOVING_TO_OFFSET, self._zero_offset,
            f'Moving to offset={self._zero_offset:.4f}')

    def _finish(self, message):
        self._disengage()
        self._set_state(CalibrationState.IDLE, message)

    # ---------------------------------------------------------------- helpers

    def _set_state(self, state, message):
        self._state = state
        self._publish_status(message)

    def _publish_status(self, message):
        self.get_logger().info(message)
        self._status_pub.publish(String(data=message))

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

    def _disengage(self):
        self._send_command(StepperCommand.CONTROL_MODE_DISENGAGED)


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
