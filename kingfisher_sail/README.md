# kingfisher_sail

The sail's command interface and its calibration. Two nodes:

- `sail_controller` turns position and velocity commands into Phidgets stepper commands. It runs on
  the boat and in sim, where `kingfisher_sim`'s `sail_stepper_sim` stands in for the stepper.
- `sail_calibration` finds the magnet switch on the mast, moves the sail to its true zero and zeroes the
  stepper and the encoder there. Boat only.

Supersedes `phidgets_launch/nodes/calibrate.py` in
[sawasp](https://optimav.georgiatech-metz.fr/cedricpradalier/sawasp.git), which calibrated as soon as
it was started.

## Running

On the boat, `kingfisher_bringup/robot.launch.py` starts both nodes and the Phidgets drivers (stepper,
high-speed encoder, digital inputs) under `/kingfisher`, with parameters from
`kingfisher_bringup/config/kingfisher.yaml`. The defaults in the nodes match that file. Names are
relative, so to run a node on its own, join the bringup namespace:

```bash
ros2 run kingfisher_sail sail_controller --ros-args -r __ns:=/kingfisher
```

Calibrate once after every start, then command:

```bash
ros2 service call /kingfisher/sail/calibrate std_srvs/srv/Trigger
ros2 topic echo /kingfisher/sail/calibrated
ros2 topic pub --once /kingfisher/sail/cmd_position std_msgs/msg/Float64 '{data: 1.5708}'
ros2 topic pub -r 10 /kingfisher/sail/cmd_velocity std_msgs/msg/Float64 '{data: 0.5}'
```

The service answers immediately: `success: true` means the calibration has started. The log shows the
progress, and `sail/calibrated` turns `true` when it is done.

## Interface

Relative to the node's namespace (`/kingfisher` under bringup). Angles are rad, + counter-clockwise
seen from above; 0 is the sail along the hull, long side aft.

| | name | type | node |
|---|---|---|---|
| sub | `sail/cmd_position` | `std_msgs/Float64`, rad | controller |
| sub | `sail/cmd_velocity` | `std_msgs/Float64`, rad/s | controller |
| sub | `joint_states` (`sail_joint`) | `sensor_msgs/JointState` | controller |
| pub | `sail/calibrated` | `std_msgs/Bool`, latched | calibration |
| service | `sail/calibrate` | `std_srvs/Trigger` | calibration |
| pub | `phidgets_stepper/command` | `phidgets_msgs/StepperCommand` | both |
| sub | `digital_input00` | `std_msgs/Bool`, `false` while on the switch | calibration |
| sub | `phidgets_stepper/joint`, `phidgets_stepper/state` | `JointState`, `StepperState` | calibration |
| client | `phidgets_stepper/zero`, `phidgets_high_speed_encoder/zero` | `std_srvs/Trigger`, `phidgets_msgs/Trigger` | calibration |

## sail_controller

- **Position.** The command is wrapped to [−π, π], and the sail takes the short way from the measured
  `sail_joint` angle (a STEP at `max_velocity`). The stepper keeps counting turns, so a command never
  unwinds them.
- **Velocity.** A non-zero command runs the stepper (RUN), clamped to `max_velocity`. A 0, or no
  command for `velocity_timeout`, sends RUN at 0, so the stepper ramps down at its `acceleration` and
  holds. Velocity commands must stream: `joy_teleop` sends nothing when its deadman is released.
- **Velocity wins.** Positions are ignored until `position_holdoff` after the last non-zero velocity.
  Zeros don't count, so a teleop streaming 0 doesn't block positions.
- **Calibration.** Nothing is sent to the stepper while `sail/calibrated` is false or hasn't arrived.
- Ignored commands and mode changes are logged.

| parameter | default | |
|---|---|---|
| `max_velocity` | 2.0 rad/s | STEP velocity limit and RUN clamp |
| `velocity_timeout` | 0.25 s | a jog stops after this long without `cmd_velocity` |
| `position_holdoff` | 1.0 s | `cmd_position` is ignored this long after a non-zero `cmd_velocity` |
| `joint` | `sail_joint` | the measured joint in `joint_states` |

The acceleration is the stepper driver's `acceleration` parameter (10 rad/s²).

## sail_calibration

1. **Search**: run at `search_velocity` (positive) until the switch is reached. Skipped if the sail is
   known to be on the switch.
2. **Clear**: run at `measure_velocity` (positive) until the switch is left. Nothing is measured here.
3. **Negative pass**: run at `-measure_velocity` across the switch; `p1` is where it is left.
4. **Positive pass**: run at `measure_velocity` across the switch; `p2` is where it is left.
5. **Center**: step to `(p1 + p2) / 2`, zero stepper and encoder.
6. **Offset**: wait `offset_move_delay`, step to `zero_offset`, zero both again. Skipped when
   `zero_offset` is 0.
7. **Hold** there, engaged.

Both measured edges are crossings of the whole switch at the same constant speed in opposite
directions, so detection latency shifts `p1` and `p2` equally and cancels in the average. That is what
the clear step is for.

A call is refused while a calibration is running, before any stepper joint state has arrived, or while
either zero service is unavailable.

| parameter | default | |
|---|---|---|
| `search_velocity` | 2.0 rad/s | search and the moves to center / offset |
| `measure_velocity` | 0.2 rad/s | clear and both measuring passes |
| `zero_offset` | −π/2 rad | true zero relative to the switch center |
| `offset_move_delay` | 0.8 s | pause between the center zero and the offset move |
| `position_tolerance` | 1e-3 rad | "arrived" threshold for the step moves |
| `encoder_channel` | 0 | channel passed to the encoder zero service |

## Failure modes

Known, not handled:
- **Missed steps.** STEP targets are in the stepper's count. If the stepper slips (wind load, too much
  acceleration), its count drifts from the encoder, and the sail ends off target by the drift.
  `joint_states` (the encoder) still shows the real angle. Compare it with `phidgets_stepper/joint`,
  and recalibrate.
- **The stepper driver restarts** (container respawn, power loss at the hub). Its count resets, so the
  zero is lost while `sail/calibrated` still says `true`. Recalibrate.
- **RUN ↔ STEP briefly disengages the motor.** The driver does it on every mode change. The controller
  only changes mode after a jog has stopped, so the sail just loses holding torque for an instant.

## Notes

- **`digital_input00` is only published on change.** With the driver's `publish_rate` at 0 (as
  launched), a node started after the driver never sees the current switch state.
- **Overshoot depends on the stepper acceleration**, `phidgets_stepper.acceleration` in
  `kingfisher.yaml`. Calibration slows from `search_velocity` to `measure_velocity` at that rate; at low
  acceleration, lower `search_velocity` if calibration takes too long.
- **`zero_offset` is measured in the stepper's positive direction** from the switch center. Its sign
  depends on how the switch is mounted.
