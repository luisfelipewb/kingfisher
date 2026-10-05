# kingfisher_sail

Sail calibration. One node, `sail_calibration`, that finds the magnet switch on
the sail mast, moves the sail to its true zero and zeroes the stepper and the
encoder there. It does nothing until its service is called.

Supersedes `phidgets_launch/nodes/calibrate.py` in
[sawasp](https://optimav.georgiatech-metz.fr/cedricpradalier/sawasp.git), which
calibrated as soon as it was started and only logged progress to the terminal.

## Running

On the boat the node is started by `kingfisher_bringup/robot.launch.py`, with
its parameters from `kingfisher_bringup/config/kingfisher.yaml`. The defaults in
the node match that file. To run it on its own:

```bash
ros2 run kingfisher_sail kingfisher_sail
```

Either way, the Phidgets drivers (stepper, high-speed encoder, digital inputs)
must be up; they are launched from `sawasp/phidgets_launch`
(`launch_all.launch.py` or `launch_all_wired.launch.py`).

```bash
ros2 topic echo /sail_calibration/status
ros2 service call /sail_calibration/calibrate std_srvs/srv/Trigger
```

The service answers immediately: `success: true` means the calibration has
started, not that it has finished. Follow `~/status` for progress; the last
message is `Calibration complete` or `Calibration failed: ...`. A call is
refused while a calibration is running, before any joint state has arrived, or
while either zero service is unavailable.

## Sequence

1. **Search** — run at `search_velocity` (positive) until the switch is reached.
   Skipped if the sail is known to be on the switch.
2. **Clear** — run at `search_velocity` (positive) until the switch is left.
   Nothing is measured here.
3. **Negative pass** — run at `-measure_velocity` across the switch; `p1` is
   where it is left.
4. **Positive pass** — run at `measure_velocity` across the switch; `p2` is
   where it is left.
5. **Center** — step to `(p1 + p2) / 2`, zero stepper and encoder.
6. **Offset** — wait `offset_move_delay`, step to `zero_offset`, zero both
   again. Skipped when `zero_offset` is 0.
7. Disengage the stepper.

Both measured edges are crossings of the whole switch at the same constant
speed in opposite directions, so detection latency shifts `p1` and `p2` equally
and cancels in the average. That is what the clear step is for.

## Interface

| | name | type |
|---|---|---|
| service | `~/calibrate` | `std_srvs/Trigger` |
| pub | `~/status` | `std_msgs/String` |
| pub | `/phidgets_stepper/command` | `phidgets_msgs/StepperCommand` |
| sub | `/digital_input00` | `std_msgs/Bool` — `false` while on the switch |
| sub | `/phidgets_stepper/joint` | `sensor_msgs/JointState` |
| sub | `/phidgets_stepper/state` | `phidgets_msgs/StepperState` |
| client | `/phidgets_stepper/zero` | `std_srvs/Trigger` |
| client | `/phidgets_high_speed_encoder/zero` | `phidgets_msgs/Trigger` |

## Parameters

| name | default | |
|---|---|---|
| `search_velocity` | 2.0 rad/s | search and the moves to center / offset |
| `measure_velocity` | 0.2 rad/s | clear and both measuring passes |
| `zero_offset` | −π/2 rad | true zero relative to the switch center |
| `offset_move_delay` | 0.8 s | pause between the center zero and the offset move |
| `position_tolerance` | 1e-3 rad | "arrived" threshold for the step moves |
| `encoder_channel` | 0 | channel passed to the encoder zero service |

## Notes

- **`/digital_input00` is only published on change.** With the driver's
  `publish_rate` at 0 (as launched), a node started after the driver never
  sees the current switch state.
- **Overshoot depends on the stepper acceleration**, which is set in the
  Phidgets launch file. Slowing from `search_velocity` to `measure_velocity`
  At low acceleration, lower `search_velocity` if calibration takes too long.
- **`zero_offset` is measured in the stepper's positive direction** from the
  switch center. Its sign depends on how the switch is mounted.
