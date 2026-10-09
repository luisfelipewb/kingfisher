# kingfisher

ROS 2 stack for the Kingfisher ASV.

## Packages

| package | build type | what it does |
|---|---|---|
| `kingfisher_msgs` | ament_cmake | `Drive` and `Sense`, wire-compatible with the MCU |
| `kingfisher_description` | ament_cmake | URDF/xacro and meshes, no Gazebo |
| `kingfisher_rosserial` | — | the MCU bridge, see its own README |
| `kingfisher_twist` | ament_python | `cmd_vel` → `cmd_drive` |
| `kingfisher_viz` | ament_python | thrust arrows for RViz |
| `wifi_monitor` | ament_python | `has_wifi` link liveness |
| `kingfisher_teleop` | ament_cmake | joystick config for `joy` + `joy_teleop` |
| `kingfisher_sail` | ament_python | sail command interface and calibration, see its own README |
| `kingfisher_bringup` | ament_python | what the boat actually runs |

## Running the boat

```bash
ros2 launch kingfisher_bringup robot.launch.py
```

Like the sim (VRX), the boat runs under the `kingfisher` namespace: topics are
`/kingfisher/cmd_vel`, `/kingfisher/joint_states`, … and frames are
`kingfisher/base_link`, …. `/tf` and `/tf_static` stay global. Two launch
arguments control this, `namespace` (default `kingfisher`) and `frame_prefix`
(default `kingfisher/`); `namespace:=/ frame_prefix:=/` gives plain root
topics and frames (`/` because `ros2 launch` rejects empty values; a leading
`/` is stripped from `frame_prefix`).

`kingfisher_bringup` is self-contained: it declares nodes directly against
`kingfisher_bringup/config/kingfisher.yaml`. That one file is the boat's
configuration. Its keys are `/**/<node name>` so they match in any namespace;
a plain `<node name>` key would be silently ignored under `/kingfisher`. Each package also keeps its own `launch/` and `config/` for
running a node on its own while debugging; those are not used in normal operation.

## Teleop

`kingfisher_teleop` configures `joy`'s `game_controller_node` and `joy_teleop` for a Logitech F310 in
XInput mode. Hold LB to drive through `cmd_vel` (left stick throttle, right stick turn), or RB for
`cmd_drive` (one stick per thruster). Releasing either stops the stream, and the thrusters stop. The sail:
- LB + D-pad left/right turns it counter-clockwise/clockwise (`sail/cmd_velocity`) until released;
- RB + D-pad up, down, left, right sets it to 0, 180°, +90°, −90° (`sail/cmd_position`);
- Back + Start calibrates it (`sail/calibrate`).

| where the stick is | on the stick's machine | next to the robot |
|---|---|---|
| same machine | — | `ros2 launch kingfisher_teleop teleop.launch.py` |
| laptop | `ros2 launch kingfisher_teleop joy.launch.py` | `ros2 launch kingfisher_teleop teleop.launch.py joy:=false` |

Both default to the `kingfisher` namespace, like the boat and the sim. A laptop without this repo can run
`ros2 run joy game_controller_node --ros-args -r __ns:=/kingfisher -p autorepeat_rate:=30.0 -p coalesce_interval_ms:=20`.

## Robot description

`kingfisher_description` holds the boat's geometry. Link names are plain
(`base_link`, `imu_link`, `lidar_link`, …); `robot_state_publisher` adds the
`kingfisher/` prefix through `frame_prefix`, on the boat (`robot.launch.py`)
as in sim and `display.launch.py`. The simulation
([`kingfisher_simulation`](https://github.com/luisfelipewb/kingfisher_simulation))
builds on the same URDF, so the sim and the boat share one set of frames.

```bash
ros2 launch kingfisher_description display.launch.py   # RViz, no boat needed
```

On the boat, `robot.launch.py` runs `robot_state_publisher` on this URDF.
`joint_state_publisher` merges the sail encoder (`/kingfisher/sail/joint_states`)
into `/kingfisher/joint_states` and sends the propeller joints as 0. The URDF replaced the
ROS 1 static transforms, so bags recorded before the switch use the old
frame names: `sbg` -> `imu_link`, `laser` -> `lidar_link`, `gps_aN` ->
`gps_aN_link`, `thruster_<side>` -> `<side>_thruster_link`,
`sail_encoder_link` -> `sail_motor_link` / `sail_link`.

The package started from Clearpath's
[kf/kingfisher](https://github.com/kf/kingfisher) @ `indigo-devel` `c7fb559`.

## The sail

`robot.launch.py` starts the Phidgets drivers (sail stepper, sail encoder,
switch input) and `kingfisher_sail`'s `sail_calibration` and `sail_controller`,
all with their parameters from `kingfisher.yaml`. Commands
(`sail/cmd_position`, `sail/cmd_velocity`) are ignored until the sail is
calibrated, once after every start:

```bash
ros2 service call /kingfisher/sail/calibrate std_srvs/srv/Trigger
```

`/kingfisher/sail/calibrated` turns `true` when it is done. See `kingfisher_sail/README.md`.

The Phidgets container in `robot.launch.py` supersedes `sawasp/phidgets_launch`.

## Deliberately left on `noetic`

`kingfisher_node` (`kingfisher.py`, `sonarmite.py`, `trial.py`), the ROS 1
`kingfisher_drive_viz`, the rosbuild `kingfisher_bringup` tree
(`upstart/`, `vserial/`, the `.launch` files), `*.rosinstall`, `stack.xml`.
`kingfisher_nmea` was never more than a rosinstall file.
