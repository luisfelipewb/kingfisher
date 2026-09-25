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
| `kingfisher_bringup` | ament_python | what the boat actually runs |

## Running the boat

```bash
ros2 launch kingfisher_bringup robot.launch.py
```

`kingfisher_bringup` is self-contained: it declares nodes directly against
`kingfisher_bringup/config/kingfisher.yaml`. That one file is the boat's
configuration. Each package also keeps its own `launch/` and `config/` for
running a node on its own while debugging; those are not used in normal operation.

## Teleop

`kingfisher_teleop` configures `joy` and `joy_teleop` for a Logitech F310 in XInput mode. Hold LB to
drive through `cmd_vel` (left stick throttle, right stick turn), or RB for `cmd_drive` (one stick per
thruster). Releasing either stops the stream, and the thrusters stop. X, Back, Y, Start and B set the
sail to +90°, +45°, 0, −45° and −90°.

| where the stick is | on the stick's machine | next to the robot |
|---|---|---|
| same machine | — | `ros2 launch kingfisher_teleop teleop.launch.py` |
| laptop | `ros2 launch kingfisher_teleop joy.launch.py` | `ros2 launch kingfisher_teleop teleop.launch.py joy:=false` |

Both default to the `kingfisher` namespace. The boat still runs in the root namespace, so add
`namespace:=/` there. A laptop without this repo can run
`ros2 run joy joy_node --ros-args -r __ns:=/kingfisher -p autorepeat_rate:=30.0 -p coalesce_interval_ms:=20`.

## Robot description

`kingfisher_description` holds the boat's geometry. Link names are plain
(`base_link`, `imu_link`, `lidar_link`, …); `robot_state_publisher` adds the
`kingfisher/` prefix through `frame_prefix`. The simulation
([`kingfisher_simulation`](https://github.com/luisfelipewb/kingfisher_simulation))
builds on the same URDF, so the sim and the boat share one set of frames.

```bash
ros2 launch kingfisher_description display.launch.py   # RViz, no boat needed
```

`robot.launch.py` still publishes the older frames from `static_tfs.launch.py`
(`sbg`, `laser`, …). The switch to `robot_state_publisher` is pending.

The package started from Clearpath's
[kf/kingfisher](https://github.com/kf/kingfisher) @ `indigo-devel` `c7fb559`.

## Deliberately left on `noetic`

`kingfisher_node` (`kingfisher.py`, `sonarmite.py`, `trial.py`), the ROS 1
`kingfisher_drive_viz`, the rosbuild `kingfisher_bringup` tree
(`upstart/`, `vserial/`, the `.launch` files), `*.rosinstall`, `stack.xml`.
`kingfisher_nmea` was never more than a rosinstall file.
