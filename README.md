# kingfisher

ROS 2 stack for the Kingfisher ASV.

## Packages

| package | build type | what it does |
|---|---|---|
| `kingfisher_msgs` | ament_cmake | `Drive` and `Sense`, wire-compatible with the MCU |
| `kingfisher_rosserial` | — | the MCU bridge, see its own README |
| `kingfisher_twist` | ament_python | `cmd_vel` → `cmd_drive` |
| `kingfisher_viz` | ament_python | thrust arrows for RViz |
| `wifi_monitor` | ament_python | `has_wifi` link liveness |
| `kingfisher_bringup` | ament_python | what the boat actually runs |

## Running the boat

```bash
ros2 launch kingfisher_bringup robot.launch.py
```

`kingfisher_bringup` is self-contained: it declares nodes directly against
`kingfisher_bringup/config/kingfisher.yaml`. That one file is the boat's
configuration. Each package also keeps its own `launch/` and `config/` for
running a node on its own while debugging; those are not used in normal operation.



## Deliberately left on `noetic`

`kingfisher_node` (`kingfisher.py`, `sonarmite.py`, `trial.py`), the ROS 1
`kingfisher_drive_viz`, the rosbuild `kingfisher_bringup` tree
(`upstart/`, `vserial/`, the `.launch` files), `*.rosinstall`, `stack.xml`.
`kingfisher_nmea` was never more than a rosinstall file.
