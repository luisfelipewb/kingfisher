# kingfisher_localization

Localization for the real boat, on the same topics as the sim (VRX):

| topic (under `/kingfisher`) | type | frame |
|---|---|---|
| `sensors/imu/imu/data` | `sensor_msgs/Imu` | `kingfisher/imu_link`, orientation in ENU, rates and accelerations FLU |
| `sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | `kingfisher/gps_link`, RTK → `STATUS_GBAS_FIX` |
| `odom` | `nav_msgs/Odometry` | `odom` → `kingfisher/base_link`, twist in `base_link` |
| `sbg/*` | `sbg_driver/*` | `kingfisher/imu_link_ned`, the driver's raw NED output |

TF: `map → odom` is the identity (static), and `odom → kingfisher/base_link` comes with `odom`. `map`
is the ENU tangent plane at the site's datum, the same origin as the Gazebo world, not UTM. Each
point of the site file is published as `map → <name>`.

Without a valid SBG position (indoors), `odom` is still published: the position is held at the last
valid one (the datum if none), the velocity is 0, both with a variance of 1e6.

## Running

```bash
ros2 launch kingfisher_localization localization.launch.py
```

It runs separately from `kingfisher_bringup`, which publishes `base_link → imu_link` etc., so the tree
joins at `kingfisher/base_link` when both are up. Args: `namespace` (`kingfisher`), `frame_prefix`
(`kingfisher/`), `port` (`/dev/sbg`), `site` (`lac_symphonie`).

## Files

- `config/sbg.yaml`: the driver's parameters. `confWithRos: false`, so the device keeps its own
  stored configuration; the `log_*` values only choose which `sbg/*` topics are published.
- `config/localization.yaml`: `sbg_localization`, including the `imu_link` position in `base_link`
  (keep in sync with the URDF).
- `config/sites/<site>.yaml`: the datum (origin of `map`) and the site points.

## Dependencies

`sbg_driver` isn't in rosdep: clone
[sbg_ros2_driver](https://github.com/SBG-Systems/sbg_ros2_driver) next to `kingfisher` in the
workspace.
