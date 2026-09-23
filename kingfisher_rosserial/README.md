# kingfisher_rosserial

The bridge between the boat's PC and the Kingfisher MCU.

## The package names shadow upstream ROS packages. That is deliberate.

`rosserial_msgs` and `rosserial_python` here are **not** the upstream ROS
packages of those names. The MCU is closed-source and speaks a very old
rosserial protocol; these are that protocol ported to Jazzy.

The wire type strings in `serial_node.py`'s `known_types` table
(`kingfisher_msgs/Drive`, `rosserial_msgs/Log`, ...) are what the firmware
sends. They cannot change without new firmware. Do not "fix" the naming.

## Contents

| package | built | what it is |
|---|---|---|
| `rosserial_msgs` | yes | `Log`, `TopicInfo`, `RequestParam` as rosidl |
| `rosserial_python` | yes | the bridge node, `serial_node` |
| `rosserial_client` | **no** | firmware-side `ros_lib`, reference only |

`rosserial_client` is rosbuild C++ and carries a `COLCON_IGNORE`. It is the
headers the MCU itself compiles against, and therefore the only
documentation of the wire protocol. Two things it settles:

- `node_handle.h:311,321` — the advertised `topic_name` is the literal
  string the firmware passed to `ros::Publisher`/`Subscriber`. Whether the
  MCU's topics are absolute or relative is a property of firmware we cannot
  read, so confirm with `ros2 topic list` before namespacing anything.
- `node_handle.h:333-354` — the firmware can publish `ID_LOG`.

## Known bugs, not yet fixed

`serial_node.py` has unguarded crash paths.
The bridge dies on the first log packet the firmware sends. Launch files
set `respawn=True`, which is a guard, not a fix.
