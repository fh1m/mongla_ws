# ROS2 Conventions — Duburi AUV Codebase

ROS2 surface and coding standards for `duburi_ws`. The surface is
deliberately tiny: **one action, one telemetry topic, six parameters**.
If you're tempted to add a topic or service, reread this file and the
[architecture section of CLAUDE.md](../../CLAUDE.md#4-software-architecture)
first.

> Earlier revisions of this file documented `/duburi/attitude`,
> `/duburi/depth_cmd`, `Attitude.msg`, `RCOverride.msg`, services like
> `/duburi/arm`, and packages like `duburi_driver` / `duburi_bringup`.
> **None of those exist** in this workspace. They were either
> aspirational or carried over from the 2023/2025 reference codebases.

---

## 1. Live ROS2 surface

### Action — `/duburi/move`

`duburi_interfaces/action/Move`, served by `auv_manager_node` in
`duburi_manager`. Every CLI verb, scripted mission, and external client
goes through this single endpoint.

| Goal field            | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `command_type` (str)  | Verb name — must match a key in `duburi_control.commands.COMMANDS` |
| `duration` (float)    | Seconds — used by `move_*`, `pause`                              |
| `gain` (float)        | Percent thrust 0..100 — used by `move_*`                         |
| `target` (float)      | Magnitude — degrees for `yaw_*`, metres for `set_depth`          |
| `target_name` (str)   | String payload — mode name for `set_mode`                        |
| `timeout` (float)     | Per-command timeout (seconds)                                    |

> The full set of verbs and their accepted fields lives in
> [`duburi_control/commands.py`](../../src/duburi_control/duburi_control/commands.py).
> The action server, the `duburi` CLI, and the Python `DuburiClient` all
> read from that registry — there is no second list to keep in sync.

| Result field          | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `success` (bool)      | Did the maneuver complete?                                       |
| `message` (str)       | Human-readable status — `'completed'`, `'NO_ACK'`, `'DENIED'`, ... |
| `final_value` (float) | Final reading on the moved axis (yaw °, depth m, depth m for linear) |
| `error_value` (float) | Remaining error at exit (`|target − final|`)                     |

| Feedback field        | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `current` (float)     | Current axis reading                                             |
| `target` (float)      | Goal axis reading                                                |
| `error` (float)       | `target - current`                                               |
| `state` (str)         | `'EXECUTING'` while running                                      |

**Rule:** add a new verb by adding **one row** to `COMMANDS` and **one
method** to `Duburi` (in `duburi_control/duburi.py`). The action server,
CLI, and Python client pick it up automatically — no other file edits
needed. Only widen `Move.action` if the existing field shape genuinely
isn't enough.

### Topic — `/duburi/state`

`duburi_interfaces/msg/DuburiState` (typed message, replaces the previous
JSON-in-`std_msgs/String` carrier). Published by the manager's telemetry
timer whenever the snapshot changes (or every ~1 s as a heartbeat).

```msg
std_msgs/Header header

bool    armed
string  mode              # '' if unknown
float32 yaw_deg           # NaN if unknown
float32 depth_m           # NaN if unknown
float32 battery_voltage   # NaN if unknown
```

Reliable, depth=1, KEEP_LAST. Late subscribers get the latest snapshot.

### ROS params on `auv_manager_node`

| Param           | Type   | Default        | Notes                                                         |
|-----------------|--------|----------------|---------------------------------------------------------------|
| `mode`          | string | `sim`          | One of `sim`, `pool`, `laptop`, `desk` (see `connection_config.PROFILES`) |
| `smooth_yaw`    | bool   | `false`        | `true` → `yaw_glide` (smootherstep setpoint sweep)            |
| `smooth_linear` | bool   | `false`        | `true` → `drive_eased` (trapezoid thrust + settle-only brake) |
| `yaw_source`    | string | `mavlink_ahrs` | `mavlink_ahrs` \| `bno085`                                    |
| `bno085_port`   | string | `/dev/ttyACM0` | USB CDC device path (only when `yaw_source==bno085`)          |
| `bno085_baud`   | int    | `115200`       | BNO085 stream baud rate                                       |

`sensors_node` accepts a strict subset (`yaw_source`, `bno085_port`,
`bno085_baud`, plus `calibrate` bool) for diagnostic-only use.

---

## 2. Real package layout

```
duburi_ws/src/
├── duburi_interfaces/    # action defs ONLY (Move.action)
├── duburi_control/       # MAVLink layer + per-axis movement helpers
├── duburi_manager/       # ROS2 node + ActionServer + CLI + connection profiles
└── duburi_sensors/       # YawSource abstraction (sensor-only, read-only)
```

There are **no** `duburi_driver`, `duburi_bringup`, `duburi_teleop`,
`duburi_mission`, or `duburi_vision` packages today. The 2023/2025
reference codebases had several of those names; that history is
captured in `proven-patterns.md` for pattern reference, not for layout.

---

## 3. ROS2 node template

Used by `auv_manager_node` and `sensors_node`. Use this shape for any
new node. Don't introduce launch files until we have at least three
nodes that need to come up together — `ros2 run` with `--ros-args -p`
covers the current usage.

```python
#!/usr/bin/env python3
"""One-line description of what this node does."""

import os
# Optional but useful: simplify console output before importing rclpy
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')

import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # --- Parameters ---
        self.declare_parameter('param_name', 'default_value')
        self._param = self.get_parameter('param_name').value

        # --- Publishers / subscribers / services / actions ---
        # Add only what you actually need. Avoid drive-by topics.

        # --- Timers ---
        self.create_timer(0.1, self._control_loop)   # 10 Hz

        self.get_logger().info(f'started, param={self._param}')

    def _control_loop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().debug(f'shutdown ignored: {e!r}')


if __name__ == '__main__':
    main()
```

---

## 4. `package.xml` template (Python ROS2 package)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>duburi_package_name</name>
  <version>0.1.0</version>
  <description>Brief, accurate description.</description>
  <maintainer email="duburi@example.com">BRACU Duburi</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>duburi_interfaces</depend>

  <buildtool_depend>ament_python</buildtool_depend>
  <test_depend>ament_pep8</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 5. `setup.py` template

```python
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'duburi_package_name'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BRACU Duburi',
    description='...',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node_name = package_name.module:main',
        ],
    },
)
```

> Real-world tip: register console scripts under both the short
> (`auv_manager`) and the explicit (`auv_manager_node`) names if both
> get used in docs / muscle memory. We do this in `duburi_manager`.

---

## 6. Naming conventions

```
Packages:      duburi_<name>              duburi_control, duburi_sensors
Nodes:         duburi_<name>              auv_manager_node, sensors_node
Topics:        /duburi/<name>             /duburi/state
Actions:       /duburi/<verb>             /duburi/move
Messages:      PascalCase (file == name)  Move (action), DuburiState (msg)
Classes:       PascalCase                 Duburi, Pixhawk, BNO085Source, YawSource
Functions:     snake_case                 send_rc_override, set_target_depth
Private:       _leading_underscore        _make_result, _ensure_yaw_capable_mode
Constants:     UPPER_SNAKE                YAW_RATE_HZ, SETTLE_SEC, NETWORK
```

> Convention shift (2026-04 cleanup): module-level constants and class
> constants drop the leading underscore — they're stable knobs the
> operator may want to read or override (`Pixhawk.AUX_MIN`,
> `motion_yaw.YAW_RATE_HZ`). The leading underscore is reserved for
> genuinely-internal helpers.

---

## 7. QoS profiles

We don't currently use custom QoS — defaults work for the action and
the JSON state topic. If you add high-rate sensor topics later (e.g.
when `duburi_vision` arrives), use these:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Telemetry (best-effort, low latency)
TELEMETRY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)

# Commands (reliable)
COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10)

# Latched state (reliable, transient-local for late subscribers)
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1)
```

---

## 8. Build commands

```bash
cd ~/Ros_workspaces/duburi_ws

# Full build (preferred — handles interface generation order)
./build_duburi.sh

# Faster: rebuild Python-only packages after editing source
colcon build --symlink-install --packages-select duburi_control duburi_manager duburi_sensors

# Single package
colcon build --symlink-install --packages-select duburi_manager

# Source after build
source install/setup.bash

# Lint a package
colcon test --packages-select duburi_manager
```

---

## 9. Logging style

```python
# Use the node logger, not print()
self.get_logger().debug('low-level detail (off by default)')
self.get_logger().info('normal operation')
self.get_logger().warn('unexpected but recoverable')
self.get_logger().error('needs attention')
self.get_logger().fatal('about to die')
```

Console output format is centralised at the top of `auv_manager_node.py`:

```python
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] {message}')
```

That strips the noisy timestamp + node-name prefix so the operator sees
clean `[STATE] ...` / `[YAW  ] ...` / `[ACT  ] ...` lines. Don't add
back the timestamp unless you're debugging a timing issue.

---

## 10. What NOT to do

- Don't open a second `pymavlink` connection from another node. The
  manager owns the MAVLink reader; everything else uses the `Move`
  action.
- Don't add a "convenience" topic (`/duburi/depth_cmd`,
  `/duburi/heading_cmd`, etc) — the action covers it.
- Don't add a launch file with one node in it. `ros2 run ... --ros-args
  -p mode:=...` is the documented entry point.
- Don't introduce `std_srvs` services for arm/disarm — the action
  handles them via `command_type`.
- Don't add new QoS profiles unless you benchmarked them. Defaults work.
