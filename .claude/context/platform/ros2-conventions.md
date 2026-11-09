# ROS 2 conventions — the surface and the coding standards

ROS 2 surface and coding standards for `mongla_ws`. The surface is deliberately tiny: **one
action, one state topic, and a short list of manager parameters**. If you are tempted to add a
topic or a service, reread this file and [CLAUDE.md §3](../../../CLAUDE.md#3-architecture)
first.

> Earlier revisions of this file documented `/mongla/attitude`, `/mongla/depth_cmd`,
> `Attitude.msg`, `RCOverride.msg`, services like `/mongla/arm`, and packages like
> `mongla_driver` / `mongla_bringup`. **None of those exist.**

The generated reference is the authority for every name and default below:
[`reference/commands.md`](../reference/commands.md), rebuilt by `python3 tools/gen_reference.py`
and checked by a test. This page says *why*; that one says *what*, and cannot drift.

---

## 1. The live surface

### Action — `/mongla/move`

`mongla_interfaces/action/Move`, served by `auv_manager_node`. Every CLI verb, mission and
external client goes through this one endpoint, one verb per goal.

- **Goal:** `cmd` names the verb; the other fields are the union of every verb's arguments.
  Which verb reads which field, and its default, is `mongla_control/commands.py` — the action
  server, the `mongla` CLI and `MonglaClient` all read that registry, so there is no second
  list. `set_fields` says which fields the sender set, so an explicit `0.0` is not mistaken for
  "unset" (issue #14).
- **Result:** `success`, `message`, `final_value`, `error_value`; the vision verbs add
  `end_x_px` / `end_y_px` (signed, `NaN` = never seen), `fill_frac` and `elapsed_s`, and carry
  their outcome (`ALIGNED`, `LOST`, `TIMEOUT`, `NO_CAMERA`, `ABORTED`, `DRIFTED`) as an integer in
  `final_value`.
- **Feedback:** `phase`, `current_value`, `error_value`, live `err_x_px` / `err_y_px`,
  `status_line`.

**Rule:** a new verb is **one row** in `COMMANDS` and **one method** of the same name on
`Mongla` (`mongla_control/mongla.py`). Only widen `Move.action` if no existing field fits, and
then a test fails until some verb reads the new field.

### Topic — `/mongla/state`

`mongla_interfaces/msg/MonglaState`, published by the manager whenever the snapshot changes.

```msg
std_msgs/Header header
bool    armed
string  mode              # the board's mode name, or '' if unknown
float32 yaw_deg           # NaN when there is no fresh attitude
float32 depth_m           # negative below the surface; NaN when absent
float32 battery_voltage   # NaN when absent
```

**Absence is `NaN`, never `0.0`** — a zero is a heading, a depth and a voltage, and every
guard downstream compares with `<` or `>`.

### Manager parameters

`mav_device` (empty = find the board on USB), `payload_channels` / `payload_fire_map`
(cosmetic names for board channels — they never route a shot), `velocity_uplink` /
`position_uplink`, the `vision_uplink_*` group, `record`, `debug`,
`srot_telemetry_period_s`, and three overrides that must be typed deliberately:
`allow_fw_behaviour_mismatch`, `allow_saturated_depth_arm` and
`allow_unknown_kill_switch` (the bench, with no second board). Plus the `vision.*` tuning
layer in [`vision_tunables.py`](../../../src/mongla_manager/mongla_manager/vision_tunables.py).
Types and defaults: [`reference/commands.md` §4](../reference/commands.md).

There is one board, one heading (the board's AHRS) and one link, so there is **no backend
selector** — no backend, heading-source or connection-mode parameter.
`test_launch_and_node_defaults_agree.py` fails if one comes back.

**Two default layers.** A bare `ros2 run mongla_manager start` gets the node's
`declare_parameter` defaults; `bringup.launch.py` declares its own, and
`~/.mongla/bringup.yaml` (or `$MONGLA_BRINGUP`) can override those. Anything typed as
`name:=value` still wins, and bringup prints which values came from the file.

---

## 2. The verbs

23 verbs: 11 run on the board as one `SROT_MOVE` each (`move_*`, `yaw_*`, `turn`,
`set_depth`, `stop`, `pause`, `style_roll`); 12 run on the host (`arm`, `disarm`, `set_mode`,
`head`, `surface`, `mission_reset`, `calibrate_depth`, `calc_distance`, `fire`, `motor_test`,
`vision_align`, `vision_move`). Fields, defaults and which side each runs on:
[`reference/commands.md` §1](../reference/commands.md); behaviour and the traps:
[`command-reference.md`](../missions/command-reference.md).

---

## 3. Package layout

```
mongla_ws/src/
├── mongla_interfaces/    # Move.action, MonglaState.msg, the TargetPose family
├── mongla_control/       # the verbs, the board driver (fc/), the vision loops — a library
├── mongla_manager/       # auv_manager_node: the action server, telemetry, the VisionState pool
├── mongla_vision/        # cameras, Hailo detection, the lock ladder, flow, optics, the HUD
├── mongla_localization/  # the right-invariant EKF, retrodiction, course priors, frames
└── mongla_planner/       # MonglaClient, the mission DSL, the missions, the `mongla` CLI
```

There are **no** `mongla_driver`, `mongla_bringup`, `mongla_teleop` or `mongla_mission`
packages, and `mongla_control` has **no entry points** — it is imported, never run.

---

## 4. ROS2 node template

The shape `auv_manager_node` and the vision nodes follow. Use it for any new node, and add it
to a launch file in the same change — a node nothing starts is unreachable.

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

## 5. `package.xml` template (Python ROS2 package)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mongla_package_name</name>
  <version>0.1.0</version>
  <description>Brief, accurate description.</description>
  <maintainer email="fh1m.dev@gmail.com">Muhammad Fahim Faisal</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>mongla_interfaces</depend>

  <buildtool_depend>ament_python</buildtool_depend>
  <test_depend>ament_pep8</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 6. `setup.py` template

```python
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mongla_package_name'

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
    maintainer='Muhammad Fahim Faisal',
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
> get used in docs / muscle memory. We do this in `mongla_manager`.

---

## 7. Naming conventions

```
Packages:      mongla_<name>              mongla_control, mongla_vision
Nodes:         mongla_<role>[_<camera>]   auv_manager_node, mongla_detector_forward
Topics:        /mongla/<name>             /mongla/state, /mongla/vision/forward/detections
Actions:       /mongla/<verb>             /mongla/move
Messages:      PascalCase (file == name)  Move (action), MonglaState (msg)
Classes:       PascalCase                 Mongla, SrotFC, VisionState
Functions:     snake_case                 send_heartbeat, stop_motion
Private:       _leading_underscore        _build_params, _command_scope
Constants:     UPPER_SNAKE                VISION_LOOP_HZ, FW_BEHAVIOUR_REV_REQUIRED
```

Module and class constants carry no leading underscore when they are knobs an operator may
read; the underscore is for genuinely internal helpers.

---

## 8. QoS — one table, both ends

**Warning: A RELIABLE subscriber against a BEST_EFFORT publisher receives nothing**, and rclpy says
so once, at construction. It happened twice on `image_raw`. So the vision links do not
construct their own profiles: they import [`mongla_vision/qos.py`](../../../src/mongla_vision/mongla_vision/qos.py),
and `test_qos_contract.py` checks every pairing *and* scans the source for a file that builds
its own `QoSProfile` for one of those topics. Images and the debug image are a BEST_EFFORT
mailbox (depth 1, the newest frame wins); detections are RELIABLE (depth 10); camera info and
latched state are RELIABLE + TRANSIENT_LOCAL (depth 1). Note `qos_profile_sensor_data` is not
the mailbox — it is a five-deep queue.

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

## 10. Build commands

```bash
cd ~/Ros_workspaces/mongla_ws
./build_mongla.sh                                     # the full build (interfaces first)
colcon build --symlink-install --packages-select mongla_manager   # one package
source install/setup.bash
```

---

## 11. What NOT to do

- Don't open a second `pymavlink` connection from another node. The manager owns the board;
  everything else uses the `Move` action. `PortGuard` refuses a second opener.
- Don't add a "convenience" topic (`/mongla/depth_cmd`, `/mongla/heading_cmd`, …) — the
  action covers it.
- Don't introduce `std_srvs` services for arm/disarm — the action handles them via `cmd`.
- Don't construct a `QoSProfile` for a vision topic outside `mongla_vision/qos.py`.
- Don't ship a node, a parameter or a topic that nothing starts, reads or subscribes to —
  `test_no_capability_is_built_and_unreachable.py` and the topic-wiring sweep both refuse it.

---

## Frames (2026-09-30)

**One owner: `mongla_localization/frames.py`.** Names, sensor offsets
(`frames.yaml`), rotations, and every TF edge live there; nothing else states
them.

```
map (ENU) -- pool (NED)                        static
 `- odom (ENU)                                 identity, ONLY once anchored
     `- odom_ned (NED)                         static
         `- mongla (FRD, at the IMU)           dynamic, localization
             |- base_link (FLU)                static, manager
             `- {downward,forward}_cam (FRD)   static, ONLY when measured
```

- **FRD/NED inside, REP-103 at the boundary.** Do not convert internally.
- **Depth:** `MonglaState.depth_m` is altitude (negative below the surface);
  `/mongla/odom` z and VISION_POSITION_ESTIMATE are NED (positive down). The one
  flip is `frames.ned_z_from_altitude`; `test_depth_sign.py` walks one depth
  through all three.
- **`odom_ned` jumps once, at the heading anchor** (the state is rotated). A
  consumer that integrates odom must treat that as a reset.
- **Not in the tree yet:** image frames (`forward` / `downward` from
  `camera_node`, and `lock_node`'s pixel-space outputs), because optical frames
  need the measured image→body signs; and `{cam}_cam` until the offsets are
  taped. An unmeasured edge is absent, never zero.

