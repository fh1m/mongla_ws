# ROS2 Conventions — Duburi AUV Codebase

> **Backend note.** The vehicle is the SROT board (firmware Hengla) + a Raspberry Pi 5 with a
> Hailo-8. `lock_heading`, `move_*_dist`, `arc` and `style_yaw` are **refused** there, `ALT_HOLD`
> is not one of its modes, and the depth-setpoint vision axes are refused. Where this page shows
> an older idiom, the current contract is [`command-reference.md`](../missions/command-reference.md) and the
> legacy path is [`legacy-pixhawk-and-sitl.md`](legacy-pixhawk-and-sitl.md).

ROS2 surface and coding standards for `mongla_ws`. The surface is
deliberately tiny: **one action, one telemetry topic, and a small set of
manager ROS params** (the manager declares 29 base params + a 10-param
`vision.*` tuning layer — see the table in §1, which used to list only 14 and
was missing `flight_controller`, the single most load-bearing param in the
codebase). If you're tempted to add a topic or service, reread this file and the
[architecture section of CLAUDE.md](../../../CLAUDE.md#4-software-architecture)
first.

> Earlier revisions of this file documented `/mongla/attitude`,
> `/mongla/depth_cmd`, `Attitude.msg`, `RCOverride.msg`, services like
> `/mongla/arm`, and packages like `mongla_driver` / `mongla_bringup`.
> **None of those exist** in this workspace. They were either
> aspirational or carried over from the 2023/2025 reference codebases.

---

## 1. Live ROS2 surface

### Action — `/mongla/move`

`mongla_interfaces/action/Move`, served by `auv_manager_node` in
`mongla_manager`. Every CLI verb, scripted mission, and external client
goes through this single endpoint.

| Goal field            | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `cmd` (str)           | Verb name — must match a key in `mongla_control.commands.COMMANDS` |
| `duration` (float)    | Seconds — used by `move_*`, `arc`, `pause`, `vision_*`           |
| `gain` (float)        | Percent thrust 0..100 — `move_*`/`arc`; `vision_*`: max-speed cap |
| `target` (float)      | Magnitude — degrees for `yaw_*`/`turn`, metres for `set_depth`   |
| `target_name` (str)   | String payload — mode name for `set_mode`                        |
| `timeout` (float)     | Per-command timeout (seconds)                                    |

> Above is the common core. `Move.action` carries ~52 goal fields total
> (style, DVL, the full `vision_align`/`vision_move` surface incl. the
> precision + mid-hold-fire knobs, and on `lock` the anchor fields). The
> authoritative per-verb field list lives in
> [`mongla_control/commands.py`](../../../src/mongla_control/mongla_control/commands.py);
> the action server, the `mongla` CLI, and the Python `MonglaClient` all read
> from that registry — there is no second list to keep in sync.

| Result field          | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `success` (bool)      | Did the maneuver complete? (vision verbs always `True`)          |
| `message` (str)       | Human-readable status — `'completed'`, `'NO_ACK'`, `'DENIED'`, ... |
| `final_value` (float) | Final reading on the moved axis (yaw °, depth m); vision: outcome code 0–4 |
| `error_value` (float) | Remaining error at exit (`|target − final|`); vision: worst residual px |
| `end_x_px` (float)    | vision: SIGNED target-from-centre px at last seen frame (`NaN`=never seen) |
| `end_y_px` (float)    | vision: SIGNED target-from-centre px at last seen frame (`NaN`=never seen) |
| `fill_frac` (float)   | vision_move: bbox fill fraction at exit [0..1] (0 for align)     |
| `elapsed_s` (float)   | vision: verb duration s (non-vision verbs leave 0)              |

| Feedback field        | Meaning                                                          |
|-----------------------|------------------------------------------------------------------|
| `phase` (str)         | e.g. `'EXECUTING'`, `'DONE'`                                     |
| `current_value` (float) | Current axis reading (depth m)                                |
| `error_value` (float) | Remaining error                                                 |
| `err_x_px` (float)    | vision: live SIGNED target-from-centre px (`NaN` otherwise)     |
| `err_y_px` (float)    | vision: live SIGNED target-from-centre px (`NaN` otherwise)     |
| `status_line` (str)   | Human-readable one-liner                                         |

**Rule:** add a new verb by adding **one row** to `COMMANDS` and **one
method** to `Mongla` (in `mongla_control/mongla.py`). The action server,
CLI, and Python client pick it up automatically — no other file edits
needed. Only widen `Move.action` if the existing field shape genuinely
isn't enough.

### Topic — `/mongla/state`

`mongla_interfaces/msg/MonglaState` (typed message, replaces the previous
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

> **Two default layers — don't confuse them.** The column below is the **node's
> own `declare_parameter` default** (what you get from a bare
> `ros2 run mongla_manager start`). The operator-facing **`bringup.launch.py`
> overrides some of these** for pool use (`mode:=pool` — and ONLY `mode`; verified
> by extraction 2026-09-08, `yaw_source` is `mavlink_ahrs` in both) — so a
> launched stack and a bare `ros2 run` can pick different profiles. `mode:=auto`
> probes the environment (UDP 14550 busy → `pool`; Pixhawk USB → `desk`; else `sim`).

| Param                   | Type   | Node default    | Notes                                                                        |
|-------------------------|--------|-----------------|------------------------------------------------------------------------------|
| `mode`                  | string | `auto`          | `auto`, `pool`, `sim`, `laptop`, `desk` (bringup.launch default: `pool`)     |
| `mav_device`            | string | `''`            | Override the connection string (e.g. `/dev/ttyACM0`, `udpin:0.0.0.0:14560`); `''` = use the mode profile |
| `smooth_yaw`            | bool   | `false`         | `true` → `yaw_glide` (smootherstep setpoint sweep)                           |
| `smooth_translate`      | bool   | `false`         | `true` → `drive_*_eased` (trapezoid thrust + settle-only brake)              |
| `yaw_source`            | string | `mavlink_ahrs`  | `mavlink_ahrs` \| `bno085` \| `bno085_dvl` \| `dvl` (bringup.launch default: `mavlink_ahrs`, same as the node — this used to say `dvl` and was wrong; the DVL is not fitted) |
| `bno085_port`           | string | `auto`          | `auto` = VID/PID scan (303a:1001); explicit path skips the scan              |
| `bno085_baud`           | int    | `115200`        | BNO085 stream baud rate                                                      |
| `payload_port`          | string | `auto`          | ESP32 payload (fire/drop) board; `auto` = VID/PID scan (1a86:7523)           |
| `nucleus_dvl_host`      | string | `192.168.2.201` | DVL TCP hostname                                                             |
| `nucleus_dvl_port`      | int    | `9000`          | DVL TCP port                                                                 |
| `nucleus_dvl_password`  | string | `nortek`        | DVL authentication password                                                  |
| `dvl_auto_connect`      | bool   | `true`          | Auto-connect DVL at startup (background retry loop)                          |
| `dvl_retry_s`           | float  | `5.0`           | Seconds between auto-connect retry attempts                                  |
| `debug`                 | bool   | `false`         | `true` → per-command `[MAV …]` tracing + DEBUG logging                       |
| `flight_controller`     | string | `srot`          | **The backend switch** — `srot` (default) or `pixhawk`. Gates `UNSUPPORTED_VERBS`, mode names, depth-axis refusal. |
| `allow_fw_behaviour_mismatch` | bool | `false`    | Override: arm even if the board reports below `FW_BEHAVIOUR_REV_REQUIRED`    |
| `srot_telemetry_period_s` | float | `2.0`         | srot telemetry poll period                                                   |
| `allow_saturated_depth_arm` | bool | `false`      | Override: arm even with a saturated depth reading                            |
| `position_source`       | string | `none`          | Localization input source                                                    |
| `baro_calibration`      | bool   | `true`          | Re-zero the barometer at startup                                             |
| `record`                | string | `''`            | Bag-record path/name, empty = off                                            |
| `vision_uplink_camera`  | string | `''`            | Camera whose detections uplink to `mongla_localization`                      |
| `vision_uplink_class`   | string | `''`            | Class filter for the vision uplink                                           |
| `vision_uplink_hz`      | float  | `25.0`          | Vision uplink rate                                                           |
| `vision_uplink_medium`  | string | `water`         | `water` \| `air` — selects the refraction/FOV correction applied to the uplink |
| `velocity_uplink`       | bool   | `false`         | Publish downward-camera optical-flow velocity to localization                |
| `position_uplink`       | bool   | `false`         | Publish estimated position to localization                                   |
| `payload_channels`      | string | `''`            | Board relay channels available to `fire()`                                   |
| `payload_fire_map`      | string | `''`            | Verb-name → channel mapping for `fire()`                                     |

Plus the `vision.*` tuning layer (10 params: `kp_lat/kp_yaw/kp_depth/kp_forward`,
`lost_grace_s`, `frame_fill_default`, `align_stable_frames`, `range_gain_floor`,
`ki_lat`, `ctrl_conf`) — see [`command-reference.md`](../missions/command-reference.md) §9 and
[`vision_tunables.py`](../../../src/mongla_manager/mongla_manager/vision_tunables.py).

`sensors_node` accepts a strict subset (`yaw_source`, `bno085_port`,
`bno085_baud`, plus `calibrate` bool, `mavlink_url`, `print_period_s`) for
diagnostic-only use.

---

## 2. Complete command reference

All verbs listed here are entries in `mongla_control/commands.py` and are available on the `/mongla/move` action, `mongla` CLI, and `MonglaMission` DSL.

### Motion

| Verb               | Key params                          | Notes                                               |
|--------------------|-------------------------------------|-----------------------------------------------------|
| `arm`              | `timeout`                           | Waits for ACK                                       |
| `disarm`           | `timeout`                           | Clears RC overrides, then disarms                   |
| `set_mode`         | `target_name` (str)                 | Mode name: **srot** `STABILIZE\|DEPTH_HOLD\|SURFACE\|MANUAL\|ACRO` (`ALT_HOLD` aliases to `DEPTH_HOLD`); **pixhawk** (legacy) `MANUAL\|ALT_HOLD\|STABILIZE\|...` |
| `stop`             | —                                   | **Safety**: active neutral RC (1500) on all channels |
| `surface`          | —                                   | **Safety**: ascend to 0 m; bypasses the command_active gate (runs mid-mission) |
| `mission_reset`    | —                                   | Stop heading lock + clear abort + RC neutral; **call at start of every `run()`** |
| `pause`            | `duration`                          | Release all RC overrides; autopilot takes over       |
| `head`             | —                                   | Read-only: returns current yaw in `final_value`     |
| `move_forward`     | `duration`, `gain`, `settle`        | Ch5 forward thrust, open-loop timed                 |
| `move_back`        | `duration`, `gain`, `settle`        | Ch5 reverse thrust, open-loop timed                 |
| `move_left`        | `duration`, `gain`, `settle`        | Ch6 lateral left, open-loop timed                   |
| `move_right`       | `duration`, `gain`, `settle`        | Ch6 lateral right, open-loop timed                  |
| `arc`              | `duration`, `gain`, `yaw_rate_pct`, `settle` | **Refused on srot** (`UNSUPPORTED_VERBS`) — Ch5 + Ch4 combined; curved trajectory; pixhawk/legacy only |
| `style_roll`       | `gain`, `timeout`, `flips`, `headroom` | N×360° roll in ACRO (surface-depth guarded)      |
| `style_yaw`        | `flips`, `deg_per_step`, `settle`   | **Refused on srot** (`UNSUPPORTED_VERBS`) — N×360° yaw spin in `ALT_HOLD`/`DEPTH_HOLD` (no mode change); pixhawk/legacy only |
| `yaw_left`         | `target` (deg), `timeout`, `settle` | Pivot left by N degrees                             |
| `yaw_right`        | `target` (deg), `timeout`, `settle` | Pivot right by N degrees                            |
| `turn`             | `target` (deg), `timeout`, `settle` | Rotate to **absolute** heading (0–360), direction auto |
| `set_depth`        | `target` (m neg), `timeout`, `settle` | **srot**: refused — the depth-setpoint axis is refused on this backend (root `CLAUDE.md` §2). **pixhawk** (legacy): engage `ALT_HOLD` + drive to depth. |
| `lock_heading`     | `target` (deg), `timeout`           | **Refused on srot** (`UNSUPPORTED_VERBS`) — 50 Hz background yaw lock; `target=0` = current; pixhawk/legacy only |
| `unlock_heading`   | —                                   | Cancels the heading lock thread                     |

### DVL (pool only — requires Nortek Nucleus 1000; **no DVL is fitted on the current vehicle,
### and none has ever been validated in water** — root `CLAUDE.md`)

> ⛔ The three `*_dist` verbs are in srot's `UNSUPPORTED_VERBS` and are **refused before
> dispatch** on the default backend. `dvl_connect` exists in code but talks to hardware that
> is not fitted. This whole section is pixhawk/legacy-backend and unvalidated-hardware
> reference, not something to run on the vehicle as configured today.

| Verb                 | Key params                             | Notes                                          |
|----------------------|----------------------------------------|------------------------------------------------|
| `dvl_connect`        | —                                      | Manual TCP connect (auto if `dvl_auto_connect:=true`); no DVL fitted |
| `move_forward_dist`  | `distance_m`, `gain`, `dvl_tolerance`, `settle` | **Refused on srot** — DVL closed-loop forward |
| `move_back_dist`     | `distance_m`, `gain`, `dvl_tolerance`, `settle` | **Refused on srot** — DVL closed-loop reverse |
| `move_lateral_dist`  | `distance_m`, `gain`, `dvl_tolerance`, `settle` | **Refused on srot** — DVL closed-loop lateral (+ve = right) |

Heading lock stays active during all `*_dist` moves — Ch4 holds heading while DVL drives Ch5/Ch6.

### Payload / depth / distance (host-side, srot-supported)

| Verb              | Key params  | Notes                                                                 |
|-------------------|-------------|------------------------------------------------------------------------|
| `calibrate_depth` | —           | Re-zero the barometer at the surface (QGC "Calibrate Pressure" equivalent), disarmed |
| `calc_distance`   | `phase`     | Downward optical-flow distance bracket: `phase='start'` latches, `phase='stop'` reports; experimental |
| `fire`            | `fire_channel` | Actuate a board relay channel (torpedo/dropper); `fire_channel` = board channel N, 1–16 |

### Vision (closed-loop, requires camera + detector running)

Exactly **two** pixel-native verbs (the 2026-06 rewrite replaced the old
9-verb axis API). Both ALWAYS return `success=True`; the align/move outcome
rides in `final_value` as an integer code (`0`=ALIGNED, `1`=LOST,
`2`=TIMEOUT, `3`=NO_CAMERA, `4`=ABORTED). `gain` is a hard max-speed cap.

| Verb           | DSL              | Key params                                                                                     | Notes                                                                                                              |
|----------------|------------------|-----------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------|
| `vision_align` | `vision.align()` | `camera`, `target_class`, `axes` (CSV of `lat,yaw,depth`), `offset_lat/yaw/depth`, `err_px`, `duration`, `gain` | Centre target on each active axis at its signed pixel offset (`0`=centre). Aligned when every axis is within `err_px` for `align_stable_frames` ticks. |
| `vision_move`  | `vision.move()`  | `camera`, `target_class`, `fwd_fill`, `mode` (`area`/`width`/`height`), `maintain_px`, `maintain_on`, `hold_s`, `err_px`, `duration`, `gain` | Drive forward until the bbox fills `fwd_fill`% of the frame. `maintain_px` holds a lateral offset; never re-centres yaw/depth. |

Beyond the core fields, `vision_align` also takes the lat/yaw caps
`gain_lat`/`gain_yaw` (depth has no `%` cap — its rate is `depth_step`, m/update),
the **forward range-hold** `fwd_fill`/`mode`/`kp_forward` (the standoff shot), the
arrival-brake (`brake_off`/`brake_gain`), the active station-keep `hold_s`,
**mid-hold fire** `fire_channels`/`fire_t` (+ `fire_pass_enabled` for a guaranteed
end-of-command shot), the fire-window quiet mode `hold_heading`, the **precision**
knobs `lock_target`/`ctrl_conf`/`range_gain_floor`/`ki_lat`/`settle_px`, and the
tuning fields `kp_lat`/`kp_yaw`/`kp_depth`/`lost_grace_s`/`align_stable_frames`/
`hold_through_loss`; `vision_move` also takes `gain_lat`, `brake_off`/`brake_gain`,
`range_gain_floor`, `kp_forward`/`kp_lat`/`lost_grace_s`/`hold_through_loss`. The control loop reads
`/mongla/vision/<cam>/detections` directly; the tracker's `/tracks` feeds
the HUD only (no `--tracking` flag). The standalone `fire` verb
(`fire_channel`: 1/2=torpedo, 3/4=dropper) actuates payloads — there is no
vision-fire verb.

---

## 3. Real package layout

```
mongla_ws/src/
├── mongla_interfaces/    # ROS2 message + action defs (Move.action, MonglaState.msg)
├── mongla_control/       # MAVLink layer + per-axis motion helpers + commands registry
├── mongla_manager/       # ROS2 node: ActionServer, telemetry, VisionState pool
├── mongla_sensors/       # YawSource abstraction (sensor-only, read-only)
├── mongla_vision/        # Camera factory, YOLO detector, draw overlays, tracker
└── mongla_planner/       # MonglaClient, MonglaMission DSL, mission scripts, CLI
```

There are **no** `mongla_driver`, `mongla_bringup`, `mongla_teleop`, or `mongla_mission`
packages. The 2023/2025 reference codebases had several of those names; that history is
captured in `legacy-pixhawk-and-sitl.md` for pattern reference, not for layout.

---

## 4. ROS2 node template

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
Packages:      mongla_<name>              mongla_control, mongla_sensors
Nodes:         mongla_<name>              auv_manager_node, sensors_node
Topics:        /mongla/<name>             /mongla/state
Actions:       /mongla/<verb>             /mongla/move
Messages:      PascalCase (file == name)  Move (action), MonglaState (msg)
Classes:       PascalCase                 Mongla, Pixhawk, BNO085Source, YawSource
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

## 8. QoS profiles

We don't currently use custom QoS — defaults work for the action and
the JSON state topic. If you add high-rate sensor topics later (e.g.
when `mongla_vision` arrives), use these:

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

## 9. Build commands

```bash
cd ~/Ros_workspaces/mongla_ws

# Full build (preferred — handles interface generation order)
./build_mongla.sh

# Faster: rebuild Python-only packages after editing source
colcon build --symlink-install --packages-select mongla_control mongla_manager mongla_sensors

# Single package
colcon build --symlink-install --packages-select mongla_manager

# Source after build
source install/setup.bash

# Lint a package
colcon test --packages-select mongla_manager
```

---

## 10. Logging style

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

## 11. What NOT to do

- Don't open a second `pymavlink` connection from another node. The
  manager owns the MAVLink reader; everything else uses the `Move`
  action.
- Don't add a "convenience" topic (`/mongla/depth_cmd`,
  `/mongla/heading_cmd`, etc) — the action covers it.
- Don't add a launch file with one node in it. `ros2 run ... --ros-args
  -p mode:=...` is the documented entry point.
- Don't introduce `std_srvs` services for arm/disarm — the action
  handles them via the `cmd` field.
- Don't add new QoS profiles unless you benchmarked them. Defaults work.
