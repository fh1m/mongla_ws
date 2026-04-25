# Python client + mission DSL API

> Three layers, three audiences, one underlying action.
>
> 1. **`DuburiClient`** -- raw blocking action client. ROS2-aware,
>    feedback-aware, raises typed exceptions. Lives in
>    [`src/duburi_planner/duburi_planner/client.py`](../../src/duburi_planner/duburi_planner/client.py).
> 2. **`DuburiMission`** (the "DSL") -- thin friendly wrapper over the
>    client; what you call from inside `def run(duburi, log)`. Lives
>    in [`src/duburi_planner/duburi_planner/duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py).
> 3. **`Duburi`** facade -- the manager-side serialised dispatcher
>    that *implements* the verbs (owns the lock, the heartbeat, the
>    pixhawk, etc.). Lives in
>    [`src/duburi_control/duburi_control/duburi.py`](../../src/duburi_control/duburi_control/duburi.py).
>    Mission code does NOT instantiate this directly.
>
> Layer 3 is what receives `/duburi/move` goals and runs the actual
> motion. Layers 1 and 2 are what mission code uses to talk to it.
> Every verb in [`command-reference.md`](./command-reference.md) is
> reachable from all three.

---

## 1. `DuburiClient` -- the raw action surface

```python
from duburi_planner.client import DuburiClient, MoveRejected, MoveFailed
import rclpy

rclpy.init()
node   = rclpy.create_node('mission_runner')
client = DuburiClient(node)
client.wait_for_connection(timeout=15.0)            # blocks until server up

result = client.send('move_forward', duration=5.0, gain=60.0)
print(result.success, result.message,
      result.final_value, result.error_value)
```

### Method surface

| Method                       | Behaviour                                                                              |
| ---------------------------- | -------------------------------------------------------------------------------------- |
| `wait_for_connection(timeout=15.0)` | Blocks until the `/duburi/move` server is reachable. Raises `TimeoutError`. |
| `send(cmd, **fields)`        | Sends one goal, blocks until completion. Returns `Move.Result`. Raises `MoveRejected` (server refused goal) or `MoveFailed` (server accepted but `result.success` was False). |
| `client.<cmd>(**fields)`     | Sugar -- equivalent to `send(cmd, **fields)`. Validated against the `COMMANDS` registry: `client.mvoe_forward(...)` raises `AttributeError`, NOT a silent send of an unknown verb. |

### `Move.Goal` fields the client accepts

The `**fields` kwargs map directly onto `duburi_interfaces.action.Move.Goal`.
Unset fields stay at their rosidl defaults (0.0 / '' / False) -- the
manager-side dispatcher then substitutes per-command defaults from
the `COMMANDS` registry, or live `vision.*` ROS-param values for
vision verbs.

| Field                | Type     | Used by                               |
| -------------------- | -------- | ------------------------------------- |
| `duration`           | float32  | move_*, arc, pause, vision_*          |
| `gain`               | float32  | move_*, arc, vision_acquire           |
| `target`             | float32  | set_depth (m) / yaw_* (deg) / lock_heading (deg) |
| `target_name`        | string   | set_mode, vision_acquire (sweep verb) |
| `timeout`            | float32  | every command; defaults vary          |
| `settle`             | float32  | post-command neutral hold; default 0  |
| `yaw_rate_pct`       | float32  | arc, vision_acquire('arc')            |
| `camera`             | string   | vision_*                              |
| `target_class`       | string   | vision_*                              |
| `axes`               | string   | vision_align_3d (CSV)                 |
| `deadband`           | float32  | vision_align_*                        |
| `kp_yaw`             | float32  | vision_align_yaw / _3d                |
| `kp_lat`             | float32  | vision_align_lat / _3d                |
| `kp_depth`           | float32  | vision_align_depth / _3d              |
| `kp_forward`         | float32  | vision_hold_distance / _3d            |
| `target_bbox_h_frac` | float32  | vision_hold_distance / _3d            |
| `visual_pid`         | bool     | vision_align_3d (placeholder for v2)  |
| `on_lost`            | string   | vision_*  ('fail' or 'hold')          |
| `stale_after`        | float32  | vision_*                              |
| `depth_anchor_frac`  | float32  | vision_align_depth / vision_align_3d  |
| `lock_mode`          | string   | vision_* ('settle' / 'follow' / 'pursue') |
| `distance_metric`    | string   | vision_hold_distance / vision_align_3d |
| `tracking`           | bool     | all vision_* — `True` subscribes `/tracks` (ByteTrack IDs + Kalman-smoothed), requires tracker_node |

### `Move.Result` fields you get back

| Field           | Meaning                                                                          |
| --------------- | -------------------------------------------------------------------------------- |
| `success`       | True iff the verb completed without error                                        |
| `message`       | Human-readable outcome / failure reason                                          |
| `final_value`   | Axis-correct: yaw deg for yaw verbs, depth m for depth verbs, etc. (see `Move.action` for the full mapping) |
| `error_value`   | Axis-correct error: heading error deg / depth error m / age-of-last-detection s  |

### Exceptions

| Exception        | When                                                            |
| ---------------- | --------------------------------------------------------------- |
| `MoveRejected`   | Server refused the goal (e.g. another command already active, or the action server is down). |
| `MoveFailed`     | Server accepted the goal but `result.success` was False (timeout, mode rejected, motion exception). |
| `TimeoutError`   | `wait_for_connection` couldn't reach the server.                |
| `ValueError`     | `send(...)` called with a `cmd` not in the `COMMANDS` registry. |

### Why blocking?

Missions are scripts. Each line is one MAVLink command that runs to
completion before the next line starts. There is no parallelism
*inside* a mission -- it's a straight line, top to bottom. Daemon
threads (`HeadingLock`, `Heartbeat`) handle the few things that
*do* run in parallel; the mission-author surface stays sequential.

---

## 2. `DuburiMission` -- the mission DSL

This is what every `missions/<name>.py` file actually receives:

```python
def run(duburi, log):       # `duburi` is a DuburiMission instance
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)
    duburi.vision.find(sweep='right', timeout=20)
    duburi.vision.lock(axes='yaw,forward', distance=0.55, duration=12)
    duburi.disarm()
```

Internally `DuburiMission` does three things:

1. **Wraps a `DuburiClient`** -- every verb funnels through
   `self._send(cmd, **fields)` which calls `client.send(cmd, **fields)`.
2. **Logs every outcome** -- `_format_outcome(cmd, result)` produces
   one human-readable line per verb (`[OK ] move_forward 5.0 s
   gain=60 final=-0.50 m`) so missions don't need any logging
   boilerplate.
3. **Holds sticky context** -- `self.camera = 'laptop'` and
   `self.target = 'person'` are read by every `duburi.vision.*` call
   that doesn't specify them explicitly.

### Constructor

```python
DuburiMission(client, log, *, camera='laptop', target='person')
```

You don't usually call this -- the runner
([`src/duburi_planner/duburi_planner/runner.py`](../../src/duburi_planner/duburi_planner/runner.py))
constructs it for you and hands it to your `run(duburi, log)`.

### Top-level verbs (`duburi.*`)

Every verb that exists in [`command-reference.md`](./command-reference.md)
is exposed as a method on `DuburiMission`. The signature uses
human-friendly kwarg names where they differ from the action field
names:

```python
duburi.arm(timeout=15.0)
duburi.disarm(timeout=20.0)
duburi.set_mode('ALT_HOLD', timeout=8.0)

duburi.stop()
duburi.pause(seconds=2.0)

duburi.set_depth(meters=-1.5, timeout=30.0)

duburi.move_forward(seconds=5.0, gain=80.0, settle=0.0)
duburi.move_back   (seconds=5.0, gain=80.0)
duburi.move_left   (seconds=5.0, gain=80.0)
duburi.move_right  (seconds=5.0, gain=80.0)

duburi.yaw_left (degrees=90.0, timeout=30.0)
duburi.yaw_right(degrees=90.0, timeout=30.0)

duburi.arc(seconds=5.0, gain=50.0, yaw_rate_pct=30.0)

duburi.lock_heading(degrees=0.0, timeout=300.0)   # returns immediately
duburi.release_heading()                          # joins the daemon

# DVL closed-loop distance (heading lock stays active during these)
duburi.dvl_connect()                              # manual connect; auto-connect is default
duburi.move_forward_dist(meters=2.0, gain=60.0, dvl_tolerance=0.1)
duburi.move_lateral_dist(meters=1.0, gain=36.0, dvl_tolerance=0.1)
# Negative meters = reverse / left:
duburi.move_forward_dist(-1.5, gain=60.0)         # 1.5 m backward
duburi.move_lateral_dist(-0.5, gain=36.0)         # 0.5 m left
```

**DVL distance gotchas:**
- Heading lock is NOT suspended during `move_forward_dist` / `move_lateral_dist`.
  The lock owns Ch4 (yaw rate) and keeps the AUV on heading while DVL owns Ch5/Ch6.
  Call `lock_heading()` BEFORE the distance moves.
- Falls back to open-loop time estimate (with a warning) if the yaw source has no
  DVL component (i.e. `yaw_source=mavlink_ahrs` or `yaw_source=bno085`).
- `dvl_connect()` is only needed if `dvl_auto_connect:=false`. The default is
  `dvl_auto_connect:=true` (auto-connect at manager startup).

### Vision verbs (`duburi.vision.*`)

`duburi.vision` is a `_VisionDSL` sub-namespace bound to the parent
mission's sticky `camera`/`target`. Verbs:

```python
duburi.vision.find    (target=None, sweep='right',
                       timeout=25.0, gain=25.0,
                       yaw_rate_pct=22.0, stale_after=0.0)

duburi.vision.yaw     (target=None, duration=8.0,  **overrides)
duburi.vision.lateral (target=None, duration=8.0,  **overrides)
duburi.vision.depth   (target=None, duration=8.0,  **overrides)
duburi.vision.forward (target=None, distance=0.55, duration=12.0, **overrides)

duburi.vision.lock    (target=None, axes='yaw,forward',
                       distance=0.55, duration=15.0, **overrides)

duburi.vision.follow  (target=None, axes='yaw,forward',
                       distance=0.55, duration=60.0, **overrides)
# Convenience wrapper for lock(..., lock_mode='follow').
# Never exits on settle — tracks the target until `duration` expires
# or the target is lost. Use for a swimmer / diver tracking window.
```

`sweep` mapping (in `_FIND_SWEEP_DRIVERS`):

| `sweep`     | Underlying `target_name` (drive while waiting) |
| ----------- | ---------------------------------------------- |
| `'right'`   | `'yaw_right'`                                  |
| `'left'`    | `'yaw_left'`                                   |
| `'forward'` | `'move_forward'`                               |
| `'arc'`     | `'arc'`                                        |
| `'still'`   | `''` (no drive -- wait in place)               |
| `'none'`    | `''`                                           |

`**overrides` are any of `kp_yaw`, `kp_lat`, `kp_depth`, `kp_forward`,
`deadband`, `target_bbox_h_frac`, `stale_after`, `on_lost`,
`depth_anchor_frac`, `lock_mode`, `distance_metric`, `visual_pid`,
**`tracking`** (bool, default `False` — set `True` to use ByteTrack stable IDs + Kalman-smoothed bbox).

Only pass them when you want to *pin* a value for that one call — omit
them and the live `vision.*` ROS-param value applies.

### Enabling tracking via DSL

```python
# Option A: per-call
duburi.vision.lock(target='gate', axes='yaw,forward',
                   distance=0.45, duration=20.0,
                   tracking=True)

# Option B: global ROS param (applies to all subsequent vision goals)
import subprocess
subprocess.run(['ros2', 'param', 'set', '/duburi_manager',
                'vision.use_tracks', 'true'])

# Option C: launch flag (cameras_.launch.py handles tracker_node startup)
#   ros2 launch duburi_vision cameras_.launch.py with_tracking:=true
```

When `tracking=True`, the manager subscribes `/tracks` (published by `tracker_node`) instead
of `/detections`. The `Sample.track_id` field is populated with the stable ByteTrack integer ID.
Short occlusions (up to `max_predict_frames`, default 5 frames = ~0.25 s) are bridged by
Kalman prediction — the control loop never sees a gap.

### Escape hatch -- raw `send`

```python
duburi.send('whatever_new_verb', some_field=1.2, other_field='x')
```

Anything not defined as an explicit verb falls through `__getattr__`
to the underlying `DuburiClient.send` and STILL gets the one-line
outcome log. That means any future row in the `COMMANDS` registry
is reachable as `duburi.<cmd>(...)` even before someone wires it
explicitly above.

### Outcome logging format

Every successful verb prints one line via the mission `log`:

```
[OK ] move_forward 5.0 s gain=60.0  final=-0.50 m  err=0.000
[OK ] yaw_right    90 deg            final=125.4 deg  err=-0.4
[OK ] vision.lock  axes=yaw,forward  final=0.07       err=0.30 s
```

Failures raise `MoveFailed` (the runner prints the traceback at the
top level, so missions don't need a try/except unless they want to
recover and continue).

---

## 3. `Duburi` -- the manager-side facade (do not instantiate)

This is the **implementation** of the verbs. The `auv_manager_node`
ActionServer constructs one instance per process, owns it for the
lifetime of the manager, and dispatches incoming `/duburi/move`
goals through it.

### What it owns

| Resource          | Why                                                                  |
| ----------------- | -------------------------------------------------------------------- |
| `Pixhawk`         | The MAVLink connection. Single instance per process.                 |
| `HeadingLock`     | Daemon thread for the Ch4 yaw-rate streamer.                         |
| `Heartbeat`       | Daemon thread for the 5 Hz neutral RC override (FS_PILOT_INPUT guard). |
| `vision_state_provider` callable | Returns the `VisionState` cache instance for vision verbs. |
| `threading.Lock`  | Serialises every command -- AT MOST ONE motion command runs at a time. |

### What you'd touch only when extending

Adding a new verb is a 4-line change:

1. Add a row to `COMMANDS` in `commands.py` (defines fields + defaults).
2. Add a method on `Duburi` (open-loop) or `VisionVerbs` (vision)
   whose name matches the row's key.
3. (optional) Add a friendlier wrapper to `DuburiMission` if the
   default kwarg naming is awkward.
4. (optional) Add a one-line CLI entry in
   `src/duburi_planner/duburi_planner/cli.py` if you want a
   command-line affordance distinct from `duburi <cmd>`.

The dispatch already auto-discovers any new `COMMANDS` row -- the
client and CLI just work without further edits.

### Command-internal context manager

Inside `Duburi`, every command method runs under
`with self._command_scope(verb):`, which:

1. Acquires `self.lock` (serialises commands).
2. Opens a `tracing.command_scope(verb)` so every MAVLink frame the
   verb emits carries `cmd=<verb>` in its `[MAV ]` DEBUG line when
   the manager is started with `debug:=true`.
3. Pauses `self._heartbeat` (so its 5 Hz neutral writes don't race
   the command's own RC writes).
4. On exit: resumes the heartbeat (unless a different command path
   already did, e.g. `lock_heading` keeps it paused for the lifetime
   of the lock).

Vision verbs in `VisionVerbs` use the same `_command_scope()`. There
is no branch in user code that needs to know about either the lock,
the tracing tag, or the heartbeat -- this is the contract that makes
the verbs composable.

---

## 4. Cross-references

* CLI driver (the `duburi` shell command): [`cli.py`](../../src/duburi_planner/duburi_planner/cli.py)
* Mission runner (auto-discovers `missions/*.py`): [`runner.py`](../../src/duburi_planner/duburi_planner/runner.py)
* Action server dispatch (verifies field shape, hands to `Duburi`): [`auv_manager_node.py`](../../src/duburi_manager/duburi_manager/auv_manager_node.py)
* All verbs (CLI + Python + DSL forms in one table): [`command-reference.md`](./command-reference.md)
* Mission cookbook (composing verbs): [`mission-cookbook.md`](./mission-cookbook.md)
* Testing every layer end-to-end: [`testing-guide.md`](./testing-guide.md)
