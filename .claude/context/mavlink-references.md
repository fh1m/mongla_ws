# MAVLink / ArduSub references + per-call audit

> Companion to [`mavlink-reference.md`](./mavlink-reference.md) (raw
> message reference) and [`ardusub-reference.md`](./ardusub-reference.md)
> (mode + parameter cheatsheet).

## 1. Authoritative references

### Specs

* [MAVLink common.xml](https://mavlink.io/en/messages/common.html)
  -- the canonical message catalogue. `RC_CHANNELS_OVERRIDE`,
  `SET_ATTITUDE_TARGET`, `SET_POSITION_TARGET_GLOBAL_INT`,
  `MAV_CMD_SET_MESSAGE_INTERVAL`, all here.
* [ArduSub developer docs](https://www.ardusub.com/developers/pymavlink.html)
  -- the Blue Robotics-recommended pymavlink patterns. Our
  `set_attitude_setpoint` and `set_target_depth` shapes come from this
  page directly.
* [ArduPilot source: ArduSub/](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub)
  -- ground truth for what each mode actually does. `mode_althold.cpp`,
  `mode_manual.cpp`, `mode_stabilize.cpp` are the relevant files.

### Community lessons that shaped our stack

* [Blue Robotics forum: "Depth Hold Problems?"](https://discuss.bluerobotics.com/t/depth-hold-problems/8108)
  -- the well-known stale-ALT_HOLD-integrator bug. Our
  `prime_alt_hold` (in `motion_depth.py`) drains it explicitly.
* [Blue Robotics forum: "Set Target Depth/Attitude in pymavlink"](https://discuss.bluerobotics.com/t/set-target-depth-and-set-target-attitude-in-pymavlink/12162)
  -- canonical example for both setpoint shapes. Our code mirrors it
  field-for-field including the typemask choice.
* [Blue Robotics forum: "RC override and modes"](https://discuss.bluerobotics.com/t/rc-overrides-on-companion-question/4911)
  -- explains the 65535 NO_OVERRIDE sentinel and the autopilot's "did
  the pilot disconnect?" timeout. Informs our `pause` semantics.
* [ArduSub Discord (#dev) and r/RoboSub](https://old.reddit.com/r/RoboSub/)
  -- repeated finding that internal compass yaw is unreliable under
  ESC interference. Cornell, MIT, Caltech, Auburn writeups all use an
  external IMU + SET_ATTITUDE_TARGET. Our `heading_lock.py` is the
  same pattern with a source-agnostic plug.
* [Hacker News: "How to write reliable embedded software"](https://news.ycombinator.com/item?id=27243405)
  -- general advice on stream-rate-pinning and watchdog patterns.
  Underwrites our explicit `MAV_CMD_SET_MESSAGE_INTERVAL` calls in
  `auv_manager_node.MESSAGE_RATES`.

## 2. Per-call audit

The table below covers every distinct MAVLink call we make from
`pixhawk.py`, the message ID it sends, what we use it for, and any
gotcha we've learned the hard way.

| `pixhawk.py` method            | MAVLink message               | Where used                     | Notes                                                                                                           |
| ------------------------------ | ----------------------------- | ------------------------------ | --------------------------------------------------------------------------------------------------------------- |
| `arm()` / `disarm()`           | `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM)` | `Duburi.arm`, `Duburi.disarm` | Wait for `COMMAND_ACK`; ArduSub *also* needs the heartbeat-arm bit before motors hot, hence we poll heartbeat too. |
| `set_mode(name)`               | `COMMAND_LONG (MAV_CMD_DO_SET_MODE)` | `Duburi.set_mode`, auto-engage in `set_depth` / yaw / lock_heading | Mode mapping comes from `master.mode_mapping()`, not hardcoded ints (ArduSub's IDs differ from ArduCopter). |
| `send_rc_override(**chans)`    | `RC_CHANNELS_OVERRIDE`        | constant/eased motion, `arc`   | ArduSub drops to RC failsafe after ~1 s of silence. We re-send at 20 Hz. 1500 = neutral, 65535 = no-override. |
| `send_rc_translation(**chans)` | `RC_CHANNELS_OVERRIDE`        | motion when heading-lock active | Sends 1500 on Ch3/Ch5/Ch6 (or commanded value), 65535 on Ch1/Ch2/Ch4. Lets `SET_ATTITUDE_TARGET` keep yaw authority. |
| `send_neutral()`               | `RC_CHANNELS_OVERRIDE`        | `Duburi.stop`, brake settles   | Six 1500s. Use `release_rc_override()` instead when you want ArduSub's automation to take over. |
| `release_rc_override()`        | `RC_CHANNELS_OVERRIDE`        | `Duburi.pause`, manager shutdown | Six 65535s. Pilot OFF the loop. ArduSub's mode automation (ALT_HOLD, etc.) runs unhindered. |
| `set_attitude_setpoint(yaw_deg=...)` | `SET_ATTITUDE_TARGET`   | `motion_yaw`, `heading_lock`   | Typemask ignores body rates + throttle. Quaternion is roll/pitch/yaw deg -> rad -> q. Stream >= 5 Hz or ArduSub falls back. |
| `set_target_depth(depth_m)`    | `SET_POSITION_TARGET_GLOBAL_INT` | `motion_depth`              | Typemask ignores everything but Z. `alt = depth_m`. Negative below surface. ALT_HOLD or GUIDED only. |
| `set_servo_pwm(aux_n, pwm)`    | `COMMAND_LONG (MAV_CMD_DO_SET_SERVO)` | (payload only)         | AUX1..AUX6 maps to servo channels 9..14. We add the +8 offset internally; calling with 1..8 would silently drive a thruster. |
| `set_message_rate(msg_id, hz)` | `COMMAND_LONG (MAV_CMD_SET_MESSAGE_INTERVAL)` | `auv_manager_node.MESSAGE_RATES` startup | Without this ArduSub picks defaults (4 Hz AHRS2). Pin to 50 Hz so our control loops aren't bottlenecked. |
| `send_heartbeat()`             | `HEARTBEAT`                   | `auv_manager_node` 0.5 Hz timer | Mandatory >= 1 Hz or ArduSub considers us disconnected and may drop overrides. |
| `clear_ack()` / `wait_ack()`   | (reads `COMMAND_ACK` cache)   | every ACK-bearing command      | Stale ACK from previous command would be a false positive -- always clear first. |

## 3. Things we found wrong / risky during this audit

### Fixed in this refactor

1. **`motion_depth.prime_alt_hold` was sending plain `pixhawk.send_neutral()`**
   -- when a `lock_heading` is active, that 1500 us on Ch4 would
   override `SET_ATTITUDE_TARGET` from the lock thread. **Fix:**
   `prime_alt_hold` now takes a `neutral_writer` callable and the
   `Duburi.set_depth` facade passes `writers.neutral` (lock-aware).
2. **Old `motion_linear.py` returned `pixhawk.send_rc_override(forward=..., lateral=...)` even when only one axis was active**
   -- that wrote 1500 to the *other* axis explicitly, blocking another
   command from reusing it concurrently and conflicting with future
   per-axis commands. **Fix:** new `motion_forward.py` and
   `motion_lateral.py` write only their own axis via the `Writers`
   bundle; the other translation channel naturally goes to 1500 via
   `send_rc_override`'s default kwargs.
3. **No way to combine forward thrust with yaw rate** -- meant every
   "turn" was a sharp pivot, no curved trajectories possible. **Fix:**
   new `arc(...)` writes Ch5 + Ch4 in the same `RC_CHANNELS_OVERRIDE`
   packet at 20 Hz.
4. **No persistent heading hold** -- ArduSub's internal compass-based
   hold is unreliable under ESC interference; we were re-issuing
   yaw_left/yaw_right between every move. **Fix:** new
   `heading_lock.py` daemon thread streams `SET_ATTITUDE_TARGET`
   continuously, source-agnostic over the `YawSource` interface.

### Known good but worth flagging

* **`MAVLINK20=1` env var is set inside `pixhawk.py` BEFORE pymavlink imports.**
  Forgetting this silently downgrades us to MAVLink v1, which lacks
  several typemask bits. Any new module that imports pymavlink
  outside pixhawk.py needs the same env shim.
* **`master.messages` cache vs. `recv_match()`.** Only the reader
  thread in `auv_manager_node.reader_loop` calls `recv_match`. Every
  other consumer reads `master.messages.get('AHRS2')` etc. from the
  cache. Mixing the two from worker threads causes pymavlink to drop
  messages -- consumers see stale data and we get apparent telemetry
  freezes mid-mission.
* **`COMMAND_ACK` collisions.** Two ACK-bearing commands fired in
  quick succession can land on the same cached ACK. We `clear_ack()`
  before every wait, but if a future module forgets, the first
  command's stale ACK will satisfy the second command's wait.

## 4. Hidden gems we deliberately don't (yet) use

| Feature                                | Why not (yet)                                                                                                                |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `MAV_CMD_GUIDED_CHANGE_SPEED`          | Speeds in m/s; we run open-loop in % thrust. Would require a velocity feedback source we don't have.                          |
| `SET_POSITION_TARGET_LOCAL_NED`        | Lateral/forward position setpoints in body frame. Useful if/when we have DVL or visual odometry providing position estimate.  |
| `STATUS_TEXT` severity filtering       | We log every STATUSTEXT at INFO. Severity-based routing (ERROR -> rclpy logger.error) would surface ArduSub failures louder.  |
| `MISSION_ITEM_INT` upload + AUTO mode  | ArduSub's onboard mission system. Worth exploring once we have a stable depth + heading sensor stack -- offloads the timing loop. |
| `MAV_CMD_DO_REPOSITION`                | Single-shot lat/lon move in GUIDED. Same blocker as POSITION_TARGET_LOCAL_NED -- needs position estimate.                     |
| `EKF_STATUS_REPORT`                    | Useful for surfacing ArduSub-side EKF rejections (compass disagrees with gyro etc.). Currently we infer this from `STATUSTEXT`. |

## 5. Where to look next

* `src/duburi_control/duburi_control/pixhawk.py` -- only file that
  imports pymavlink. All MAVLink shapes live here. Keep it that way.
* `.claude/context/ardusub-reference.md` -- mode/parameter notes,
  including the known ALT_HOLD I-term reset bug.
* `.claude/context/sensors-pipeline.md` -- yaw source contract and
  the 250 ms staleness budget.
* `.claude/context/heading-lock.md` -- the new heading-lock streamer.
