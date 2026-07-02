# Heading lock -- Ch4 yaw-rate streamer

> Companion to [`axis-isolation.md`](./axis-isolation.md),
> [`yaw-stability-and-fusion.md`](./yaw-stability-and-fusion.md),
> [`mavlink-reference.md`](./mavlink-reference.md),
> [`ardusub-canon.md`](./ardusub-canon.md).

## 1. The pitch in one sentence

`lock_heading` spawns a daemon thread that runs a **proportional Ch4
yaw-rate** loop at 50 Hz against the heading reported by the
configured `YawSource`, while the rest of the motion verbs leave Ch4
neutral so the lock thread is the only writer on that axis. It is
the spiritual cousin of ArduSub's onboard `ALT_HOLD` for depth --
engage once at the start of a leg, stack any number of motion verbs
on top of it, and yaw drift stays bounded by the lock loop's
proportional gain.

## 2. Why ArduSub's built-in heading hold isn't enough

ArduSub *already* holds heading in ALT_HOLD/POSHOLD/STABILIZE when
Ch4 is at 1500 -- using its internal compass + AHRS yaw. Two
physical realities make that unreliable on a competition AUV:

1. **Magnetic interference.** Thruster ESCs draw 10A peaks; the
   field they produce dwarfs the geomagnetic vector. ArduSub's
   compass walks. This is exactly why we have a BNO085 on the
   vehicle (see [`yaw-stability-and-fusion.md`](./yaw-stability-and-fusion.md)).
2. **Asymmetric thrust.** No vectored frame has all four thrusters
   exactly through CG, so every `move_forward` / `move_lateral`
   injects a small parasitic yaw moment. Without our own correction
   the bow weather-cocks several degrees per command.

`lock_heading` puts our chosen `yaw_source` in charge -- Pixhawk
AHRS, BNO085, or a future Gazebo / vision yaw -- and lets ArduSub's
onboard 400 Hz attitude stabiliser do the actual rate damping when
it sees Ch4 != 1500.

## 3. How the loop works

Each tick at `LOCK_STREAM_HZ` (50 Hz, matched to the BNO085 firmware rate):

1. Read `current` from the `yaw_source` (or AHRS if no source given).
2. `error = Pixhawk.heading_error(target, current)`, in `[-180, 180]`.
3. Inside `LOCK_DEADBAND_DEG` -> command `0` (noise must not twitch the
   sub). Outside it, the law is `_lock_command(error)`:
   `mag = min(LOCK_PCT_MAX, |error| * LOCK_KP_PCT_PER_DEG)`, then
   `speed = max(_lock_floor(|error|), mag)`, signed by the error.
   `_lock_floor` is a **stiction-break floor that TAPERS to 0 at the
   deadband edge**: full `LOCK_SPEED_MIN_PCT` at/above
   `LOCK_APPROACH_BAND_DEG`, decaying linearly to 0 at `LOCK_DEADBAND_DEG`.
   A hard floor on a yaw *rate* channel is a relay that limit-cycles the
   hull when it has to reject a sustained disturbance (e.g. the lateral
   strafe of a `vision_align`) -- the taper kills that wobble while still
   breaking T200 stiction for real corrections (see
   [`known-issues.md`](./known-issues.md) D7). **The cure for align-yaw
   jitter is this taper, NOT releasing the lock** -- releasing it hands
   yaw to ArduSub's untrusted hull compass.
4. `pixhawk.send_rc_yaw_only(percent_to_pwm(yaw_pct))` -- writes **only
   Ch4** (Ch5/Ch6/Ch3 stay at NO_OVERRIDE 65535) so a concurrent DVL
   forward/lateral move's Ch5/Ch6 thrust is never clobbered by a lock
   tick. ArduSub treats Ch4 != 1500 as a pilot yaw-rate command, so the
   Python-side `yaw_source` is the sole feedback closing the loop.

**Fire-window quiet mode (2026-07-01, D12).** The active deadband is a
per-instance field (`self._deadband_deg`, default `LOCK_DEADBAND_DEG`),
toggled by `HeadingLock.set_hold_mode(on)`. A terminal `vision_align`
that releases yaw to the lock (the torpedo hole-lock: `lat`/`depth` only)
and passes `hold_heading=True` calls `set_hold_mode(True)` for the
duration of the call (try/finally restores it), widening the deadband to
`LOCK_HOLD_DEADBAND_DEG` (3.0°). This makes the lock **hold a steady
launcher heading** instead of chasing sub-degree BNO / thrust-coupling
noise -- the residual terminal yaw jitter -- while the torpedo fires.
`_lock_command`/`_lock_floor` take the active deadband as a param so they
stay pure/testable; the default path (setter never called) is unchanged.

There is no `SET_ATTITUDE_TARGET` involved -- that approach was
considered and dropped because the rate-override path is simpler,
matches what ArduSub does for joystick pilots, and avoids stream-
rate failsafes specific to attitude setpoints.

Tunables live at the top of
[`heading_lock.py`](../../src/duburi_control/duburi_control/heading_lock.py):

```python
LOCK_KP_PCT_PER_DEG    = 1.2     # proportional gain (% thrust per deg)
LOCK_SPEED_MIN_PCT     = 5.0     # stiction-break floor (tapered, see below)
LOCK_PCT_MAX           = 22.5    # clamp; matches yaw_snap max
LOCK_DEADBAND_DEG      = 1.0     # inside this -> command 0 (normal)
LOCK_HOLD_DEADBAND_DEG = 3.0     # widened deadband in fire-window quiet mode (set_hold_mode)
LOCK_APPROACH_BAND_DEG = 6.0     # floor tapers full->0 across deadband..band
DRIFT_LOG_SEC          = 1.0     # [LOCK ] heartbeat cadence
SOURCE_DEAD_S          = 0.5     # release Ch4 after this many silent seconds
```

## 4. Source-agnostic by design

`HeadingLock` accepts ANY `duburi_sensors.YawSource` (or `None` to
fall back to AHRS). The same code runs against:

* `mavlink_ahrs` -- Pixhawk's AHRS2 yaw (works in Gazebo SITL, on
  the bench, when BNO085 isn't wired)
* `bno085`       -- external 9-DOF IMU (preferred on the real sub)
* future Gazebo / DVL / vision yaw -> drop-in `YawSource` subclass

This means a pool team can rehearse heading-lock missions in Gazebo
with `mavlink_ahrs`, then switch to `bno085` for the actual run with
`--ros-args -p yaw_source:=bno085`. Zero code change.

It also makes it trivial to validate sensor sync: run with
`yaw_source:=mavlink_ahrs` in SITL, then `yaw_source:=bno085` on the
real sub, and compare the `[LOCK ] cur:` log lines -- they should
agree to within the BNO085 - AHRS calibration offset.

## 5. State diagram

```
                        +---------------+
                        | NO LOCK       |
                        | (default)     |
                        +-------+-------+
                                |
                lock_heading(target,timeout)
                                |
                                v
                        +---------------+
              start --> | LOCK ACTIVE   | <-- retarget(new_deg)
                        | thread runs   |        (yaw_left, yaw_right, arc)
                        +---+---------+-+
                            |         ^
              suspend()     |         |  resume()
              (yaw, arc,    |         |
              pause)        v         |
                        +---------------+
                        | LOCK SUSPENDED|
                        +-------+-------+
                                |
                       (block ends; resume)
                                |
                                v
                        +---------------+
                        | LOCK ACTIVE   |
                        +-------+-------+
                                |
                  unlock_heading() OR timeout
                                |
                                v
                        +---------------+
                        | NO LOCK       |
                        +---------------+
```

* **suspend / resume** is internal -- `Duburi._suspend_heading_lock()`
  is a context manager that wraps any verb that intentionally
  changes heading (`yaw_left`, `yaw_right`, `arc`, and `vision_align`
  when `yaw` is an align axis) or releases the override (`pause`).
  The lock thread stays alive, just stops streaming.
* **retarget** is called automatically after a yaw / arc / yaw-axis
  `vision_align` exits, so the lock follows the most recently commanded
  heading without the operator having to call `unlock` +
  `lock_heading(new_target)`.
* **timeout / disarm exit** -- the lock can also end *without*
  `unlock_heading`. On `timeout` the lock thread fires an `on_exit`
  callback (`Duburi._on_lock_timeout`) that releases Ch4, clears
  `Duburi._heading_lock`, and releases the heartbeat hold, so a
  timed-out lock can never linger as an "active" zombie that still
  pauses the heartbeat and makes translation verbs release Ch4 to a
  dead thread. `disarm()` stops and joins an active lock (and releases
  the heartbeat) before dropping to MANUAL. Both paths land in
  `NO LOCK` exactly like `unlock_heading`.

## 6. Cooperation with `Heartbeat`

The [`Heartbeat`](../../src/duburi_control/duburi_control/heartbeat.py)
daemon (5 Hz neutral RC override; prevents `FS_PILOT_INPUT`) and
the heading lock both write to `RC_CHANNELS_OVERRIDE`. To prevent
them stepping on each other:

* While `lock_heading` is active, `Heartbeat` is **paused** (the
  Duburi facade calls `self._heartbeat.pause()` inside
  `lock_heading`). The heartbeat sends a *full neutral* 6-channel
  override including `Ch4=1500`, which would fight the lock's Ch4
  stream -- so it is paused and the lock becomes the sole Ch4 writer.
  The lock's own Ch4 packet at 50 Hz (20 ms) keeps `FS_PILOT_INPUT`
  from triggering on its own.
* `unlock_heading` resumes the Heartbeat so the wire stays warm
  during whatever runs next. The same heartbeat hold is released on
  the two other lock-exit paths — a `timeout` (via the `on_exit`
  callback) and `disarm()` — so the Heartbeat is never left paused
  after a lock ends.

The lock uses `send_rc_yaw_only` (writes **only Ch4**, all other
channels at NO_OVERRIDE 65535) rather than a full 6-channel
`send_rc_override` -- so a concurrent DVL `move_*_dist` can drive
Ch5/Ch6 through its own `send_rc_translation` while the lock holds
Ch4, with neither writer clobbering the other's channels.

## 7. Interaction contract with motion commands

| Command            | Behaviour while lock active                                      |
| ------------------ | ---------------------------------------------------------------- |
| `move_forward`     | `motion_forward.drive_forward_*` writes Ch5 only; lock owns Ch4. |
| `move_back`        | Same.                                                            |
| `move_left/right`  | `motion_lateral.drive_lateral_*` writes Ch6 only.                |
| `set_depth`        | Sends `SET_POSITION_TARGET_GLOBAL_INT` (depth axis) -- doesn't touch RC. Lock keeps yaw. |
| `yaw_left/right`   | Suspend lock, run yaw, retarget lock to new heading, resume.    |
| `arc`              | Suspend lock, run arc, retarget lock, resume.                   |
| `pause`            | Suspend lock for the duration, resume.                           |
| `stop`             | Suspends lock briefly via `_command_scope`, send_neutral, resume. |
| `vision_align`     | If `yaw` IS an align axis: suspend lock, loop drives Ch4, retarget lock on exit. If `yaw` is NOT an axis but a lock is live: `release_yaw` — loop writes lateral via `send_rc_translation`, Ch4 left entirely to the lock (never writes `yaw=1500` to fight it). |
| `vision_move`      | Never commands yaw; with a lock active it drives forward/lateral via `send_rc_translation` (`release_yaw`), so the lock keeps Ch4. |
| `unlock_heading`   | Stop the thread, send_neutral, resume Heartbeat.                |
| `disarm`           | Stops/joins the lock + releases the Heartbeat hold, *then* disarms — the lock's Ch4 yaw-rate stream must not outlive the drop to MANUAL. |

The motion modules don't know about lock state -- they receive a
`Writers` from `motion_writers.make_writers(..., release_yaw=...)`
and call `writers.forward(pwm)` / `writers.lateral(pwm)` /
`writers.neutral()`. The `release_yaw` flag picks
`send_rc_translation` (Ch4 released to 65535 so the lock thread's
override wins the slot) or `send_rc_override` (Ch4 = 1500). Clean
separation: lock is owned by `Duburi`, axis writes are owned by
the axis module.

## 8. Failure modes

| Failure                              | Behaviour                                                         |
| ------------------------------------ | ----------------------------------------------------------------- |
| Source returns `None` for a tick     | Last valid heading held; loop never blocks.                       |
| Source dead > `SOURCE_DEAD_S` (0.5 s) | `[LOCK ] WARN yaw source silent` log; loop releases Ch4 (1500) so the sub does not yaw on stale data; recovers automatically (with an INFO log) when samples resume. |
| Operator forgets `unlock_heading`    | `timeout` (default 300 s) fires the `on_exit` callback: Ch4 released, `Duburi._heading_lock` cleared, Heartbeat hold released — so a timed-out lock can't linger as an "active" zombie. |
| `disarm()` while a lock is active    | `disarm()` stops/joins the lock and releases the Heartbeat hold before dropping to MANUAL, so the Ch4 yaw-rate stream stops cleanly. FSM `DisarmState` also calls `release_heading()` first. |
| Manager process exits                | `daemon=True` kills the thread; manager's `finally` calls `pixhawk.send_neutral()` and stops the lock first. |
| User Ctrl+Cs the manager             | Same as above.                                                    |
| Two `lock_heading` calls in a row    | First lock is `stop()`'d (Heartbeat hold released), then a new one starts. |
| `lock_heading` called from non-yaw mode | `_ensure_yaw_capable_mode` engages ALT_HOLD first (or raises `ModeChangeError` if the mode change is rejected). |

## 9. Verification recipe

```bash
# Terminal 1 -- manager
ros2 run duburi_manager start --ros-args -p mode:=sim -p yaw_source:=mavlink_ahrs

# Terminal 2 -- demo mission
ros2 run duburi_planner mission heading_lock_demo
```

Watch for these log lines:

* `[LOCK ] start  target=10.5deg  source=mavlink_ahrs  timeout=120s`
* `[LOCK ] tgt:  10.5  cur:  11.2  err: -0.7` (every 1 s)
* `[FWD  ] t=2.0s  drift=+0.1  depth=-0.50m` (drift stays small)
* `[LOCK ] retarget -> 55.0deg` after a `yaw_left 45`
* `[LOCK ] stopped` after `unlock_heading`

To inspect the per-frame MAVLink trace, restart the manager with
`debug:=true` (it raises the logger to DEBUG and enables the
per-command tag) -- you'll see one `[MAV send_rc_yaw_only] yaw=1543`
line for every Ch4 write the lock makes (no `cmd=` because the lock
runs in its own thread and contextvars don't cross thread boundaries):

```bash
ros2 run duburi_manager start --ros-args -p mode:=sim -p debug:=true
```

Bench test for source swap (verify BNO + Gazebo sync):

```bash
# Run identical mission against both sources and compare drift logs.
ros2 run duburi_manager start --ros-args -p mode:=sim -p yaw_source:=mavlink_ahrs
ros2 run duburi_planner mission heading_lock_demo

# Reboot manager:
ros2 run duburi_manager start --ros-args -p mode:=sim -p yaw_source:=bno085
ros2 run duburi_planner mission heading_lock_demo
```

Both should hold within +/-2 deg over the same trajectory.

## 10. Community references

The design is informed by published RoboSub / BlueROV teams running
exactly this pattern -- our contribution is the source-agnostic
`YawSource` plug and the lock-aware `Writers` so motion commands
stack cleanly on top of the lock.

* [ArduSub source: heading hold in `mode_alt_hold.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/mode_althold.cpp)
  -- shows ArduSub's own onboard heading-hold runs at 400 Hz when
  Ch4 is 1500. Our lock instead drives Ch4 at small angles to
  command yaw rate, which ArduSub's stabiliser turns into actual
  yaw via the four-thruster mixer.
* [r/RoboSub thread on heading drift in pool runs](https://old.reddit.com/r/RoboSub/)
  -- consensus from Cornell, MIT, Caltech writeups: external IMU +
  attitude target + onboard PID is the canonical stack. Internal
  compass alone fails the moment the thrusters spin up.
* [YASMIN docs](https://uleroboticsgroup.github.io/yasmin/4.2.3/) --
  state-machine library we'll use to wrap `lock_heading` calls into
  composite states once the script-based missions outgrow linear
  sequences (see `duburi_planner/state_machines/README.md`).
