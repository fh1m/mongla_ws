# Heading lock -- depth-hold's yaw cousin

> Companion to [`axis-isolation.md`](./axis-isolation.md),
> [`yaw-stability-and-fusion.md`](./yaw-stability-and-fusion.md),
> [`mavlink-references.md`](./mavlink-references.md).

## 1. The pitch in one sentence

`lock_heading` spawns a daemon thread that streams `SET_ATTITUDE_TARGET`
at 20 Hz with a fixed yaw, while motion commands switch from
`send_rc_override` to `send_rc_translation` so Ch4 stays released and
the attitude target keeps authority. It's `set_depth` for yaw -- engage
once, run a multi-command sequence, drift is corrected continuously by
ArduSub's onboard 400 Hz attitude stabiliser.

## 2. Why ArduSub's built-in heading hold isn't enough

ArduSub *already* holds heading in ALT_HOLD/POSHOLD/STABILIZE when Ch4
is at 1500 -- using its internal compass + AHRS yaw. Two physical
realities make that unreliable on a competition AUV:

1. **Magnetic interference.** Thruster ESCs draw 10A peaks; the field
   they produce dwarfs the geomagnetic vector. ArduSub's compass yaw
   walks. This is exactly why we have a BNO085 on the vehicle (see
   [`yaw-stability-and-fusion.md`](./yaw-stability-and-fusion.md)).
2. **Asymmetric thrust.** No vectored frame has all four thrusters
   exactly through CG, so every `move_forward` / `move_lateral`
   injects a small parasitic yaw moment. Without our own correction
   the bow weather-cocks several degrees per command.

`lock_heading` puts our chosen `yaw_source` in charge -- Pixhawk AHRS,
BNO085, or a future Gazebo / vision yaw -- and lets ArduSub's onboard
attitude stabiliser do the actual rate damping.

## 3. Source-agnostic by design

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

## 4. State diagram

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
  is a context manager that wraps any verb that intentionally changes
  heading (`yaw_left`, `yaw_right`, `arc`) or releases the override
  (`pause`). The lock thread stays alive, just stops streaming.
* **retarget** is called automatically after a yaw / arc exits, so
  the lock follows the most recently commanded heading without the
  operator having to call `unlock` + `lock_heading(new_target)`.

## 5. Interaction contract with motion commands

| Command            | Behaviour while lock active                     |
| ------------------ | ----------------------------------------------- |
| `move_forward`     | Uses `send_rc_translation` -- Ch4 released.     |
| `move_back`        | Same.                                           |
| `move_left/right`  | Same.                                           |
| `set_depth`        | `prime_alt_hold` uses `writers.neutral` so Ch4 stays released. |
| `yaw_left/right`   | Suspend lock, run yaw, retarget lock to new heading, resume. |
| `arc`              | Suspend lock, run arc, retarget lock, resume.   |
| `pause`            | Suspend lock for the duration, resume.          |
| `stop`             | Lock-aware neutral via `writers.neutral`.       |
| `unlock_heading`   | Stop the thread, send_neutral.                  |

The motion modules don't know about lock state -- they receive a
`Writers` from `motion_common.make_writers(..., release_yaw=...)` and
call `writers.forward(pwm)` / `writers.lateral(pwm)` / `writers.neutral()`.
The `release_yaw` flag picks `send_rc_translation` (Ch4 released) or
`send_rc_override` (Ch4 = 1500). Clean separation: lock is owned by
`Duburi`, axis writes are owned by the axis module.

## 6. Failure modes

| Failure                              | Behaviour                                                         |
| ------------------------------------ | ----------------------------------------------------------------- |
| Source returns `None` for a tick     | Last valid heading held; drift log shows '--'; loop never blocks. |
| Source dead > 2 s                    | `[LOCK ] WARN yaw source silent` log every 2 s; ArduSub's onboard stabiliser still tries to hold even without our drift visibility. |
| Operator forgets `unlock_heading`    | `timeout` (default 300 s) auto-stops the thread.                 |
| Manager process exits                | `daemon=True` kills the thread; manager's `finally` calls `pixhawk.send_neutral()` and stops the lock first. |
| User Ctrl+Cs the manager             | Same as above.                                                    |
| Two `lock_heading` calls in a row    | First lock is `stop()`'d, then a new one starts.                  |
| `lock_heading` called from non-yaw mode | `_ensure_yaw_capable_mode` engages ALT_HOLD first (or raises `ModeChangeError` if the mode change is rejected). |

## 7. Verification recipe

```bash
# Terminal 1 -- manager
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim -p yaw_source:=mavlink_ahrs

# Terminal 2 -- demo mission
ros2 run duburi_planner mission heading_lock_demo
```

Watch for these log lines:

* `[LOCK ] start  target=10.5deg  source=mavlink_ahrs  timeout=120s`
* `[LOCK ] tgt:  10.5  cur:  11.2  err: -0.7` (every 1 s)
* `[FWD  ] t=2.0s  drift=+0.1  depth=-0.50m` (drift stays small)
* `[LOCK ] retarget -> 55.0deg` after a `yaw_left 45`
* `[LOCK ] stopped` after `unlock_heading`

Bench test for source swap (verify BNO + Gazebo sync):

```bash
# Run identical mission against both sources and compare drift logs.
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim -p yaw_source:=mavlink_ahrs
ros2 run duburi_planner mission heading_lock_demo

# Reboot manager:
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim -p yaw_source:=bno085
ros2 run duburi_planner mission heading_lock_demo
```

Both should hold within +-2 deg over the same trajectory.

## 8. Community references

The design is informed by published RoboSub / BlueROV teams running
exactly this pattern -- our contribution is the source-agnostic
`YawSource` plug and the lock-aware `Writers` so motion commands
stack cleanly on top of the lock.

* [Blue Robotics forum: "Set Target Depth/Attitude" canonical example](https://discuss.bluerobotics.com/t/set-target-depth-and-set-target-attitude-in-pymavlink/12162)
  -- the SET_ATTITUDE_TARGET pattern we stream from the lock thread.
* [ArduSub source: heading hold in `mode_alt_hold.cpp`](https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/mode_althold.cpp)
  -- shows ArduSub's own onboard heading-hold runs at 400 Hz when Ch4
  is 1500. Our lock STREAMS the target so the controller has something
  to track instead of just "hold".
* [r/RoboSub thread on heading drift in pool runs](https://old.reddit.com/r/RoboSub/)
  -- consensus from Cornell, MIT, Caltech writeups: external IMU +
  attitude target + onboard PID is the canonical stack. Internal
  compass alone fails the moment the thrusters spin up.
* [YASMIN docs](https://uleroboticsgroup.github.io/yasmin/4.2.3/) --
  state-machine library we'll use to wrap `lock_heading` calls into
  composite states once the script-based missions outgrow linear
  sequences (see `duburi_planner/state_machines/README.md`).
