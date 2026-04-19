# Testing guide -- every layer, every command

> What to run, what to look for, and what to do when it goes wrong.
>
> Three test surfaces: **unit** (pure-Python, fast, no ROS),
> **bringup** (smoke checks against a live manager + simulated or
> real Pixhawk), and **field** (in-water mission rehearsal).

## 0. The single command for "is this build still good?"

```bash
cd ~/Ros_workspaces/duburi_ws
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select duburi_control duburi_vision \
                              duburi_planner duburi_manager
colcon test-result --verbose
```

What you should see if the workspace is healthy:

```
duburi_control:    52 passed
duburi_vision:     ?  passed (CUDA tests skip on CPU-only hosts)
duburi_planner:    1  passed (mission smoke discovers all *.py)
duburi_manager:    -  (no unit tests yet; bringup-only)
```

A red number from `duburi_control` is almost always a regression --
that suite covers the whole motion / lock / heartbeat / dispatch
stack and runs against `FakePixhawk` so it never needs hardware.

---

## 1. Unit tests (pure-Python, fast)

Every motion / lock / heartbeat module in `duburi_control` ships with
a `FakePixhawk`-driven test that exercises it without MAVLink.

| Suite                                        | What it tests                                                                                              | Touches                                |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------- | -------------------------------------- |
| [`test_commands.py`](../../src/duburi_control/test/test_commands.py)            | Every COMMANDS row maps to a real Move.Goal field; defaults are subset; `fields_for` substitutes correctly. | `commands.py`, `Move.action`           |
| [`test_motion_easing.py`](../../src/duburi_control/test/test_motion_easing.py)        | `smoothstep`, `smootherstep`, `trapezoid_ramp` math identities and edge cases (zero duration, etc.).        | `motion_easing.py`                     |
| [`test_motion_forward.py`](../../src/duburi_control/test/test_motion_forward.py)      | `drive_forward_*`, `arc` write the right Ch5/Ch4 PWM at the right cadence; brake-kick fires on `_constant`. | `motion_forward.py`, `motion_writers.py` |
| [`test_motion_lateral.py`](../../src/duburi_control/test/test_motion_lateral.py)      | `drive_lateral_*` writes Ch6 only; brake/settle behave.                                                    | `motion_lateral.py`                    |
| [`test_heading_lock.py`](../../src/duburi_control/test/test_heading_lock.py)          | Lock thread streams Ch4 at `LOCK_STREAM_HZ`, deadbands noise, releases Ch4 when source is dead, retargets cleanly. | `heading_lock.py`                      |
| [`test_heartbeat.py`](../../src/duburi_control/test/test_heartbeat.py)                | Heartbeat streams neutral at `HEARTBEAT_HZ`; pause/resume is reentrant; `hold()` context manager works.    | `heartbeat.py`                         |
| [`test_duburi.py`](../../src/duburi_control/test/test_duburi.py)                      | Facade serialises commands; `_command_ctx` pauses heartbeat; `lock_heading` + motion verbs cooperate cleanly. | `duburi.py`                            |
| [`test_pixhawk_helpers.py`](../../src/duburi_control/test/test_pixhawk_helpers.py)    | Pure-math helpers (`percent_to_pwm`, `heading_error`).                                                     | `pixhawk.py`                           |

Run a single one:

```bash
colcon test --packages-select duburi_control \
            --pytest-args -q test/test_heartbeat.py
```

Run only the `test_motion_forward` matching pattern:

```bash
colcon test --packages-select duburi_control \
            --pytest-args -q -k 'motion_forward'
```

Skip the known-pre-existing failures while you debug a new one:

```bash
colcon test --packages-select duburi_control \
            --pytest-args -q --deselect test/test_commands.py::test_every_field_is_on_move_goal
```

Vision unit tests:

| Suite                                        | What it tests                                                                            |
| -------------------------------------------- | ---------------------------------------------------------------------------------------- |
| [`test_detector.py`](../../src/duburi_vision/test/test_detector.py)   | YOLOv8 wrapper round-trips synthetic images.                                            |
| [`test_factory.py`](../../src/duburi_vision/test/test_factory.py)     | Backend factory picks the right detector class for a given config.                       |
| [`test_messages.py`](../../src/duburi_vision/test/test_messages.py)   | `Detection2DArray` round-trips through `VisionState`.                                    |
| [`test_gpu.py`](../../src/duburi_vision/test/test_gpu.py)             | CUDA path -- skipped on CPU-only hosts via `pytest.importorskip('torch.cuda')`.         |

Mission smoke (planner):

| Suite                                                       | What it tests                                                                                              |
| ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| [`test_missions_smoke.py`](../../src/duburi_planner/test/test_missions_smoke.py) | Every file in `missions/` (not starting with `_`) exposes `run(duburi, log)` and imports cleanly. Fails fast if a refactor breaks a mission script's imports. |

---

## 2. Per-command bringup tests (live manager, no water)

These run against a **live `auv_manager` node** -- either pointing at
the SITL Gazebo Pixhawk, or at a tethered real Pixhawk on the bench.
They are how you verify a verb actually puts the right MAVLink frames
on the wire.

### 2.1 Start the manager

Sim:

```bash
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim
```

Real Pixhawk over USB at /dev/ttyACM0:

```bash
ros2 run duburi_manager auv_manager --ros-args -p mode:=real
```

With BNO085 yaw + a peek at every MAVLink frame:

```bash
ros2 run duburi_manager auv_manager --ros-args \
    -p mode:=real -p yaw_source:=bno085 -p debug:=true
```

`debug:=true` flips two things at once:

1. The per-command MAVLink trace tag (`[MAV file:func cmd=verb] ...`)
   is enabled (off by default; production runs stay quiet).
2. The manager's logger is raised to DEBUG so the lines actually
   print. The startup banner ends with `DEBUG TRACE: ON`.

This is the recommended "did we send what we thought we sent?"
check before reaching for `tcpdump`.

#### Greppable session logs (the point of the cmd= tag)

Pipe the manager output to a file. Then for any verb:

```bash
rg "cmd=lock_heading" session.log | head      # every frame the verb produced
rg "pixhawk.py:set_target_depth" session.log  # every depth setpoint, regardless of verb
rg "cmd=yaw_right .* yaw=1[0-9]{3}"  session.log
                                              # every yaw frame the right-turn produced
```

Background daemons don't carry `cmd=` (contextvars don't cross
threads), but they keep `file:func`, so:

```bash
rg "pixhawk.py:send_rc_override\] RC_OVERRIDE.*yaw=14" session.log
                                              # every Ch4 frame the lock thread emitted at <1500
```

still works.

### 2.2 Bringup-check helper

`src/duburi_manager/duburi_manager/bringup_check.py` walks through
the most common startup failures (telemetry rate, mode mapping,
heartbeat present, AHRS2 fresh, BNO085 reachable) and prints a
GREEN/RED summary. Run it after the manager comes up:

```bash
ros2 run duburi_manager bringup_check
```

If anything goes RED, the suggested fix line in the output usually
tells you what to do. Common ones:

* **`AHRS2 stream rate is 4 Hz`** -- the `MESSAGE_RATES` startup
  loop didn't fire (manager started before MAVLink came up). Restart
  the manager; if it persists, check the `set_message_rate` log
  line at INFO level.
* **`BNO085 not seen on /dev/ttyACM0`** -- check the file is
  group-readable (`ls -l /dev/ttyACM0`); the docker user is in
  group `dialout`. See [`hardware-setup.md`](./hardware-setup.md).
* **`No autopilot HEARTBEAT in the last 3 s`** -- pymavlink is
  receiving our own loopback only. Check `bridge_endpoints` /
  BlueOS for the right routing.

### 2.3 Per-verb bench tests

The recipe is the same for every verb:

1. Start the manager (sim or real).
2. Run `bringup_check`.
3. Send the verb via the CLI -- one command at a time -- and watch
   the manager's log + `[MAV ]` trace.

Below is a copy-pasteable matrix. Every line assumes the manager is
already running in another terminal.

#### Power & mode

```bash
duburi arm
duburi set_mode --target_name ALT_HOLD
duburi set_mode --target_name MANUAL
duburi disarm
```

Expected `[MAV ]` lines with `debug:=true`:

```
[MAV pixhawk.py:arm cmd=arm]               ARM    cmd_long(COMPONENT_ARM_DISARM, p1=1)
[MAV pixhawk.py:set_mode cmd=set_mode]     SET_MODE custom_mode=2 (ALT_HOLD)
[MAV pixhawk.py:set_mode cmd=set_mode]     SET_MODE custom_mode=19 (MANUAL)
[MAV pixhawk.py:disarm cmd=disarm]         DISARM cmd_long(COMPONENT_ARM_DISARM, p1=0)
```

(`disarm` actually emits `set_mode MANUAL` -> `send_neutral` ->
`disarm` in that order; the `cmd=disarm` tag appears on ALL three
because the whole verb body runs under the `disarm` context.)

#### Stop / pause

```bash
duburi stop
duburi pause --duration 2.0
```

Expect `RC_OVERRIDE` (six 1500s) for `stop`, `RC_RELEASE` (six
65535s) for `pause`. After `pause` the heartbeat resumes -- you'll
see one `RC_OVERRIDE` per 200 ms again at DEBUG.

#### Translations

```bash
duburi move_forward --duration 3 --gain 50
duburi move_back    --duration 3 --gain 50
duburi move_left    --duration 3 --gain 50
duburi move_right   --duration 3 --gain 50
```

DEBUG: `RC_OVERRIDE` at 20 Hz with one of `fwd=` (Ch5) or `lat=`
(Ch6) deviating from 1500. In sim the AHRS2 telemetry shows the
position drift in the right axis.

#### Yaw

```bash
duburi yaw_left  --target 45
duburi yaw_right --target 45
```

DEBUG: `RC_OVERRIDE` with `yaw=` deviating from 1500 at 10 Hz
(`YAW_RATE_HZ`). Verify `final_value` in the response matches the
post-turn AHRS2 yaw.

#### Arc

```bash
duburi arc --duration 4 --gain 40 --yaw_rate_pct 25
```

DEBUG: `RC_OVERRIDE` writes `fwd=` AND `yaw=` in the same packet at
20 Hz. Single packet, both axes, every tick.

#### Depth

```bash
duburi set_depth --target -0.5
duburi set_depth --target -1.0
duburi set_depth --target  0.0
```

DEBUG (during the drive only):

```
[MAV pixhawk.py:set_mode cmd=set_depth]         SET_MODE custom_mode=2 (ALT_HOLD)        # if not already ALT_HOLD
[MAV pixhawk.py:set_target_depth cmd=set_depth] SET_POS_TGT depth=-0.50 m  (alt only, ...)  # at 5 Hz until we settle
```

After the verb returns, **no more `SET_POS_TGT` traffic** -- ALT_HOLD
holds depth on its own. The only MAVLink chatter from us is the 5 Hz
`Heartbeat` neutrals.

#### Heading lock

```bash
duburi lock_heading --target 0           # latch current heading
duburi move_forward --duration 5
duburi yaw_right --target 30             # lock auto-retargets
duburi unlock_heading
```

DEBUG: while the lock is active, you'll see `RC_OVERRIDE` with
`yaw=` close to 1500 (small corrections) at 20 Hz. The `Heartbeat`
goes silent during the lock and resumes after `unlock_heading`.

#### Vision

```bash
duburi vision_acquire     --target_class person --target_name yaw_right --timeout 20
duburi vision_align_yaw   --target_class person --duration 8
duburi vision_align_lat   --target_class person --duration 8
duburi vision_align_depth --target_class person --duration 8
duburi vision_hold_distance --target_class person --target_bbox_h_frac 0.55 --duration 12
duburi vision_align_3d    --target_class person --axes yaw,forward --duration 12
```

What to look at:

* INFO log line per loop tick from the manager: `[VIS  ]
  ex=+0.05 ey=-0.02 h=0.31 axes_in=YF settle=2/4 lost=0/12`.
* DEBUG `[MAV ]` shows the corresponding RC writes (one per tick at
  20 Hz).
* `final_value` in the response is the composite normalised error
  (lower = better-aligned); `error_value` is the age in seconds of
  the last detection at exit.
* If the camera frame is empty, expect `MoveFailed: ... lost ...`
  unless you set `--on_lost hold`.

The full vision pipeline can be sanity-checked end-to-end with
`duburi_vision/utils/check_pipeline.py` (synthesises a fake
detection stream so you can verify the manager's `VisionState` cache
+ control loop without needing a camera).

```bash
ros2 run duburi_vision check_pipeline
```

#### Servo PWM (payload)

There is no `duburi <verb>` for `set_servo_pwm` -- it's a Python
helper on `Pixhawk` directly, exposed for payload (torpedo, grabber,
dropper). Test it from the manager's REPL:

```bash
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim &
sleep 5
python3 -c "
import rclpy
from duburi_control import Pixhawk
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
m.wait_heartbeat()
p = Pixhawk(m)
p.set_servo_pwm(1, 1900)   # AUX1 full
p.set_servo_pwm(1, 1100)   # AUX1 reverse
p.set_servo_pwm(1, 1500)   # AUX1 neutral
"
```

DEBUG: `DO_SET_SERVO ch=9 (AUX1) pwm=1900` etc.

### 2.4 BNO sanity at the pool (3 checks, ~2 min)

Confirms `yaw_source:=bno085` is actually closing the Python control
loops -- *without* believing the misleading hint about needing to
physically rotate the chip. See
[`ardusub-canon.md`](./ardusub-canon.md) §4A for why STABILIZE-tilt
tests are NOT a BNO test.

Start the manager with the BNO source AND debug tracing on. Debug
flips per-command MAVLink trace tags on; default is off so production
runs stay quiet.

```bash
ros2 run duburi_manager auv_manager --ros-args \
    -p mode:=pool -p yaw_source:=bno085 -p debug:=true
```

The startup banner must end with `Yaw source: BNO085 (...) Earth-ref
offset: +XX.XX deg`. If you don't see the offset, the calibration
failed -- abort and fix.

**Check 1 -- telemetry shows BNO yaw.** Watch `[STATE]`:

```
[STATE] ARM | ALT_HOLD   | YAW: 124.3 (BNO085) | DEPTH: -0.42m | BAT: 15.6V
```

The `(BNO085)` label MUST be there. If it says `(AHRS)`, the yaw
source did not start and we silently fell back -- restart and read
the launch banner.

**Check 2 -- `lock_heading` closes on BNO.** From a second terminal:

```bash
duburi arm
duburi set_depth --target -0.4
duburi lock_heading --target 0           # latch current heading
```

In the manager log you will see `[LOCK ] start target=NNN.N
source=BNO085`. Now run a translation:

```bash
duburi move_forward --duration 4 --gain 50
```

While it runs, watch `[LOCK ] tgt:NNN.N cur:MMM.M err:+E.E pct:+P.P`
ticks. Both `tgt` and `cur` are in BNO degrees -- if you physically
rotate a fake target near the BNO cable, `cur` MUST move; if you
yank the BNO USB, the next tick MUST log `[LOCK ] yaw source silent
for X.Xs -- releasing Ch4` and the `[MAV pixhawk.py:send_rc_override
cmd=lock_heading]` lines for Ch4 must drop to 1500 us.

**Check 3 -- `yaw_left / yaw_right` close on BNO.** Still locked:

```bash
duburi yaw_right --target 30
```

The `[YAW ]` line and the `[STATE]` post-turn yaw must both end
within ~2 deg of `(start_yaw + 30) mod 360`, where `start_yaw` is
the **BNO** value from `[STATE]` before the command (not whatever
ArduSub's compass thinks). If they disagree, BNO is being sampled
but the loop is reading AHRS instead -- file a bug with the full
trace.

```bash
duburi unlock_heading
duburi disarm
```

End of recipe. If all three checks pass, BNO is genuinely the
closed-loop yaw source for the rest of the pool day.

---

## 3. Mission smoke + dry-run

Every mission file (`src/duburi_planner/duburi_planner/missions/*.py`)
is auto-discovered by the runner and is verified to import cleanly
by `test_missions_smoke.py`. To dry-run a mission against the SITL
manager:

```bash
# Terminal 1
ros2 run duburi_manager auv_manager --ros-args -p mode:=sim

# Terminal 2
ros2 run duburi_planner mission --list             # list discovered missions
ros2 run duburi_planner mission hello_world        # run one
```

If the mission fails partway, the runner prints the traceback and
ensures `disarm()` is called via the manager's atexit. To re-run
quickly, just re-issue the command -- the manager doesn't need a
restart.

---

## 4. Field test checklist (in-water)

Before you put the AUV in water:

* [ ] `colcon test` is GREEN (or only the documented pre-existing
      failure remains).
* [ ] `bringup_check` is GREEN against the *real* Pixhawk (not just
      sim).
* [ ] BNO085 yaw is reading sensible values
      (`ros2 topic echo /duburi/state | grep yaw_deg`).
* [ ] Tether is plugged in, FOX is forwarding to your deck topside.
* [ ] You have run **at least one** mission against SITL with the
      same parameters you intend to use in water.
* [ ] You have an emergency disarm shortcut wired
      (`duburi disarm` over the tether is the canonical answer; QGC
      kill-switch is a backup).

In water:

1. **Power-on test.** Manager up; bringup_check green; `arm` once;
   `disarm` once. No unintended thruster wiggles.
2. **Single-axis test.** Each of `move_forward`, `move_back`,
   `move_left`, `move_right`, `yaw_left`, `yaw_right`, `set_depth
   -0.3`, `set_depth 0` at the lowest gain you'll use (~30 %). Verify
   the AUV moves the right way.
3. **Lock smoke.** `lock_heading`, then a slow `move_forward 3 60`,
   then `unlock_heading`. Verify the bow stays within ~5 deg of the
   start heading. If not, tune `LOCK_KP_PCT_PER_DEG` lower (it's
   probably oscillating) or higher (it's probably losing the bow to
   thrust asymmetry).
4. **Mission rehearsal.** Run one of the cookbook missions
   (`hello_world`, `square_pattern`, etc.) at low gain. Watch for any
   verb that takes longer than expected -- usually a sign of a stuck
   detection or a depth setpoint that's just outside the deadband.

---

## 5. When something is wrong

The order of escalation:

1. **Check the manager's log** for ERROR / WARN. The most common
   cause is a typo in the goal (the dispatcher logs the rejected
   field name). Or `_ensure_yaw_capable_mode` failed to engage
   ALT_HOLD.
2. **Restart the manager with `debug:=true`** and re-run the verb.
   The `[MAV file:func cmd=verb]` trace tells you exactly what we
   sent AND which verb caused it. `rg "cmd=<verb>"` filters the
   session to one command. If the right message went out and the
   AUV still didn't move, the problem is downstream of us (ArduSub,
   ESCs, mode, parameters).
3. **Inspect ArduSub's `STATUSTEXT`** -- it's already mirrored
   into the manager's INFO log. ArduSub announces failsafes,
   pre-arm refusals, and EKF rejections via STATUSTEXT.
4. **Inspect ArduSub parameters via QGC.** `FS_PILOT_INPUT`,
   `FS_PILOT_TIMEOUT`, `PSC_*`, `ATC_*` are the usual suspects.
   See [`ardusub-canon.md`](./ardusub-canon.md) for the parameter
   map.
5. **Re-run the failing scenario in SITL** (`mode:=sim`). If it
   reproduces, the bug is in our code. If it doesn't, the bug is
   either a parameter on the real Pixhawk or a hardware fault
   (compass interference, ESC silence, etc.).

---

## 6. Cross-references

* Every verb's CLI / Python / DSL form: [`command-reference.md`](./command-reference.md)
* How to compose verbs into a mission: [`mission-cookbook.md`](./mission-cookbook.md)
* The MAVLink shapes the verbs end up emitting: [`mavlink-reference.md`](./mavlink-reference.md)
* ArduSub modes / failsafes / parameters: [`ardusub-canon.md`](./ardusub-canon.md)
* Hardware setup (camera, BNO085, Pixhawk wiring): [`hardware-setup.md`](./hardware-setup.md)
* Sim setup (Gazebo SITL + ArduSub): [`sim-setup.md`](./sim-setup.md)
