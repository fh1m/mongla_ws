# Known Issues — tracked from the 2026-04 audit

This file lists concrete code bugs the audit found. Every entry has:

- **File + line range** so the next code sprint can jump straight in.
- **Symptom** — what's wrong / why it matters.
- **Fix sketch** — minimum change to close it. Not a final design.

When picking these up: convert one entry at a time into a focused PR;
don't bundle. Each one is ~10–40 LOC.

> **Status (2026-04, post-audit sprint):** Every issue in this file is
> **FIXED**. Issues **#1–#3** landed in the `known_issues_critical`
> sprint; **#4–#9** in the follow-up sprint the same day. See the
> inline resolution notes below for files touched and how to verify.

> **File / class renames (2026-04 cleanup, after these fixes landed):** the
> entries below name files and classes as they existed at fix time. The
> follow-up "clean code" pass renamed them — when reading entries below,
> mentally substitute:
>
> | Old name (in entries below)                   | Current name (in code today)                  |
> |-----------------------------------------------|-----------------------------------------------|
> | `mavlink_api.py` / `MavlinkAPI`               | `pixhawk.py` / `Pixhawk`                      |
> | `movement_commands.py` / `MovementCommands`   | `duburi.py` / `Duburi`                        |
> | `movement_yaw.py` (`yaw_step` / `yaw_ramp`)   | `motion_yaw.py` (`yaw_snap` / `yaw_glide`)    |
> | `movement_linear.py` (`linear_step` / `linear_ramp`) | `motion_linear.py` (`drive_constant` / `drive_eased`) |
> | `movement_depth.py` (`depth_hold`)            | `motion_depth.py` (`hold_depth`)              |
> | `movement_pids.py` (`DepthPID` / `YawPID`)    | **deleted** — ArduSub onboard PID is the only loop |
> | `_AUX_*` constants on `MavlinkAPI`            | `AUX_*` on `Pixhawk` (no leading underscore)  |

---

## 1. Action result lies on yaw / depth failure — **FIXED 2026-04**

- **File:** `src/duburi_manager/duburi_manager/auv_manager_node.py`, `src/duburi_control/duburi_control/movement_yaw.py`, `src/duburi_control/duburi_control/movement_depth.py`
- **Resolution:** Added `duburi_control.errors.MovementTimeout`. `yaw_step`, `yaw_ramp`, and `depth_hold` now raise it on timeout instead of logging-and-returning. The action server's existing `except Exception` block converts it into `Move.Result.success = False` + abort + a descriptive `result.message`, and now also populates `result.final_value` (current depth) on the failure path. A safety net additionally calls `MavlinkAPI.send_neutral()` so a stale `SET_ATTITUDE_TARGET` / `SET_POSITION_TARGET` does not survive into the next command.
- **Verify:** drive `ros2 run duburi_manager duburi yaw_right 90 --timeout 1` (deliberately too short); the action result must be `success=False message='yaw_right: exception — yaw_right timeout after 1.0s — cur=… tgt=… err=…'` and the next command must start from neutral.

## 2. `set_mode` failures continue silently — **FIXED 2026-04**

- **File:** `src/duburi_control/duburi_control/movement_commands.py` (`_ensure_yaw_capable_mode`, `set_depth`)
- **Resolution:** Both call sites now raise `duburi_control.errors.ModeChangeError` when `MavlinkAPI.set_mode('ALT_HOLD')` returns `(False, …)`. The action server catches it like any other movement failure (see issue #1) and the operator sees an explicit `[YAW] ✗ set_mode ALT_HOLD failed: <reason>` line plus an aborted action result.
- **Verify:** force a denial by overriding `mode_mapping()` in a unit test, or run against a SITL where ALT_HOLD is disabled; the action must report `REJECTED` rather than spinning the sub on a stale rate command.

## 3. `MavlinkAhrsSource` has no staleness gate — **FIXED 2026-04**

- **File:** `src/duburi_sensors/duburi_sensors/sources/mavlink_ahrs.py`, `src/duburi_control/duburi_control/mavlink_api.py`
- **Resolution:** Added `MavlinkAPI.get_attitude_age() -> float | None` which uses pymavlink's per-message `_timestamp` to compute seconds-since-receipt of the last `AHRS2`. `MavlinkAhrsSource.read_yaw()` and `is_healthy()` now both gate on `age <= 0.25 s`, returning `None` / `False` when stale. Matches the [`sensors-pipeline.md`](./sensors-pipeline.md) §"Stale handling" contract.
- **Verify:** kill the SITL while the manager is running; `is_healthy()` should flip to `False` within ≈ 250 ms and yaw commands should stop locking onto the cached yaw.

## 4. `set_servo_pwm` missing AUX +8 offset — **FIXED 2026-04**

- **File:** `src/duburi_control/duburi_control/mavlink_api.py`
- **Resolution:** `set_servo_pwm(aux_n, pwm)` now takes an AUX index (1..6, i.e. the AUX1..AUX6 silkscreen), validates the range with a `ValueError`, clamps `pwm` to `1100..1900`, and adds the `+8` offset internally before sending `MAV_CMD_DO_SET_SERVO`. The constants `_AUX_PWM_OFFSET=8`, `_AUX_MIN=1`, `_AUX_MAX=6` make the mapping inspectable. No callers existed yet, so this is preventative — payload code (torpedo / grabber / dropper) can land safely.
- **Verify:** `MavlinkAPI(None).set_servo_pwm(7, 1500)` raises `ValueError: aux_n must be 1..6 ...` (confirmed in smoke test).

## 5. `Move.action` field semantics are inconsistent — **FIXED 2026-04**

- **Files:** `src/duburi_interfaces/action/Move.action`, `src/duburi_control/duburi_control/movement_commands.py`, `src/duburi_manager/duburi_manager/auv_manager_node.py`
- **Resolution:** `Move.action` now spells out the per-command contract for `final_value` / `error_value` in comments. `MovementCommands.set_depth`, `move_*`, `yaw_*`, and `stop` all return `(final_value, error_value)`; the action server's `_execute_cb` populates the result fields straight from that tuple. Per-command axes:
  - `yaw_*` → `final_value = final yaw [°]`, `error_value = signed heading error [°]`
  - `set_depth` → `final_value = final depth [m]`, `error_value = |target − final| [m]`
  - `move_*` / `stop` → `final_value = current depth [m]`, `error_value = 0.0` (no DVL odometry yet)
  - `arm` / `disarm` / `set_mode` → `final_value = current depth [m]`, `error_value = 0.0` (no axis to report)
- **Verify:** `Move.Result()` accepts `error_value = 0.25` (confirmed in smoke test) and `auv_manager_node` no longer hard-codes depth in the success path.

## 6. `test_runner.py` exits 0 on mission failure — **FIXED 2026-04**

- **File:** `src/duburi_manager/duburi_manager/test_runner.py`
- **Resolution:** Mission body extracted into `_run_mission()`; `main()` wraps it in try/except, sets `exit_code = 1` on any `Exception`, and `sys.exit(exit_code)` after teardown. The ~10 lines of commented-out alternate mission steps are deleted, the docstring now matches the actual square-pattern mission, and the file's policy line ("if a step is disabled, delete it") is encoded in the docstring.
- **Verify:** `ros2 run duburi_manager test_runner` against an unreachable action server prints `MISSION FAILED` and exits with `$? == 1`.

## 7. Silent `except Exception: pass` on shutdown — **FIXED 2026-04**

- **Files:** `src/duburi_manager/duburi_manager/auv_manager_node.py`, `src/duburi_sensors/duburi_sensors/sensors_node.py`, `src/duburi_sensors/duburi_sensors/sources/bno085.py`
- **Resolution:** Every `except Exception: pass` on a shutdown path now logs the swallowed exception at **debug** level (so steady-state stays quiet, but `--ros-args --log-level debug` reveals the cause of an ugly exit). `sensors_node.py` uses a stdlib `logging.getLogger('duburi_sensors.sensors_node')` for the post-`destroy_node()` `rclpy.shutdown()` path because the ROS logger is no longer guaranteed to be alive there.
- **Verify:** `colcon build` + run; on `Ctrl-C`, debug log enabled → see ignored cleanup exceptions if any. Steady-state output unchanged.

## 8. `factory.py` comment claims stubs are wired — **FIXED 2026-04**

- **File:** `src/duburi_sensors/duburi_sensors/factory.py`
- **Resolution:** `dvl` and `witmotion` are now registered in `_BUILDERS` via `_build_dvl_stub` / `_build_witmotion_stub` thin wrappers. Each instantiates the existing `*Source` class which raises `NotImplementedError("... not implemented yet. Use yaw_source='mavlink_ahrs' or 'bno085'.")` — so users hitting the wrong source get the friendly per-stub message instead of the generic `unknown yaw_source` `ValueError`. The comment now matches the code.
- **Verify:** `make_yaw_source('dvl')` raises `NotImplementedError: DVL yaw source not implemented yet. Use yaw_source='mavlink_ahrs' or 'bno085'.` (confirmed in smoke test).

## 9. `config/modes.yaml` points to a non-existent README — **FIXED 2026-04**

- **File:** `src/duburi_manager/config/modes.yaml`
- **Resolution:** The broken `src/duburi_sensors/README.md` reference is replaced by direct links to the two real files anyone looking up `bno085_*` semantics actually needs: `src/duburi_sensors/firmware/esp32c3_bno085.md` (firmware contract) and `src/duburi_sensors/duburi_sensors/sources/bno085.py` (driver). The comment also lists the additional registered yaw sources (`dvl`, `witmotion`) so operators know they exist.

---

## Triage suggestion (not binding)

Every audit-tracked bug below has been resolved. Backlog is empty.

Pool-blocker tier (fix before next pool test):
- ~~**#1** action-result lies — affects every chained mission.~~  **DONE 2026-04**
- ~~**#2** silent `set_mode` failure — directly causes the "yaw doesn't change" bug we already debugged once.~~  **DONE 2026-04**
- ~~**#3** AHRS staleness gate — same risk surface as #1/#2 once the radio glitches.~~  **DONE 2026-04**

Payload tier (fix before any torpedo / grabber / dropper code):
- ~~**#4** AUX offset.~~  **DONE 2026-04**

Quality-of-life tier:
- ~~**#5, #6, #7, #8, #9**~~  **DONE 2026-04**

Next-up candidates not from this audit (keep here as a hand-off list):
- **DVL** Nortek Nucleus1000 driver — replace `_build_dvl_stub`.
- **WitMotion** binary parser if we ever want a backup IMU — replace `_build_witmotion_stub`.
- **Vision** package (`duburi_vision`, YOLOv11 + DeepSORT) per TDR — currently absent on purpose.
- **Mission FSM** — replace `test_runner.py` with YASMIN once missions outgrow a linear script.
