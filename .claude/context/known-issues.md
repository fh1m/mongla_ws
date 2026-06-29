# Known Issues — tracked from the 2026-04 audit

> **Superseded for current state (2026-05-30).** The "backlog empty / all
> FIXED" status below is true *only for the 2026-04 and 2026-05 sprints*.
> The post-v4f full-stack audit found a 🔴 CRITICAL that postdated this
> file — `vis_approach` dispatched a `target_vis_range` field missing from
> `Move.action` (AttributeError on every call); now fixed (field added,
> interfaces rebuilt). For the **current** cross-cutting findings (TDR⇄code
> gaps, clock residue in `thrust_loop`, stray `missions/mission.py`,
> missions lacking disarm-in-finally, vision-control test gap), see
> [`robosub-2026-audit.md`](./robosub-2026-audit.md). **Live status / open
> work / fix log is now centralized in
> [`development-board.md`](./development-board.md) — start there.** This file
> is the resolved-bug history; the board is the dashboard.

> **Yaw never settled / wobbled to TIMEOUT (FIXED 2026-06).** Pool testing on
> the BNO heading source: `yaw_left`/`yaw_right`/`turn` reached the target but
> wobbled and never declared locked, so every yaw command TIMEOUTed. Cause: the
> shared settle path `_YawPID.update` in `motion_yaw.py` applied a **hard 7.5%
> min-speed floor** (`max(YAW_SPEED_MIN_PCT, |raw|)`) at *every* error past a
> (regression-tightened) **1° tolerance** — the Ch4 yaw rate couldn't decay near
> target, so a 20 kg hull (+BNO/actuator latency at 10 Hz) overshot the band and
> limit-cycled. The floor also masked the anti-stall integral. Introduced by
> `6e9f40b` (`YAW_TOL_DEG` 2°→1° + the hard floor), surfaced only when yaw-with-BNO
> was exercised hard (BNO didn't cause it; it widened the overshoot). **Fix:** the
> floor now **tapers to 0 across an approach band** (`_yaw_floor`, full only outside
> `YAW_APPROACH_BAND_DEG=6°`, →0 at tol) so the command eases in and the integral is
> unmasked; `YAW_TOL_DEG` restored to **2°**. `heading_lock` untouched (its 50 Hz
> continuous hold was never affected). **Pool tuning order:** confirm it *declares*
> success → tighten `YAW_TOL_DEG`/raise rate (scale `YAW_LOCK_N`) for precision →
> adjust band width / `YAW_KI` if it stalls in the taper zone.

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
> | `movement_linear.py` (`linear_step` / `linear_ramp`) | `motion_forward.py` + `motion_lateral.py` (`drive_*_constant` / `drive_*_eased`) — split per-axis in 2026-04 with shared logic in `motion_writers.py` |
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

## 4. `set_servo_pwm` — **SUPERSEDED 2026-06**

- **Status:** Method deleted from `pixhawk.py`. Payload (torpedo/dropper) uses ESP32 USB serial — `duburi.fire(n)` / `PayloadDriver`. No Pixhawk AUX path exists.

## 5. `Move.action` field semantics are inconsistent — **FIXED 2026-04**

- **Files:** `src/duburi_interfaces/action/Move.action`, `src/duburi_control/duburi_control/movement_commands.py`, `src/duburi_manager/duburi_manager/auv_manager_node.py`
- **Resolution:** `Move.action` now spells out the per-command contract for `final_value` / `error_value` in comments. `MovementCommands.set_depth`, `move_*`, `yaw_*`, and `stop` all return `(final_value, error_value)`; the action server's `_execute_cb` populates the result fields straight from that tuple. Per-command axes:
  - `yaw_*` → `final_value = final yaw [°]`, `error_value = signed heading error [°]`
  - `set_depth` → `final_value = final depth [m]`, `error_value = |target − final| [m]`
  - `move_*` / `stop` → `final_value = current depth [m]`, `error_value = 0.0` (no DVL odometry yet)
  - `arm` / `disarm` / `set_mode` → `final_value = current depth [m]`, `error_value = 0.0` (no axis to report)
- **Verify:** `Move.Result()` accepts `error_value = 0.25` (confirmed in smoke test) and `auv_manager_node` no longer hard-codes depth in the success path.

## 6. `test_runner.py` exits 0 on mission failure — **FIXED 2026-04**

- **File:** historically `src/duburi_manager/duburi_manager/test_runner.py`; now lives at `src/duburi_planner/duburi_planner/missions/square_pattern.py` (the legacy choreography) and the `mission` runner in `src/duburi_planner/duburi_planner/mission.py`.
- **Resolution:** Mission body extracted into `_run_mission()`; the runner wraps it in try/except, sets `exit_code = 1` on any `Exception`, and `sys.exit(exit_code)` after teardown. The ~10 lines of commented-out alternate mission steps are deleted, the docstring now matches the actual square-pattern mission, and the file's policy line ("if a step is disabled, delete it") is encoded in the docstring.
- **Verify:** `ros2 run duburi_planner mission square_pattern` against an unreachable action server prints `MISSION FAILED` and exits with `$? == 1`.

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
- ~~**DVL** Nortek Nucleus1000 driver — replace `_build_dvl_stub`.~~  **DONE 2026-04** (`nucleus_dvl.py` ships, distance commands work at pool)
- ~~**DVL heading lock**: heading lock was suspended during DVL moves (caused AUV to drift).~~  **FIXED 2026-04** (lock stays active, lock owns Ch4, DVL drives Ch5/Ch6)
- ~~**Yaw P-only controller**: snapped to target with undershoots/oscillation.~~  **FIXED 2026-04** (full PID in `motion_yaw._lock_to_target`, Kp/Ki/Kd tuned)
- ~~**DVL auto-connect**: required manual `dvl_connect` each session.~~  **FIXED 2026-04** (`dvl_auto_connect=true` background retry thread in manager)
- ~~**BNO+DVL combo**: could not use BNO085 heading + DVL position together.~~  **FIXED 2026-04** (`CompositeBnoDvlSource`, `yaw_source=bno085_dvl`)
- **WitMotion** binary parser if we ever want a backup IMU — replace `_build_witmotion_stub`.
- **Mission FSM** — populate `src/duburi_planner/duburi_planner/state_machines/` with YASMIN once missions outgrow `duburi_planner/missions/*.py` linear scripts.

---

## 2026 Competition-Grade Audit — ALL FIXED

> **Status (2026-05):** All P0 + P1 issues from the competition-prep audit are resolved.
> Commit: `fix: competition-grade audit — P0 + P1 hardening`.

### B1. Timer clock mixing — NTP-unsafe deadlines — **FIXED 2026-05**
- **Files:** `motion_yaw.py` (`_lock_to_target`, `yaw_glide`), `motion_depth.py` (`prime_alt_hold`, `wait_for_depth`)
- **Fix:** All `time.time()` deadlines replaced with `time.monotonic()`.

### B2. RC override not neutralised on arc() exception — **FIXED 2026-05**
- **File:** `motion_forward.py` `arc()`
- **Fix:** Inner while loop wrapped in `try/finally: pixhawk.send_neutral()`.

### B3. VisionState cached before preflight — **FIXED 2026-05**
- **File:** `auv_manager_node.py` `_vision_state_for()`
- **Fix:** Only insert into `_vision_states` after `wait_vision_state_ready` passes.

### B4. DVL no-reconnect on TCP drop — **FIXED 2026-05**
- **File:** `nucleus_dvl.py`
- **Fix:** Exponential-backoff reconnect supervisor thread (5 → 10 → 20 → 40 → 60 s). Position/heading reset on reconnect. `close()` stops supervisor cleanly.

### B5. DVL result unchecked in gate_flare_prequal — **FIXED 2026-05**
- **File:** `gate_flare_prequal.py`
- **Fix:** Check `dvl_result.success`; log WARNING before distance moves if DVL offline.

### B6. BNO085 calibration timeout kills sensors node — **FIXED 2026-05**
- **File:** `bno085.py`
- **Fix:** Catch `RuntimeError` from `_calibrate()`; log WARN and continue in raw mode. `offset_deg` stays `None` to signal uncalibrated state.

### B7. ByteTrack `_class_map` never pruned — **FIXED 2026-05**
- **File:** `bytetrack.py`
- **Fix:** After each `update()`, prune `_class_map` to only live + `lost_tracks` IDs.

### B8. Heading lock source-death delay 2 s — **FIXED 2026-05**
- **File:** `heading_lock.py`
- **Fix:** `SOURCE_DEAD_S = 0.5` (was `2.0`). Max uncontrolled spin ~22° (was ~90°).

### B9. Heartbeat exception at WARN — **FIXED 2026-05**
- **File:** `heartbeat.py`
- **Fix:** Log at `ERROR` level so MAVLink connection loss is visible.

### B10. No mission scoreboard — **FIXED 2026-05**
- **Files:** `duburi_dsl.py`, `mission.py`
- **Fix:** `DuburiMission._scoreboard` accumulates per-verb `{cmd, success, elapsed, msg}`. `log_scoreboard(json_path='auto')` called in `mission.py` finally block.

### B11. Surface timeout 30 s too short — **FIXED 2026-05**
- **File:** `gate_flare_prequal.py`
- **Fix:** `set_depth(0.0, timeout=60.0)`.

---

## 2026-06 Pool-Test Fixes — ALL FIXED (commit 0a3d8e1)

### C1. Vision depth alignment can command surfacing — **FIXED 2026-06-23**
- **File:** `motion_vision.py`
- **Symptom:** the vision depth-axis nudges had no floor; repeated "move up" corrections could command depth → 0m → boat hull collision.
- **Fix:** `_MIN_DEPTH_M = -0.2` constant; the `align_loop` depth axis clamps `depth_setpoint = min(depth_setpoint, _MIN_DEPTH_M)` after every nudge. Uses `min()` because depth is negative-down. (Still in force after the 2026-06-24 two-verb rewrite.)

### C2. Mission state carry-over across runs — **FIXED 2026-06-23**
- **Files:** `commands.py`, `duburi.py`, `auv_manager_node.py`, all `run()` mission files
- **Symptom:** Heading lock and `_abort_event` persisted after a mission. Second mission armed into previous heading; abort state could prevent motion.
- **Fix:** New `mission_reset` verb (in `_UNARM_SAFE`): stops heading lock thread, clears `_abort_event`, sends RC neutral. All `run()` functions call `duburi.mission_reset()` as first line. `cancel_callback` now also calls `unlock_heading()`.

### C3. Slalom diagonal movement — **FIXED 2026-06-23, SUPERSEDED 2026-06-24**
- **File:** `motion_vision.py`
- **Symptom:** the old combined-axis tracker drove `lat` and `forward` at once → AUV moved diagonally → risk of hitting slalom pipes.
- **Fix (original):** lat-priority smooth gating inside the old combined tracker (`fwd_pct` scaled down by a `speed`-based lateral-dominance term).
- **Superseded:** the two-verb rewrite split tracking into `vision_align` (centre only, never drives forward) and `vision_move` (drives forward; optional `maintain=±px` lateral hold). The diagonal case can no longer arise, so the removed `speed`-scaled dominance gate is no longer needed.

---

## 2026-06 Vision Two-Verb Rewrite + Harmony Fixes — ALL FIXED (2026-06-24)

> The 9-verb vision API (`vision_align_yaw/lat/depth`, `vision_align_3d`,
> `vision_hold_distance`, `vision_lock_fire`, `vision_acquire`, `look_around`;
> DSL `vision.find/home/turn/slide/hover/approach/track/scan/hold`) was
> replaced by **two** pixel-native verbs — `vision_align` (centre on
> lat/yaw/depth at signed pixel offsets) and `vision_move` (drive forward to a
> bbox fill ratio). Neither raises on a miss: the server returns
> `success=True` with an outcome code in `Move.Result.final_value`
> (`ALIGNED`=0 / `LOST`=1 / `TIMEOUT`=2 / `NO_CAMERA`=3 / `ABORTED`=4). `gain`
> is a hard max-speed cap; search is a mission-authored `fallback`. Removed
> params: `vision.deadband`, `lock_mode`, `depth_anchor_frac`,
> `distance_metric`, `target_bbox_h_frac`, `stable_lock_s`, `h_frac_close`,
> `proximity_min_scale`, `speed`, `use_tracks` (and the `--tracking` flag).
> New `/duburi_manager` params: `vision.kp_lat`=60, `vision.kp_yaw`=60,
> `vision.kp_depth`=0.05, `vision.kp_forward`=200, `vision.lost_grace_s`=1.0,
> `vision.frame_fill_default`=95, `vision.align_stable_frames`=3. Control loops
> read `/detections` only; `detected()` is case-insensitive.

### D1. `surface()` re-entrant deadlock — **FIXED 2026-06-24**
- **File:** `duburi.py`
- **Symptom:** `surface()` runs in `_command_scope('surface')` and then calls `set_depth()` (its own scope). With a plain `threading.Lock` the second acquire on the same thread blocked forever — the one safety verb most likely to be needed could hang.
- **Fix:** `Duburi.lock` is now a `threading.RLock`. Commands stay serialized one-at-a-time across threads (an `RLock` only re-admits the thread that already holds it).

### D2. Heading-lock lifecycle leaks (timeout zombie + disarm) — **FIXED 2026-06-24**
- **Files:** `heading_lock.py`, `duburi.py`, `state_machines/states/navigation.py`
- **Symptom:** a lock that hit its own `timeout` released Ch4 but left `Duburi._heading_lock` set and the heartbeat paused — a "zombie" lock that still looked active (so translation verbs released Ch4 to a dead thread). `disarm()` did not stop an active lock, so its Ch4 yaw-rate stream kept firing after the drop to MANUAL.
- **Fix:** `HeadingLock` gained an `on_exit` callback; on timeout it fires `Duburi._on_lock_timeout`, which clears the handle and releases the heartbeat (identity-guarded so a freshly engaged lock isn't clobbered). `disarm()` now stops/joins an active lock and releases the heartbeat before disarming. FSM `LockHeadingState` passes a long `lock_timeout` (task/hold duration, not the FSM state timeout) so the lock isn't killed mid-task; FSM `DisarmState` calls `release_heading()` before `disarm()`.

### D3. `vision_align` fought an active heading lock — **FIXED 2026-06-24**
- **Files:** `vision_verbs.py`, `motion_vision.py`
- **Symptom:** a lat/depth-only `vision_align` wrote `yaw=1500` every tick, racing the background heading lock's 20 Hz Ch4 stream.
- **Fix:** when a lock is live and `yaw` is *not* an align axis, `vision_align` runs the loop with `release_yaw=True` — lateral goes via `send_rc_translation` and Ch4 is left entirely to the lock (the same path `vision_move` uses). When `yaw` *is* an axis the lock is suspended and the loop drives Ch4 itself, retargeting the lock to the achieved heading on exit.

### D4. `vision_move` depth + post-failure cleanup — **FIXED 2026-06-24**
- **Files:** `vision_verbs.py`, `auv_manager_node.py`
- **Symptom:** a mission jumping straight to `vision_move` from MANUAL had its depth setpoint silently dropped; and the manager's post-exception cleanup used raw `send_neutral()` which clobbered an active lock's Ch4 for a tick.
- **Fix:** `vision_move` calls `_ensure_alt_hold('vision_move')` at entry. The manager exception path now neutralises via the lock-aware `duburi._writers().neutral()`.

### D5. A bad vision verb could abort a whole mission — **FIXED 2026-06-24**
- **File:** `vision_dsl.py`
- **Symptom:** a setup error (bad camera name, ALT_HOLD rejected, disarmed) raised `MoveFailed`/`MoveRejected` out of the DSL and unwound the mission.
- **Fix:** `_orchestrate` catches `MoveFailed`/`MoveRejected`/`Exception` and returns a non-fatal `VisionResult(False, 'FAILED', …)`; the mission logs it and continues to the next step.

### D6. Silent "sees but doesn't move" on a mis-scaled stream — **FIXED 2026-06-24**
- **Files:** `motion_vision.py`, `vision_state.py`
- **Symptom:** pixel math ran before `camera_info` arrived (image size still `(0,0)`), mis-scaling the error; and a class-name mismatch produced no diagnostic.
- **Fix:** the verbs return `NO_CAMERA` until `vision_state.info_seen()` is true, so the controller never steers on an unscaled pixel error. A throttled "`<class>` not among live detections […]" warning fires when boxes are present but none match the requested class. Class matching is case-insensitive.

---

## Forks we evaluated (so we don't revisit)

### `BumblebeeAS/ardupilot_fix` — STALE DUD (evaluated 2026-04)

* **Source:** https://github.com/BumblebeeAS/ardupilot_fix
* **State vs upstream:** **1 commit ahead, 3911 commits behind** `ArduPilot/master`. 0 stars, 0 forks. No CI configured.
* **The single commit** (`xelisce`, 2025-05-23, "hard code variables into file fix, passed all tests"): adds 9 unused declarations to `libraries/AP_DDS/AP_DDS_Client.cpp`. No semantic ArduSub change. No new mode, no new failsafe, no new MAVLink behaviour.
* **Verdict:** nothing to learn or pull. The fork name suggests a fix for something interesting but the diff is non-semantic. Stay on the upstream Sub-stable-V4.5.x branch documented in [`ardusub-canon.md`](./ardusub-canon.md).
* **Re-evaluate when:** the fork's `xelisce` author (or `BumblebeeAS` org) ships a second semantic commit. Until then, do not spend an evening "evaluating" this again.
