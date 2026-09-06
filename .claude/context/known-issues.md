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

### D7. `heading_lock` jittered the hull during a lat-only `vision_align` — **FIXED 2026-06-29**
- **File:** `heading_lock.py`
- **Symptom:** with a lock active, a lat/depth-only `vision_align` made the hull **wobble left/right in yaw** while lateral-correcting. The lock's `speed = max(LOCK_SPEED_MIN_PCT, …)` hard floor is a **relay on the Ch4 yaw RATE**: holding a still hull it barely fired (error stayed in the 1° deadband), but a lat align strafes on Ch6 and an off-CG vectored frame turns that into a **continuous yaw moment** that pushes heading out of the deadband every tick → the lock kicks ≥5% → overshoot → sign flip → **limit cycle**. Same mechanism as the `turn` bug fixed in `ab2014f` (`motion_yaw`), which deliberately left `heading_lock` untouched on the (correct-for-still-hold) assumption it was fine.
- **Fix:** taper the floor — `LOCK_APPROACH_BAND_DEG = 6.0` + `_lock_floor()`; `_lock_command()` now does `max(_lock_floor(|err|), min(LOCK_PCT_MAX, kp·|err|))`. Only the 1°–6° band softens (kills the relay); ≥6° keeps the full stiction-break floor, ≤1° still commands 0. Pure-P (no integral), so under a sustained disturbance it settles to a small **bounded heading offset** rather than wobbling — raise `LOCK_KP_PCT_PER_DEG` or add a `LOCK_KI` follow-up if that droop is too large. **Answer to "would releasing the lock fix it?": no — that hands yaw to ArduSub's aluminum-hull compass, the worst jitter source. Keep the BNO lock; the taper is the cure.** Pool re-verify straight-line heading hold during `move_forward_dist` (shared lock). Toy-plant test pins taper-settles vs hard-floor-wobbles.

### D8. `vision_align`/`move` commanded Ch4 when yaw wasn't requested — **FIXED 2026-06-29**
- **File:** `vision_verbs.py`
- **Symptom:** Ch4 release was gated on **lock-state** (`release_yaw = _lock_active() and not touches_yaw`), so a bare `align(lat=,depth=)` with **no** lock wrote `yaw=1500` every tick (and `vision_move` did likewise without a lock) — commanding the yaw channel the operator never asked for, and able to fight a later/other Ch4 author. (Refinement of D3, which only covered the lock-active case.)
- **Fix:** gate Ch4 on the **yaw axis**, not the lock: `vision_align` → `release_yaw = not touches_yaw`; `vision_move` → `release_yaw = True` always (it never computes a yaw command). The verb now writes Ch4 **only** when `yaw` is a requested align axis; otherwise it stays on `send_rc_translation` (lateral-only), leaving Ch4 to the lock / heartbeat / ArduSub.

### D9. `align(err=0)` logged "aligned (36px)" — **FIXED 2026-06-29**
- **Files:** `commands.py` (unchanged, documented), `motion_vision.py`, `detector_node.py`
- **Symptom:** an explicit `err=0` was silently turned into a 40px deadband, so the loop honestly reported `aligned (36px)`. Root cause is the **rosidl `0==unset` convention** in `commands.fields_for` (`unset = (value == 0.0)`) substituting the spec default — which is **load-bearing**: the CLI deliberately omits vision fields so the rosidl zero triggers `vision.*` ROS-param defaults (`_LIVE_TUNED_COMMANDS`). An explicit `0` and an omitted field are indistinguishable on the wire, so `err_px` can't be exempted.
- **Fix (honesty, not a sentinel):** `err=0` means "use the default/param" (documented); a **small positive `err` is the tight knob** (only literal `0` collides — e.g. `err=8` is honored). `align_loop` clamps the effective deadband to `MIN_ALIGN_ERR_PX` (≈5px bbox jitter) so an over-tight positive `err` can't perpetually TIMEOUT, prints the effective deadband at align start (floor noted, never silent), and the success line now states it: `aligned (N/Mpx)`. The detector's always-on line was reworded from `… align ['hole'] center -> (0,0)` to `[ offset … ] 'hole' bearing (live, off-centre)` so the operator never reads that raw-offset telemetry as the verb's alignment verdict.

### D10. Control coast through detection gaps — re-enabled SAFELY (opt-in) — **2026-06-30**
- **Files:** `vision_state.py`, `motion_vision.py`, `vision_verbs.py`, `vision_tunables.py`, `tracker_node.py`, `roboflow_tracker.py`
- **History (the bug we must not repeat):** commit `a8bf8ea` added `vision.use_tracks` → the control loop read `/tracks`; `8335afb` ripped it out. Root cause: coasted (Kalman-predicted) boxes carry `score=0.0`, which collided with the conf gate (`min_score`/`ctrl_conf>0`) → `bbox_error()` returned `None` → `_present()` False → **drive neutral → AUV stopped** despite a good predicted box. Latent second mode: `_freshness()` keys off *message* age, but a predicted box arrives every frame (age≈0) → **full authority on a drifting phantom**. The fix made control read raw `/detections` only, occlusion handled by mission `fallback`.
- **Re-enable design (Part B, `vision.coast_s`, default OFF):** the coast is now **additive and opt-in**, inverting both failure modes — (1) a live `/detections` box ALWAYS wins (`/tracks` is consulted only to fill an empty tick; coast never gates out or overrides a real box); (2) coast authority decays by **TRUE detection-age** via a separate `_coast_authority` curve dispatched by `_authority()` (a coasted box is fresh every tick, so applying `_freshness` would double-decay it — and a *live* box must NOT be coast-decayed); (3) conf-exempt **only for the locked `track_id`** (a predicted box of any other id is ignored — built outside the `min_score` path, only for `locked_id`). `coast_s=0` ⇒ byte-identical to the proven raw-`/detections` path.
- **Timeout ladder (4 rungs, must stay ordered):** `_freshness` (0.4s, per-frame live staleness) < `vision.coast_s` (~0.8, coast window) < `vision.lost_grace_s` (1.0, → LOST → `fallback`) < tracker `max_predict`/`lost_track_buffer` in **wall-time** (the track must outlive the coast). The 4th rung bit us once: the Kalman smoother drops a track from `/tracks` after `max_predict_frames`, so `max_predict` (launch default was 10=0.5s, yaml 15=0.75s — both **below** `coast_s=0.8`) silently truncated the coast; raised to 30 (1.5s @20Hz). **Pool-gated:** validate with `coast_s=0` first (byte-identical), then `≈0.8` on the torpedo hole/gate; slalom last and cautiously (a coasted lateral box drifts fastest there).

### D11. Fire could leave on a stale/coasted box; align declared on one frame at low FPS — **FIXED 2026-07-01 (completion audit)**
- **Files:** `motion_vision.py` (`align_loop`), `Move.action`, `vision_tunables.py`.
- **V-FIRE (the dangerous one):** the mid-hold `on_locked` (torpedo/dropper fire) gated only on `stable >= align_stable_frames`, with **no freshness check**. If the detector froze right after alignment, `stable` stayed at threshold on the ≤`_STALE_LIMIT_S` (1.0s) cached box (freshness zeroed the *drive* at 0.4s, but not the *fire*), and a torpedo could leave on a stale — or, with `coast_s>0`, a Kalman-**predicted** — box. **Fix:** fire now additionally requires `not sample.coasted and sample.age_s <= VISION_FRESH_FULL_S` (0.10s). Coast holds the lock; it never takes the shot. A dropping hole waits (within the hold) instead of firing blind.
- **V-STABLE:** `stable` (and the settle gate's `prev_worst`) counted 20 Hz **loop ticks**, not detections — so at 3-4 Hz a single in-band frame re-read 3× in 0.15s declared ALIGNED / armed the fire on effectively one frame, and the settle gate was degenerate (`worst==prev_worst` on re-reads). **Fix:** a detection's arrival time is `now - sample.age_s` (constant across re-reads); `stable`/`prev_worst` only advance on a **new** frame (`> last_frame_at + _FRAME_EPS_S`, 5ms). `align_stable_frames` now means "N distinct in-band detections" — FPS-independent. Backward-compatible: at healthy FPS (age≈0 every tick) every tick is a new frame, so behaviour is unchanged (and the `age_s=0` test doubles stay valid). Duration budgets re-verified (smallest align 4s ≫ ~1s worst-case declare @3-4Hz). **Raising real FPS (TensorRT engine) is still the primary lever** — this is the per-frame guard.

### D12. Torpedo 10/10: fire-would-not-leave, depth z-wobble, terminal yaw jitter — **FIXED 2026-07-01 (pool feedback, 6/10→target 9-10/10)**
- **Files:** `motion_vision.py` (`align_loop`, `_vision_yaw_floor`), `heading_lock.py` (`set_hold_mode`, deadband param), `vision_verbs.py`, `duburi.py` (`_set_lock_hold`), `commands.py`, `Move.action`, `vision_dsl.py`, `missions/{task_torpedo,pool_day_torpedo}.py`.
- **Fire would not leave despite a perfect lock (the 4/10 miss):** D11's fire-freshness gate (`age_s <= VISION_FRESH_FULL_S`, 0.10s) is *narrower than one frame period* at 3-4 Hz, so a perfectly-aligned hull kept missing the fresh-tick coincidence with `stable≥N and (now-aligned_at)≥fire_t`. **Fix:** gate the fire on **`is_new_frame and not sample.coasted`** — fire on the tick a genuinely NEW live box lands. Fresh by construction at ANY FPS (fixes the miss) AND strictly safer than D11: a frozen detector produces no new frame, so it can never fire on a stale box even while `stable` stands held at threshold through a mid-hold freeze (the freeze-AFTER-alignment case a looser age gate would reopen). Discriminating test: `test_fire_withheld_when_detector_freezes_after_alignment`.
- **`fire_pass` (opt-in, default off):** guaranteed partial-points shot — if no strict in-band fire landed, fire at a NATURAL exit (TIMEOUT / hold-complete) provided the target was seen LIVE within `lost_grace_s` (never on a never-seen or coasted-only target). DSL `align(fire_pass=True)`.
- **Depth axis z-wobble:** the setpoint was recomputed EVERY 20 Hz tick (up to 0.4 m/s slew ArduSub chased) and nudged *even inside the deadband* (dithering on bbox-y jitter). **Fix:** setpoint now steps **only in the 5 Hz block**, **frozen inside the deadband**, capped per-update by the new **`depth_step`** resolution arg (m; 0.02 slow .. 0.10 coarse; the sole depth-rate knob — `gain_depth` no longer scales depth). Max slew = `depth_step × 5 Hz`, so ArduSub's ALT_HOLD PID settles between steps. `_MIN_DEPTH_M` surface floor preserved.
- **Terminal yaw jitter (launcher wobble at the hole):** the reported wobble is NOT the vision yaw floor — both torpedo missions **drop the yaw axis** at the hole (`release_yaw=True`), so Ch4 is owned by `heading_lock`, which limit-cycled against the lateral-strafe yaw moment. **Fix:** (a) `heading_lock` **fire-window quiet mode** — `vision.align(hold_heading=True)` widens the lock deadband (`LOCK_DEADBAND_DEG` 1° → `LOCK_HOLD_DEADBAND_DEG` 3°) for the hold via `set_hold_mode()`/`_set_lock_hold()` (try/finally), so the lock holds steady instead of micro-correcting sub-deg noise; restored on exit. Wired into both torpedo terminal aligns. (b) The vision yaw floor (`motion_vision.py`) is ALSO tapered now (`_vision_yaw_floor`, mirrors `heading_lock._lock_floor` / `motion_yaw._yaw_floor`) so any yaw-axis align eases in instead of relay-slamming — correct-in-general, though it does not drive the *terminal* (yaw-dropped) path.
- **Pitch/roll:** owned entirely by ArduSub's stabilizer — an `ATC_*` tune + physical trim/ballast matter, not our loop. (A calmer depth slew may quiet pitch as a side effect on a 6dof frame with unbalanced vertical thrusters.)

### D13. Vision verb dropped into an autonomous fallback SEARCH *during* a camera switch — **FIXED 2026-07-10 (pool feedback)**
- **Files:** `duburi_dsl.py` (`_wait_detector_warm` + constants), `vision_dsl.py` (`_orchestrate` warm-gate; `align`/`move` reorder), `display_node.py` (switch indicator), `test_vision_dsl.py`.
- **The bug (mission-critical, and SILENT):** right after a camera/model/class switch the target detector is briefly **cold** — it has not yet produced a `/detections` frame under the new config. A vision verb sent immediately has its acquire clock start on that cold detector; `align_loop`'s `lost_since`/`lost_grace_s` (1.0s) returns **LOST regardless of whether the target was ever seen** (`saw_target` only swaps the *reason string*, not the timing). When a `fallback` is provided, the DSL `_orchestrate` then runs the mission's **search** — so the AUV wanders off mid-mission looking for a target that was about to appear. Invisible in logs because `LOST → fallback` is a *normal* path (no error). Long-standing since the auto-switch landed (`13bb53d`), not a fresh regression. Observed symptom: HUD still on the previous camera while the AUV had already started a fallback search.
- **Scope = DSL-only (this decides the fix layer):** `fallback` is a DSL construct, and `hold_through_loss=(fallback is None)` means a **no-fallback** verb holds through loss and never searches. So the danger is structurally *fallback-only* — the fix is in the DSL, **not** the high-criticality `align_loop` (touching it would be blast radius for a defect that can't fire without a DSL fallback).
- **Fix — warm-gate:** `_orchestrate` now blocks on `DuburiMission._wait_detector_warm(camera)` **before the first goal, only when a `fallback` is set**. It gates on the detector **PRODUCING frames** (the detector publishes every inference tick, empty or not — so a fresh `_det_cache` entry = "alive"), **NOT** on the target being visible. Consequence: a genuine "target simply absent" search is **not** delayed (a warm detector returns on the first pump), only a cold/just-switched detector waits. Covers **both** switch paths (ClassRef set-inside-`align` *and* string `set_model`/`set_classes`-before-`align`) because it gates at the goal boundary, not on how the config was set. Bounded by `_DETECTOR_WARMUP_S` (2.5s); on timeout it **warns** (`detector … still not producing frames after warm-up`) and proceeds. **Tuning tripwire:** if that warn line appears in pool logs, the detector's post-switch first-inference is exceeding 2.5s → **raise `_DETECTOR_WARMUP_S`**, it is not a hardware fault.
- **Fix — reorder (`align`/`move`):** was `_activate_camera` (resume + settle) → `_resolve_target` (program model/classes), so the settle warmed the **old** model and the detector briefly emitted wrong-class boxes. Now `_ensure_detector` → `_resolve_target` (program) → `_activate_camera` (resume + settle) so the settle warms the **new** config and the warm-gate returns instantly.
- **Fix — HUD:** during the gap between a switch and the new camera's first frame the viewer showed the *frozen old frame* ("stuck on forward"). Now shows a `CAMERA → <name> waiting for stream` screen for up to `_SWITCH_WAIT_S` (3s). **Cosmetic only** — the control danger was the fallback (fixed above). If the HUD stays stuck *past* the switch, that is a separate "downward stream never starts" camera bug, not this.
- **Verified:** live ROS-graph proof — cold detector → warm-gate **blocks 2.46s** (no immediate fallback); warm/producing → returns **0.01s** (zero search-case penalty). 5 DSL tests (gate wired when `fallback` / skipped without / program-before-activate order / warm+cold logic); planner 176, vision 53 green. The warm-gate sits *before* the `while` loop, so fallback re-entries don't re-gate — the ≤2.5s cost is paid at most once per verb, only when cold.
- **Mitigation with no code change** (for any existing mission on an older build): `if duburi.wait_for('fire', timeout=3): duburi.vision.align(...)` before the align.

### D14. Single-camera vision missions broke on model identity: `set_model`/ClassRef rejected, and a missing registry model crashed the whole pipeline — **FIXED 2026-07-10 (pool feedback)**
- **Files:** `detector_node.py` (`_model_stem`, `_resolve_model_key`, single-model `_single_model_name`, resilient registry load, `active_model` handler), `model_context.py` (ClassRef carries the STEM), `test_model_context.py`, `test_model_identity.py`.
- **The bug (reproduced live, two failure modes on a single-camera launch):**
  1. **`set_model`/ClassRef rejected in single-model mode.** `vision.launch.py model:=gate_rescue_repair` loads ONE model with no registry. Any mission that switched model — an explicit `set_model('gate_rescue_repair')`/`use(...)` **or** a `ClassRef` target (`duburi.models(...).x.cls`, which auto-`set_model`s) — hit `active_model: no registry loaded (use 'models' param at startup)` and **aborted**, *even though the requested model was the one already loaded*. (`set_classes`/`set_conf` worked — only model-switch was broken.)
  2. **`models:=` registry was all-or-nothing → "no pipeline".** The eager registry load `raise`d `RuntimeError` if **any** model failed, taking the **whole detector node down** (no `/detections`, no OpenCV) — even if other models loaded fine. So a full-competition `models:=gate=…,slalom=…,torpedo=…` **crashed** whenever a task's `.pt` wasn't on the box yet (slalom/torpedo weights absent). Single-model mode loads async and survives; multi-model was eager + fatal — the asymmetry.
- **Root cause (identity fracture):** `ClassRef` carried the **DSL alias** (`duburi.models(alias='stem')` key), not the stem — `ModelHandle` stored `_stem` but never used it. Missions register `duburi.models(gate='gate_flare_medium_100ep')` / `robosub=('gate_rescue_repair',…)` where **alias ≠ stem**, so `set_model('<alias>')` named something no detector knew. The stem is the one identity both sides share (it's what actually loads).
- **Fix — stem is the universal model identity:**
  - `ClassRef` now carries the **stem** (`_stem`), so `_resolve_target` sends `set_model('<stem>')`. Only consumer was `vision_dsl.py:_resolve_target`.
  - **Single-model** detector records `_single_model_name` (basename-stem of `model_path`); `set_model(<that stem>)` is a **no-op SUCCESS**, any other name a **clear reject** (`single-model launch loaded 'X'; … relaunch with model:=Y`) — no `'registry'` word, so the DSL surfaces this message directly.
  - **Registry** detector builds a `stem → key` index; `set_model` accepts **KEY OR STEM** (`_resolve_model_key`), so `use('gate')` (alias key) *and* `set_model('gate_rescue_repair')` (stem/ClassRef) both switch. Stem→key collision = last-wins + warn.
  - **Registry load is resilient:** a failed model is **SKIPPED with a loud ERROR** (`… FAILED — SKIPPED … loaded: […]`), fatal **only** if the registry ends empty. If the *startup* `active_model` was the one that failed, it falls back to a loaded model with an unmissable ERROR (never a silent wrong-model substitution).
- **Verified (live detector, CUDA):** single-model `set_model('gate_rescue_repair')`→success, `set_model('slalom_red_pipe')`→clear reject, `resume_detector`/`set_classes`/`set_conf`→success; **real `ClassRef('gate_rescue_repair'/'gate')` through `_resolve_target`**→OK. Registry-with-missing-model→**node UP + `/detections` present**; `set_model` by KEY *and* STEM both succeed; skipped model rejects clearly. 10 new unit tests; planner 181, vision 53 green.
- **No `models:=` needed for a single-model pool run:** `vision.launch.py model:=<stem>` + a mission that refers to `<stem>` (ClassRef or `set_model`) now Just Works. Use `models:=` only for genuine mid-mission *multi*-model switching; prefer **bare stems** (`models:=gate_rescue_repair,slalom_red_pipe`) so key==stem.
- **Dual is the same code path — verified.** Node-routed `set_model('<stem>', node=_FWD/_DWN)` and `ClassRef(camera=…)` land on the right detector; a cross-request (forward asked for the downward model) rejects. "Dual worked before this fix" was launch-config, not code: dual launched with a bare-stem `fwd_models:=` registry (key==stem) dodged both bugs; the **default** dual single-model-per-cam config would have hit Bug 1 too — now fixed for every launch form.
- **Partial-weights competition run (know before the pool):** the resilient registry brings the pipeline **up on whatever loaded** (e.g. gate alone), so gate scores. But a mission that later *reaches* an unloaded task and calls `set_model('slalom_red_pipe')` raises out of `_resolve_target` — the mission runner then aborts (loud, safe: release+stop+disarm) **at that task**, so tasks *after* it don't run. Net vs. before: pre-fix the missing `.pt` crashed the detector at launch so **nothing** scored; now everything up to the missing model does. To run a partial set cleanly, launch with only the models you have.

### D15. Silent-bug audit (control/vision/planner) — `.engine` verified; failsafe disarm + per-model-conf fixed — **2026-07-10**
Follow-up audit for the *same class* of hidden bug as D14 (config-dependent silent divergence, identity mismatch, all-or-nothing). `.engine` (TensorRT) was verified, three fixes landed, the rest documented.

- **`.engine` (TensorRT) — VERIFIED, works with the D14 identity fix.** `yolo._resolve_model_path` prefers `<stem>.engine` over `.pt`; identity is the extension-less **stem**, so `model:=`/`models:=`/ClassRef/`set_model('<stem>')` all match a `.engine`. Live `conf`/`iou`/`max_det` tuning **DOES** take effect on an engine (passed to `predict()` each call + post-inference class filter — NOT baked). Class labels come from the sidecar `<stem>.yaml` (`_load_class_index`), preferred over embedded names, SAME path for `.pt`/`.engine`; the present sidecars (`gate_rescue_repair`, `bin_fire_blood`) match their models' class order (verified). **Only `imgsz`/`half` are export-baked** — `imgsz:=` silently only rescales the `.pt` fallback; re-export the engine to change it. **Silent risk:** a `.engine` with NO sidecar AND no embedded `names` → class indices → empty allowlist → detector returns `[]` every frame (warned once). Mitigation: **keep each `<stem>.yaml` sidecar beside the exported `<stem>.engine`.**
- **FIXED — F1: mid-command FS_PILOT disarm on a long `set_depth`/`surface` (field-observed).** `_command_scope` pauses the 5 Hz heartbeat for the whole command; `wait_for_depth` streamed only `SET_POSITION_TARGET` (no `RC_CHANNELS_OVERRIDE`). With **no heading-lock** active (a lock's Ch4 stream would otherwise feed the failsafe), RC went silent for the hold → ArduSub `FS_PILOT_INPUT` (timeout 3 s) disarmed mid-command — worst on a 60 s emergency `surface()`. **Fix:** `wait_for_depth` now streams a lock-aware `depth_keepalive` each tick (`motion_writers.make_writers`): no-lock → `send_rc_override(throttle=NO_OVERRIDE)` (the heartbeat's all-neutral frame **minus Ch3**, 5 channels feed FS_PILOT); lock → `send_rc_translation(throttle=NO_OVERRIDE)` (Ch4 left to the lock). **Ch3 is RELEASED** so ALT_HOLD's position controller still drives to the setpoint — depth is REACHED *and* the failsafe stays fed. This mirrors the pool-proven vision pattern ("Release Ch3 to ALT_HOLD whenever depth is in play", motion_vision) — vision streams the same `SET_POSITION_TARGET` with Ch3 released for 20 s and reaches target, so depth-reaching is proven by parity. Files: `motion_writers.py` (`depth_keepalive`), `motion_depth.py` (`wait_for_depth`/`hold_depth` `keepalive=`), `duburi.py` (`set_depth` + style dives). **Bench gate (on-vehicle confirmation):** a 30 s `set_depth` and a 60 s `surface()` with NO lock active must **reach depth / ascend to ~0 AND not disarm**. Frame: closes the most likely disarm mechanism; if disarms persist, cause is elsewhere (battery sag / tether / EKF).
- **FIXED — F2: `set_conf(model='<stem>')` silently no-op'd (the D14 identity-bug class, unfixed spot).** `_apply_model_conf` did an exact-key registry lookup; unlike the `active_model` handler it never used `_resolve_model_key`/`_single_model_name` (its docstring even claimed "or its stem"). So a per-model conf tighten dropped on a single-model launch (`_active_name` is None) and an aliased registry — e.g. `demo_dual_camera` running the torpedo model tight to stop a spurious box winning the terminal lock. **Fix:** resolve by KEY or STEM in registry mode, match `_single_model_name` in single mode. Live-verified: `set_conf(model='gate_rescue_repair')` now applies. **Safe on the competition bare-stem dual registry** (key==stem).
- **FIXED — F3 (diagnostic only): silent low-FPS translation stall.** `_freshness` zeroes lat/fwd for a live bbox 0.4–1.0 s old while NOT declaring LOST (no fallback fires) → the hull barely translates yet never searches, a mystery TIMEOUT at low detector FPS. **Mitigated by `.engine`** (20-30 Hz → fresh≈1.0). Added a throttled `[VIS ] low detector FPS: lat/fwd authority N%` line (`_warn_low_fps`) at both align/move freshness sites — pure observability, no behaviour change.
- **DOCUMENTED, not fixed (imperative path unaffected / phase-2):**
  - **VF1 (operator P0, not code):** `slalom_red_pipe` / `torpedo_blood_hole` / `octagon` have committed `.yaml` sidecars but **no `.pt` and no `.engine` in the tree** — those tasks cannot run and engines can't be exported without the source weights. **Get the weights on the Jetson + export engines before the run.** (Loud at `set_model` time; still the single biggest task risk.)
  - **VF6:** `vision_dual.launch.py` default `dwn_device=4` vs `config.py` `downward` device `2` — with two identical USB cameras the int index is unreliable; **use the by-path `dwn_device_path`/`fwd_device_path` symlinks** on pool day (the right int is unknowable from code).
  - **C1:** raw CLI `vision_move --fwd_fill 0` → 0==unset makes it a 95% fill-stop, not pass-through; use `--fwd_fill -1`. **DSL `move(fwd=None)` is SAFE** (sends −1).
  - **H2:** a no-yaw `vision_align` with no active lock releases Ch4 → ArduSub's untrusted compass owns heading → slow drift. Mission rule: `lock_heading` before a lat/depth-only align.
  - **FSM (phase-2, NOT run — imperative `detected()` is the competition path):** `SetDetectorState` bypasses `use_camera()` (no pause/resume handoff → can't do a forward→downward dual run); a rejected `set_model` ABORTs the whole FSM run (should skip the task); bin search times out on the default `paused:=true` dual launch. Fix when phase-2 FSM is activated, mirroring the imperative `use_camera` + per-chunk try/continue.

---

## Jetson environment / dependency pitfalls (JetPack 6.2, py3.10) — **2026-06-30**

These are **environment** problems, not code bugs — but they take down the whole
vision launch (`vision.launch.py`) with confusing tracebacks, so they live here.
All three were hit on the Orin Nano on the same day; symptom was every node dying
before the camera frame loop started.

> **Live-checked on the Jetson (Orin Nano, JetPack 6.2, py3.10) — 2026-06-30, by Claude.**
> After the three fixes below, `ros2 launch duburi_vision vision.launch.py` was run on
> the actual hardware and came up healthy: all 3 TensorRT engines loaded
> (`slalom_red_pipe` / `gate_rescue_repair` / `torpedo_blood_hole`) and the display
> reported `cam=OK det=OK trk=OK`. `trackers==2.4.0` confirmed importing and building
> `engine=ocsort` on numpy 1.26.4 on-device.

### E1. NumPy 2.x ABI break kills every vision node (`_ARRAY_API not found`)
- **Symptom:** every node (`camera_node`/`detector_node`/`vision_display`) crashes at
  `from cv_bridge import CvBridge` with
  `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6 …`
  → `AttributeError: _ARRAY_API not found`.
- **Root cause:** a user-site `numpy 2.2.6` (`~/.local/lib/...`) shadowed the
  JetPack/ROS system numpy. ROS Humble's `cv_bridge` boost extension **and** the
  system `cv2` are compiled against the NumPy 1.x C-ABI and segfault under 2.x.
  `ultralytics` also pins `numpy<2.0.0`.
- **Fix:** `pip3 install "numpy==1.26.4"` (last 1.x; correct target for this stack).
- **Guard:** do not let any `pip install` pull numpy 2 back. If something forces it,
  reinstall 1.26.4 and that package with `--no-deps`.

### E2. `vision_display` crashes with `namedWindow … rebuild with GTK` (headless OpenCV)
- **Symptom:** `cv2.error: (-2:Unspecified error) The function is not implemented.
  Rebuild the library with … GTK+ … support` at `cv2.namedWindow`, then `exit code -6`.
- **Root cause:** pip `opencv-python-headless` (no GUI) + `opencv-python` were
  installed in user-site and **shadowed** the GUI-capable JetPack system OpenCV.
  The headless wheel wins → no window backend. (These wheels also want numpy≥2,
  compounding E1.)
- **Fix:** `pip3 uninstall -y opencv-python opencv-python-headless` → import falls
  back to the system `cv2` (4.12.0, **GTK3** build, numpy-1.x compatible). Verify:
  `python3 -c "import cv2; print(cv2.__file__)"` should be under `/usr/local/lib` or
  `/usr/lib`, **not** `~/.local`.

### E3. Roboflow `trackers` "unavailable" — the **2.5.0 PyPI wheel is broken**, NOT a numpy pin
- **Symptom:** `tracker_node` logs
  `roboflow trackers unavailable (… Install: pip install trackers); falling back to
  legacy_bytetrack`, even though `pip show trackers` reports it installed.
- **Root cause (the trap):** the **`trackers 2.5.0` wheel on PyPI is a 9.7 kB dud** —
  it ships only `dist-info` metadata + a CLI stub and **contains no `trackers/`
  package**, so `import trackers` → `ModuleNotFoundError`. `--force-reinstall` just
  reuses the same empty wheel. The `numpy>=2.0.2` pin in its metadata is a **red
  herring** — it is purely conservative; the code runs fine on numpy 1.26.4.
- **Fix:** install the **last good release, `2.4.0`** (126 kB, real code), with
  `--no-deps` so it can't drag numpy 2 / `opencv-python` back and re-trigger E1/E2:
  ```bash
  pip3 install --no-deps --force-reinstall "trackers==2.4.0"
  ```
  Verified on numpy 1.26.4: `tr.OCSORTTracker` + `tr.ByteTrackTracker` instantiate
  and run real `update()` calls; the node wrapper builds `engine=ocsort`. So OC-SORT
  (the documented default) **does** work on this Jetson — you do **not** have to
  accept the legacy ByteTrack fallback.
- **Watch:** if a future `pip install` upgrades to `trackers 2.5.0`, the engine
  silently disappears again (broken wheel). Re-pin to `2.4.0 --no-deps`. Re-evaluate
  when Roboflow ships a `>2.5.0` whose wheel actually contains the module.

> **One-shot recovery (all three at once):**
> ```bash
> pip3 install "numpy==1.26.4"
> pip3 uninstall -y opencv-python opencv-python-headless
> pip3 install --no-deps --force-reinstall "trackers==2.4.0"
> ```
> Benign remaining log noise (safe to ignore): numpy "smallest subnormal … is zero"
> UserWarning (aarch64 build quirk), TRT `NvMapMemAlloc … error 12` / "engine plan
> across different models of devices", and the `target=None deprecated` FutureWarning.

### E4. cv2 windows die under VSCode Remote-SSH — `Can't initialize GTK backend` (no `$DISPLAY`)
- **Symptom (distinct from E2!):** launched from a **VSCode Remote-SSH / plain-ssh**
  terminal, `vision_display` crashes at `cv2.namedWindow` with
  `Can't initialize GTK backend in function 'cvInitSystem'` and exit code 1. The
  detector then takes ~15 s to SIGKILL (TensorRT load blocks the SIGINT handler —
  benign). E2 was *headless OpenCV* (no GTK compiled in); **E4 is the opposite** —
  OpenCV *has* GTK, but a headless SSH shell has **no display server** (`$DISPLAY`
  empty), so the GUI has nowhere to draw. (Over the old full remote-desktop session
  it worked because that terminal inherited the desktop's `DISPLAY`.)
- **Root cause:** the GNOME/Xorg session runs on display **`:1`** (owned by the same
  `duburi-jetson` user; socket `/tmp/.X11-unix/X1`). A VSCode Remote-SSH integrated
  terminal starts with `$DISPLAY` unset and never inherits it.
- **Fix (host-local, in `~/.zshrc`):** when `$DISPLAY` is empty, auto-point GUI apps
  at the live local X socket — guarded so it never clobbers a real desktop terminal:
  ```sh
  if [ -z "$DISPLAY" ]; then
      for _d in /tmp/.X11-unix/X*; do
          [ -S "$_d" ] && export DISPLAY=":${_d##*/X}" && break
      done; unset _d
  fi
  ```
  `DISPLAY=:1` alone is enough (it falls back to the valid `~/.Xauthority` cookie).
  Verified on-device: a fresh headless zsh resolves `DISPLAY=:1` and `vision_display`
  opens its HUD without error.
- **Where the window appears:** on the **Jetson's** display `:1` — so you still *view*
  it via remote desktop / VNC, but **all editing + launching happens in VSCode
  Remote-SSH** (the latency win). NOT in the committed `.vscode/settings.json`:
  hardcoding `DISPLAY` there would break a teammate's *local* VSCode (forcing `:1`
  over their real `:0`). To drop remote desktop for *viewing* too, expose the
  annotated `…/image_debug` topic via `web_video_server`/Foxglove (browser over
  VSCode's auto port-forward) — not installed today; future task.

### E5. Payload CH340 has no `/dev/ttyUSB*` after carrier-board/SSD swap — **2026-07-10**
- **Symptom:** `lsusb` shows `1a86:7523 QinHeng Electronics CH340` (payload ESP32
  board is on the bus), but `start … -p payload_port:=auto` logs
  `[PAYLOAD] no port found (auto-detect excluded: set())` and `fire()` becomes a
  log-stub. `ls /dev/ttyUSB*` → nothing (only `/dev/ttyACM0`, the BNO085).
- **Root cause (two independent, both introduced by the 2026-07 carrier-board +
  SSD swap onto a fresh Tegra kernel):**
  1. **Kernel missing the CH341 driver.** The new `5.15.185-tegra` kernel shipped
     with `# CONFIG_USB_SERIAL_CH341 is not set` — no `ch341.ko` anywhere in
     `/lib/modules/$(uname -r)`. The CH340 enumerates on USB but the kernel never
     creates `/dev/ttyUSB*`, so pyserial `list_ports` (and thus payload
     auto-detect) sees nothing. `cdc_acm` (the BNO085 ESP32-C3) is unaffected — it
     needs no vendor driver, which is why the BNO worked and the payload didn't.
  2. **`brltty` steals the CH340.** Ubuntu's braille-display driver claims any
     `1a86:7523` via a udev rule and holds it through `usbfs` (interface driver
     shows `usbfs`), blocking `ch341` even once the module exists — the classic
     Arduino/ESP-on-Ubuntu trap.
- **Fix (durable, idempotent):** run **`tools/install_ch341_driver.sh`**. It purges
  `brltty` + its udev rule, builds `ch341.ko` out-of-tree against the running
  kernel headers (fetches the 5.15 `ch341.c`), `depmod`s it in (so `modules.alias`
  auto-loads it for `1a86:7523` on any port at boot/replug), loads it, and rebinds
  the already-attached board without a physical replug. Re-run after any kernel
  update / SSD reflash. Requires `/lib/modules/$(uname -r)/build`, gcc, make, curl.
- **Verified on-device (Orin Nano, 2026-07-10):** after the script, `/dev/ttyUSB0`
  (`usb-1a86_USB_Serial-if00-port0`, driver `ch341`) appears, `PayloadDriver`
  auto-detects + connects, and `bringup`/`start` re-connect the payload.
- **Self-diagnosing now:** `PayloadDriver.connect()` calls
  `_diagnose_missing_port()` — when the board is on the bus but has no tty node it
  logs the actual cause (`held by brltty (usbfs)` / `no ch341 driver bound` /
  `bound but no node yet`) and points at the script, instead of the old blank
  `no port found`.

---

## Pool-day 1 audit (2026-07) — FIXED

### P1. Arm intermittently reports "doesn't arm, 12 s crossed" — **FIXED 2026-07**
`DuburiClient._result_deadline` gave `arm`/`disarm`/`mission_reset` a **fixed 12 s** result
backstop (`_QUICK_CMDS`), but their real server budgets are larger — **arm ~18 s** (3 s ACK +
15 s `is_armed()` poll), **disarm ~26 s**. `send()` never applies the `COMMANDS` defaults, so
`goal.timeout` is `0.0` on the wire; the client raised `MoveTimeout` and cancelled the goal
**before arming completed** (worst on a slow post-baro-rezero EKF arm-readiness — the "sometimes").
**Fix:** the quick deadline is now a **floor, not a ceiling** — it reads the effective budget from
the same registry the server enforces: `stop/surface/unlock` keep 12 s; **arm → 30 s, disarm → 35 s**.
The disarm emergency-bail floor is preserved (bounded, just covers its real budget). `client.py`,
`test_client.py`.

### P2. Aborting an arm could strand the hull ARMED (fail-open) — **FIXED 2026-07**
`pixhawk.arm()` gained an abort hook (a goal cancel mid-arm must not leave a hull that arms a beat
later — ArduSub runs pre-arm checks **after** the ACK, so `is_armed()` reads False while the arm is
still pending). Two review findings hardened it: (a) arm now **clears the abort slate at entry** so a
STALE abort from a prior cancelled command can't insta-abort + disarm a fresh arm; (b) the abort
branch does a **verified** disarm (`_disarm_after_abort` re-sends DISARM across a window and requires
the disarmed state to HOLD) — **fail-closed** with a distinct `ABORTED_DISARM_UNCONFIRMED` reason if
it can't confirm, so no caller assumes "safe" on an unverified state. `NOT_ARMED_AFTER_ACK` also now
appends ArduSub's STATUSTEXT pre-arm reason. `pixhawk.py`, `duburi.py`, `test_pixhawk_helpers.py`.

### P3. Distance estimator absent → `calc_distance('stop')` returned a phantom 0.0 m — **FIXED 2026-07**
After the latched-topic refactor, `DistanceState.start()/stop()` always returned success even when
`distance_estimation_node` wasn't running (fire-and-forget, no ack), so a distance-gated move would
trust a clean 0.0 m. `stop()` now verifies a `distance_traveled` sample arrived **after** the bracket
started; absent → `success=False` with a clear reason. Latent today (no shipped mission gates on it).
`distance_state.py`, `test_distance_state.py`. *(distance/optical-flow is experimental, off by default.)*

### P4. Bin task (downward camera) dives to the pool floor, ignoring `set_depth` — **FIXED 2026-07-15 (pool feedback)**
A **downward SURGE-only** `vision.align` (the bin task: `lat` + `fwd`→Ch5 surge, **no** fill→depth
descent) **released Ch3 throttle (65535) while streaming NO depth setpoint**. With nothing asserting
depth-hold, ArduSub had no Ch3 authority to hold on, and the negatively-buoyant hull **sank to the
floor** — armed + ALT_HOLD, `set_depth` effectively ignored. `align_loop` streamed the depth setpoint
only on a downward *descent* (`stream_depth = use_vdepth or (downward and use_fwd)`); a surge-only align
had `use_fwd=False` → no stream, bare Ch3 release. The shipped `task_bin.py` hit this too.
**Fix (`motion_vision.py`):** stream the depth setpoint on **any** downward align
(`stream_depth = use_vdepth or downward`); with no descent, `fill_deficit` stays 0 so the 5 Hz block
re-streams the **constant** captured depth (holds `set_depth` via ArduSub's position controller — the
**same proven mechanism** the forward torpedo-standoff depth axis uses), and Ch3 is released
(`65535 if stream_depth else 1500`) so ArduSub's depth PID is the sole Ch3 consumer. The loss branch
re-asserts the hold (`if stream_depth`). Forward paths byte-unchanged; mavlink-reviewer clean.
`motion_vision.py`, `test_motion_vision.py` (4 new tests).

**Related trap — the depth-bound knobs are not tunable the way it looks.** `vision.max_depth_m` /
`vision.depth_ceiling` are (a) **overridden by per-call `align(max_depth_m=…, depth_ceiling=…)` kwargs**
(a per-call value beats any `ros2 param set`), and (b) re-set every run by a mission's
`set_vision_param(...)`. They are also **NEGATIVE metres** — a POSITIVE value (e.g. `0.6`) is a sign
error that silently reads as OFF (now warned loudly). They only bound the optional fill→depth **descent**;
they are **not needed to hold depth** — plain `set_depth` holds after this fix. **Build note:**
`build_dubomini.sh` is a plain `colcon build` (no `--symlink-install`), so editing
`competition_config.py`/`vision_tunables.py`/missions requires **`./build_dubomini.sh` then restart** —
a node restart alone does nothing.

---

## Forks we evaluated (so we don't revisit)

### `BumblebeeAS/ardupilot_fix` — STALE DUD (evaluated 2026-04)

* **Source:** https://github.com/BumblebeeAS/ardupilot_fix
* **State vs upstream:** **1 commit ahead, 3911 commits behind** `ArduPilot/master`. 0 stars, 0 forks. No CI configured.
* **The single commit** (`xelisce`, 2025-05-23, "hard code variables into file fix, passed all tests"): adds 9 unused declarations to `libraries/AP_DDS/AP_DDS_Client.cpp`. No semantic ArduSub change. No new mode, no new failsafe, no new MAVLink behaviour.
* **Verdict:** nothing to learn or pull. The fork name suggests a fix for something interesting but the diff is non-semantic. Stay on the upstream Sub-stable-V4.5.x branch documented in [`ardusub-canon.md`](./ardusub-canon.md).
* **Re-evaluate when:** the fork's `xelisce` author (or `BumblebeeAS` org) ships a second semantic commit. Until then, do not spend an evening "evaluating" this again.

---

## D16 — a Hailo stream abort leaves the detector a ZOMBIE (open, 2026-09-06)

**Observed on the vehicle.** Restarting the stack while a previous detector
still held the `VDevice` put the chip into `HAILO_STREAM_ABORT(63)`. The
detector node then:

* stayed **alive** — `pgrep` sees it, the node is listed, its subscriptions are
  up;
* logged `inference failed: HailoRTStreamAborted` on **every frame**, forever;
* published **zero detections**, indefinitely, with no escalation.

```
[ERROR] [DET  ] inference failed: HailoRTStreamAborted('Stream was aborted')
[HailoRT] [error] ... pipeline status is HAILO_STREAM_ABORT(63).
```

**Why it matters more than the error itself.** Every liveness check we have
passes: the process exists, the topics exist, the graph looks correct. Only the
detection RATE reveals it, and nothing was watching the rate. This is the
"absence is not zero" family again — a subsystem that has stopped working while
continuing to exist.

**Recovery today is a manual restart**, and the stack does come back cleanly
(299 inferences/interval, frame age 42.2 ms mean).

**The fix, not yet made.** After N consecutive inference failures the detector
should either re-open the `VDevice` or **exit**, so a supervisor restarts it. A
node that cannot do its job must stop claiming to be up. The new health surface
(`duburi_manager/health.py`) is what would surface it — `detector(rate_hz)`
returns FAILED on a zero rate — but nothing is polling it yet, and the detector
itself still needs the recovery path.

**Related:** the same restart also logged
`detector init FAILED: Failure in hailort driver ioctl` on the first attempt
and succeeded on the retry, so VDevice contention is transient but real. One
process, one `VDevice` (`detection/hailo.py:158-176`) is still the rule.
