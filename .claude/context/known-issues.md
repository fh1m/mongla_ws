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

---

## Forks we evaluated (so we don't revisit)

### `BumblebeeAS/ardupilot_fix` — STALE DUD (evaluated 2026-04)

* **Source:** https://github.com/BumblebeeAS/ardupilot_fix
* **State vs upstream:** **1 commit ahead, 3911 commits behind** `ArduPilot/master`. 0 stars, 0 forks. No CI configured.
* **The single commit** (`xelisce`, 2025-05-23, "hard code variables into file fix, passed all tests"): adds 9 unused declarations to `libraries/AP_DDS/AP_DDS_Client.cpp`. No semantic ArduSub change. No new mode, no new failsafe, no new MAVLink behaviour.
* **Verdict:** nothing to learn or pull. The fork name suggests a fix for something interesting but the diff is non-semantic. Stay on the upstream Sub-stable-V4.5.x branch documented in [`ardusub-canon.md`](./ardusub-canon.md).
* **Re-evaluate when:** the fork's `xelisce` author (or `BumblebeeAS` org) ships a second semantic commit. Until then, do not spend an evening "evaluating" this again.
