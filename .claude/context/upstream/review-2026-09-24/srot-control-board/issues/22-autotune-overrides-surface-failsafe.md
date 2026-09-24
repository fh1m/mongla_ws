# A running ATUNE/USER_5 autotune overrides every SURFACE failsafe, no mode change stops it, and when it finishes it disarms at depth

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
When a relay autotune is started with the `ATUNE` parameter or `MAV_CMD_USER_5`, `autotune_active` latches, and the tuner stays active for as long as that flag is set, **whatever the mode is**. The leak, low-battery, GCS-loss and companion-loss failsafes all act by writing `mode = SURFACE`. None of them clears `autotune_active` or calls `autotune::abort()`. On the next cycle the tuner still owns the output branch, so the SURFACE demands are never computed. The relay keeps driving at full authority and holds the tune depth (`at_depth0`). When the tune finishes, the loop **disarms the vehicle wherever it is**, which may be at depth with a leak in progress.

An operator who selects STABILIZE (or any other mode) cannot stop it either. Only disarm or the safety monitor (tumble/rate/NaN/depth runaway) ends it. The comment at `task_control_loop.cpp:530` ("Leaving AUTOTUNE mode / clearing ATUNE must actually STOP the tuner") holds only for the mode-selected path, where `autotune_active` is never set.

## Evidence
At commit `f1d3ba9`.

The trigger latches with no mode involved (`src/comms/mav_commands.cpp:544-547` and `:646-649`):
```cpp
        case MAV_CMD_USER_5: {  // start relay auto-tune
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) g_state.control.autotune_active = true;
            return MAV_RESULT_ACCEPTED;
        ...
        if (strncmp(name, "ATUNE", 16) == 0 && ps.param_value >= 1.0f) {
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) g_state.control.autotune_active = true;
```
Setting `ATUNE=0` does not clear it: there is no `else` branch.

The tuner gate is an OR with the latched flag (`src/tasks/task_control_loop.cpp:75`, `:508`):
```cpp
            in.autotune = c.autotune_active;
        ...
        bool at_active = (in.autotune || in.mode == FlightMode::AUTOTUNE) && in.armed;
```

The failsafe only rewrites the mode (`task_control_loop.cpp:691-696`):
```cpp
            if (fs && in.armed) {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) g_state.control.mode = FlightMode::SURFACE;
                in.mode = FlightMode::SURFACE;
                s_fs_surface = true;     // this surfacing is the failsafe's, not the pilot's
```

The tuner branch comes before `computeDemands()`, so SURFACE never runs, and completion disarms (`task_control_loop.cpp:839-857`):
```cpp
            if (at_active) {
                bool running = autotune::update(in.roll, in.pitch, in.yaw, ...
                fwd = 0; lat = 0;   // thr driven by the depth-tune phase
                ...
                    if (!running) {
                        g_state.control.autotune_active = false;
                        ...
                        // Finished (or aborted): DISARM so the sub sits safely.
                        g_state.control.armed = false;
                        if (in.mode == FlightMode::AUTOTUNE) g_state.control.mode = FlightMode::STABILIZE;
```
`autotune_active` is cleared in only three places: tune completion (`:849`), a safety-monitor abort (`:593`) and the disarm edge (`:335`). `grep -n autotune_active src/` finds no other writer.

## Failure scenario
1. In the pool, the operator arms and sets `ATUNE=1` (or the companion sends `USER_5`). The loop switches the mode to AUTOTUNE and starts the relay (`:511-527`).
2. The hull starts leaking. With `LEAK_EN=1` the leak branch sets `mode = SURFACE` and the operator sees "Failsafe: surfacing (leak)" (`:709`).
3. On the next cycle `at_active` is still true because `in.autotune` is true. `autotune::update()` keeps relaying roll/pitch/yaw torque and holds `at_depth0`. The vehicle **does not ascend**. The failsafe branch is skipped from then on because the mode is already SURFACE (`:629`), so nothing re-evaluates it.
4. The operator switches to STABILIZE to take over. `at_active` is still true, so nothing changes.
5. Several minutes later the last phase finishes. The loop disarms the vehicle, and it stays in SURFACE mode (the STABILIZE reset only applies when the mode is AUTOTUNE). It is now disarmed, flooding and at tune depth. The surface-disarm logic never ran, because the vehicle never reached 0.25 m.

The same holds for GCS loss and companion loss: a tune started by the companion that then crashes keeps tuning until it finishes, then disarms at depth.

Round-1 patch `0005-arming-refuse-to-arm-...` (not yet filed) refuses to *arm* with an autotune pending. It does not change anything once the vehicle is armed and tuning, so it does not close this.

## Suggested fix
- In the failsafe branch (`fs && in.armed`), and on **any** mode change away from AUTOTUNE, clear `g_state.control.autotune_active`, call `autotune::abort()` and set the local `at_active = false` in the same cycle. The safety-monitor path (`:588-601`) already shows the pattern.
- Make `ATUNE=0` clear `autotune_active` while armed too.
- Better, structurally: make the mode the only source of truth. `ATUNE`/`USER_5` should *select* AUTOTUNE mode, and nothing else should gate the tuner. ArduPilot runs autotune as a flight mode. Leaving that mode calls `AC_AutoTune::stop()`, which restores the original gains (`libraries/AC_AutoTune/AC_AutoTune.cpp:90-94` @ ArduPilot `ee0c343`):
  ```cpp
  void AC_AutoTune::stop()
  {
      // set gains to their original values
      load_gains(GainType::ORIGINAL);
  ```
- On completion, do **not** disarm in place. Go to the mode the operator was in before (or DEPTH_HOLD), and let the operator disarm.
- Long term, see the "single failsafe evaluator" proposal in this batch: failsafes should run *after* every tuner and scripted mode and preempt them.

## How to verify (bench)
In air, props off, `ARMING_CHECK=0`, `LEAK_EN=1`:
1. Arm, set `ATUNE=1`. Confirm that `AT_N`/`AT_TU` telemetry advances.
2. Pull GPIO35 high (the leak input).
3. **Today:** the mode reads SURFACE and "Failsafe: surfacing (leak)" arrives, but `AT_*` keeps advancing and `SERVO_OUTPUT_RAW` shows the relay pattern, not the SURFACE heave. Select STABILIZE: nothing changes.
4. **After the fix:** "Autotune stopped" is sent in the same cycle as the failsafe line, `AT_*` freezes, and the outputs show the SURFACE ascent demand.
5. Injection check: remove the new `autotune_active = false` in the failsafe branch and confirm step 3's behaviour returns.

## Severity: High
A leak or link-loss failsafe that fires during a tune is silently defeated, and the tune then ends by disarming the vehicle at depth. Autotune is run in the water, which is exactly where the failsafe matters.

## Related
- Round-1 draft 13 (the STATUSTEXT queue drops the newest line): the "Failsafe: surfacing" line is announced even though no surfacing happens.
- "Autotune abort keeps partially tuned gains live" (draft 29 in this batch): the same tuner, and the same ArduPilot pattern fixes both.
- "Proposal: a single failsafe evaluator" (draft 39 in this batch).
- The low-severity checklist in this batch, item L1: `DO_MOTOR_TEST` has the same precedence inversion over a failsafe, but it is bounded to 3 s.

---
_Generated by [Claude Code](https://claude.ai/code)_
