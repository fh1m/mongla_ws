# Low-severity control-loop hygiene (eight small items)

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws, link to follow).

## Summary
Eight independent small defects, filed together. None is dangerous on its own. Each is a place where the code assumes something it does not check. Every item is verified at `f1d3ba9` and can be fixed separately.

## Evidence, failure scenario and fix, per item

### 1. Fixed `dt`, no overrun or jitter accounting, and control and DShot share a priority
`src/tasks/task_control_loop.cpp:275` uses `const float dt = CONTROL_LOOP_DT;` and `:952` uses `vTaskDelayUntil(&last, period);`. `include/config.h:360-361,384-385` puts `TASK_CONTROL_PRIO 5` and `TASK_DSHOT_PRIO 5` on the same core. Both are ready on the same tick, so FreeRTOS time-slices them in an arbitrary order, which adds 0–2 ms of random latency from control to output. When a cycle overruns, because of a flash stall or lock waits, the PIDs still integrate with 2 ms and no counter records it.
**Fix:** measure dt with `esp_timer_get_time()`, clamped to `[0.5, 3]×nominal`. Chain the tasks: control finishes, calls `xTaskNotifyGive(dshot)`, and DShot sends; alternatively give DShot priority 6 and have it wait on the notification. Keep `overrun_count`, `max_exec_us` and `max_period_us` for each task and publish them at 1 Hz (`NAMED_VALUE_FLOAT` or `SYS_STATUS.load`).

### 2. Euler-angle errors are fed straight into body-rate PIDs
`src/control/attitude_control.cpp:78-79,102,106-111`: `des_roll_rate = (tgt_roll - meas_roll) * ang_rll_p`, and the same for pitch and yaw. These are compared with body gyro `gx`, `gy` and `gz`. Away from level, Euler rates and body rates differ: at 30° pitch, a yaw-rate demand appears partly on the body x axis. Heading hold then couples into roll.
**Fix:** convert the desired Euler rates to body rates, `ω_body = W(φ,θ)·[φ̇,θ̇,ψ̇]`, which is three lines. Or compute the attitude error as a quaternion, as ArduSub does.

### 3. Gyro bias may be compensated twice
`src/drivers/bno085.cpp:98` enables `SH2_GYROSCOPE_CALIBRATED`, which the BNO bias-corrects dynamically. `src/tasks/task_sensor_read.cpp:123-125` subtracts a **stored** `gyro_offset` as well. That offset was measured by `calibration.cpp:206-218` as the residual of the calibrated output at calibration time, and it is persisted across boots. The BNO re-estimates its bias every boot, so the stored residual from an earlier session is then an error in its own right. Attitude comes from the BNO's own fusion and does not use this offset, so the rate loop and the angle loop disagree about the bias. The I-term absorbs the difference.
**Fix:** either use `SH2_GYROSCOPE_UNCALIBRATED` and own the bias, or drop the stored offset and trust the BNO. Do not do both. At the least, do not persist the offset across boots.

### 4. The depth stick target has no maximum
`src/control/depth_control.cpp:43-46` clamps only at 0: `s_target -= stick*MAX_CLIMB_MS*dt; if (s_target < 0) s_target = 0;`. A stuck or held-down stick pushes the target past the pool or rated depth, and the integrator follows it.
**Fix:** clamp to `[0, DEPTH_MAX]` (new param) and clamp to the current depth ± a leash, as ArduSub's `pos_control` does.

### 5. Rate feedforward is added outside the PID's anti-windup
`attitude_control.cpp:106-111`: `out = constrain(pid.update(...) + rat_*_ff * des_rate, -1, 1)`. The PID's saturation test does not include the FF term, so with FF > 0 the PID sees itself as unsaturated while the sum is clamped, and it winds up. All FF defaults are 0 today (`config.h:459`), so this is latent.
**Fix:** pass FF into `PID::update()` as a term that is part of `out_unsat`.

### 6. BATTERY_STATUS and SYS_STATUS describe the wrong batteries
`src/comms/mav_stream.cpp:273` sends **both** instances with `MAV_BATTERY_FUNCTION_ALL`. `:302` puts **PM1**, the electronics pack, into `SYS_STATUS.voltage_battery`, while the failsafe watches the **thruster** pack (`task_control_loop.cpp:664`). GCS "battery low" logic therefore watches the wrong pack. The host has to demultiplex by `id` (see the mongla docs).
**Fix:** use `MAV_BATTERY_FUNCTION_PROPULSION` for id 1 and `MAV_BATTERY_FUNCTION_AVIONICS` for id 0, and put the thruster pack into `SYS_STATUS`, or document the choice clearly.

### 7. `(int8_t)p[0]` in DO_MOTOR_TEST is undefined for out-of-range floats
`src/comms/mav_commands.cpp:371` has `int8_t motor = (int8_t)p[0] - 1;`. For `p[0]=257.0` the conversion is UB in C++. On Xtensa it truncates in practice (257 becomes 1), so this would pass the range check and spin **motor 1**.
**Fix:** `if (!(p[0] >= 1 && p[0] <= NUM_THRUSTERS)) return MAV_RESULT_DENIED; int motor = (int)p[0] - 1;`

### 8. Calibration on core 1 waits up to 20 ms for locks
`src/control/calibration.cpp` (`:98, :141, :151, :160, :176, :216, :230, :241, :247, :271, :302, :402`) uses the default `StateLock` timeout of 20 ms (`include/state_types.h:313`). This code is called from the 500 Hz control loop (`task_control_loop.cpp:286`). The `StateLock` comment itself says real-time callers pass a short timeout. Holds are µs-scale, so this rarely matters, but a contended lock stalls the loop for up to 10 cycles.
**Fix:** pass `pdMS_TO_TICKS(2)` at these sites, or keep a local `CAL_LOCK_TIMEOUT` constant.

## How to verify
- **Item 1:** the per-task `max_exec_us` and `overrun_count` telemetry itself. Then inject a 5 ms busy loop into control and confirm the counter increases.
- **Item 2:** a unit test at 30° pitch with a yaw-only demand: the body-rate demand must equal `W·[0,0,ψ̇]`.
- **Item 3:** leave the board still for 10 min across two boots and log the gyro mean. It must be 0 ± noise on both boots.
- **Item 4:** hold the stick down for 60 s in DEPTH_HOLD on the bench; the target must stop at `DEPTH_MAX`.
- **Item 5:** a unit test with FF=0.5 and a saturated plant: the integral must stay bounded.
- **Item 6:** in QGC's battery widget, instance 1 must show as propulsion.
- **Item 7:** `DO_MOTOR_TEST p1=257` must return `DENIED`.
- **Item 8:** grep in CI for `StateLock lk(g_state.mtx_[a-z]*);` in files linked into core 1.

## Severity: Low
Each item is latent or small today. Items 1 and 7 are the first to fix.

## Related
- PR #20 (windup)
- "The PID integrator limit is in error·seconds" in this batch (items 1 and 5 touch the same code)

---
_Generated by [Claude Code](https://claude.ai/code)_
