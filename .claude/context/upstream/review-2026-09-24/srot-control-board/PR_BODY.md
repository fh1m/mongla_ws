# Safety: output-path watchdog, IMU-loss demands, link-age race, armed-command refusals, arm-into-manoeuvre

Five fixes, one commit each, on top of `f1d3ba9`. They come from the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws). Each one closes a path where the board keeps driving thrusters, or starts driving them, when it should not.

| commit | what | why |
|---|---|---|
| 1 `dshot:` output-path watchdog | The control loop bumps `ThrusterState::out_seq` under the same lock as `throttle[]/norm[]/armed`. The DShot task counts the cycles in which it has not moved. After `CTRL_OUT_STALE_CYCLES` (10, i.e. 20 ms) it sends the disarmed stop. There is one edge-triggered STATUSTEXT each way, and it does not latch. | `loop_stamp_ms` was written and never read. A stalled control task kept sending fresh CRC-valid **armed** frames, so the Pico's 150 ms link failsafe never tripped and the last demand was held indefinitely. The watchdog counts cycles, not milliseconds, because a Core-0 flash write pauses both Core-1 tasks together and must not look like a stall. `loop_stamp_ms` is removed. |
| 2 `control:` IMU lost → attitude control off | While the debounced IMU-loss flag is set, roll/pitch/yaw demands are zeroed after all controllers and feedforward, and `attitude::reset()` runs every cycle. Depth and translation are kept. MANUAL is exempt (its sticks go straight through). | Zeroing the gyro while the frozen angle stayed turned the frozen angle error into a **constant torque**, while the GCS said "holding last attitude". |
| 3 `mav:` link-age race | Stamps and seen-flags are `std::atomic`. A writer stores the stamp before `seen`. A reader loads the stamp before `millis()` and uses the signed age `(int32_t)(now - stamp)`; a negative age counts as fresh. | Evaluation order is unsequenced across cores. A newer stamp stored between the two reads wrapped the unsigned age to about 4.29e9, which read as "companion lost" and **latched SURFACE**. The old writer order also let a reader see `seen && stamp == 0`. |
| 4 `mav:` refuse reboot / param reset / calibration while armed | `disarmedGate()` returns `MAV_RESULT_TEMPORARILY_REJECTED` plus a STATUSTEXT. A 5 ms lock miss refuses too, rather than guessing. | `PREFLIGHT_REBOOT` did `ESP.restart()` mid-dive. `PREFLIGHT_STORAGE` p1=2 reset FRAME_REVERSE and the motor directions live. BARO_ZERO at depth made the surface-disarm logic disarm **at depth**. Gyro calibration averaged motion. |
| 5 `arming:` refuse to arm into a manoeuvre mode | `canArm` refuses STUNT, PATTERN, AUTOTUNE, MOTOR_TUNE, MOTOR_DETECT and a pending `autotune_active`, each with its own PreArm reason. `ATUNE=0` now clears a pending trigger while disarmed. | USER_1..5 queue these modes while disarmed, so a yaw spin fired at full rate the moment the vehicle armed. |

## ⚠ Behaviour changes to check
- **MOTOR_TUNE is now "arm, then select"**; the old flow was "select, then arm". The OLED hint says so. mongla_ws `autotune` and `motor_tune` already arm first. **Check Bondor's Setup flow**, which was not checked.
- While the IMU is lost, a running STUNT, PATTERN or TURN cannot finish. It sits at zero attitude demand until its timeout or a mode change.

## Verification status: be honest about it
- **Compiled: NO.** The review container could not reach the PlatformIO registry. All four changed `.cpp` files pass `g++ -std=gnu++17 -fsyntax-only` against Arduino/FreeRTOS stub headers and the vendored MAVLink headers. We confirmed the check can fail by planting a deliberate typo, which it caught. Only the Pico backend branch of the DShot task was syntax-checked; the RMT branch, where only includes moved, was not. **Please run `pio run` for every env before merging.**
- **Bench checks** (thrusters off, props clear):
  1. Freeze the control task (e.g. a debug param that spins it for 100 ms). The thrusters stop within 20 ms, "Control loop stalled - thrusters stopped" appears, and output resumes after.
  2. Do a live PARAM_SET and an autotune save while armed on the bench. There must be **no** false "stalled".
  3. Unplug the BNO085 I2C lines while armed in STABILIZE. There is zero attitude output and the new STATUSTEXT appears.
  4. Send PREFLIGHT_REBOOT, PREFLIGHT_STORAGE(2) and PREFLIGHT_CALIBRATION(baro) while armed. Each returns TEMPORARILY_REJECTED.
  5. USER_1 (stunt) while disarmed, then arm. Expect "PreArm: STUNT queued - select a flight mode".

Related: #6 (the joystick arm path, deliberately untouched), #15, #25.

---

🤖 Generated with [Claude Code](https://claude.com/claude-code)
