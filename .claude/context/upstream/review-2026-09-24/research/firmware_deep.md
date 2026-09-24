# SROT / Hengla firmware: round-2 deep review

**Scope:** `srot-control-board` @ `22f0ccb` (2026-09-24), `srot-esc-flasher` (docs only). I only read these repos; nothing was edited, committed or pushed.
**References I cloned and cite:** `bastian2001/pico-bidir-dshot` @ `b46f543`, `bird-sanctuary/bluejay` @ `0368d11`, `betaflight` @ `31fb41d`, `ArduPilot` (ArduSub, AC_AutoTune, AP_Logger), `earlephilhower/arduino-pico` (SerialUSB.cpp). I did not do the PX4 comparison.
**Nothing here was tested on hardware or in water.** A finding marked *unverified* is a result of reading the code and has not been measured.

---

## HIGH

### H1. A running autotune overrides every SURFACE failsafe, and changing mode does not stop it
`task_control_loop.cpp:507` `bool at_active = (in.autotune || in.mode == FlightMode::AUTOTUNE) && in.armed;`
`autotune_active` gets cleared in only three places: when the tune finishes (`:858`), when the safety monitor aborts (`:593`), and on disarm (`:335`). `ATUNE=0` clears it only while disarmed (`mav_commands.cpp:713`). When a leak, GCS-loss or companion-loss failsafe fires, it writes `mode = SURFACE` (`:690`). On the next cycle `at_active` is still true, so the output branch at `:849` (`if (at_active) autotune::update(...)`) runs before `computeDemands()`. The SURFACE demands are never applied. The relay tuner keeps driving at full authority. When the tune ends it **disarms at depth** (`:862`). The comment at `:529` ("Leaving AUTOTUNE mode … must actually STOP the tuner") is false for the ATUNE/USER_5 trigger path. An operator choosing STABILIZE also cannot stop the tune. Disarming is the only way out.
**Fix:** in the failsafe branch and on any mode change away from AUTOTUNE, clear `autotune_active` and call `autotune::abort()`. ArduPilot runs autotune as a flight mode. Leaving that mode calls `AC_AutoTune::stop()`, which runs `load_gains(GainType::ORIGINAL)` (`AC_AutoTune.cpp:91-94`).
**Bench check:** in air with props off, set `ARMING_CHECK=0`, arm, set `ATUNE=1`, then force the leak input (GPIO35 high, with `LEAK_EN=1`). The control loop still runs the tuner: the mode reads SURFACE and `AT_*` telemetry keeps advancing.

### H2. MOTOR_DETECT ignores `FRAME_REVERSE`, so on this hull it reverses every axis
The test pulse is `oneToDshot(test_throttle, in.dir[m])` (`task_control_loop.cpp:809`), which bypasses the `FRAME_REVERSE` negation at `:925`. Detect compares the gyro response against the unreversed mixer row `mixer::motorAngular()` (`calibration.cpp:346`). It then composes `motor_dir *= agree` (`:406`), which drives every conclusive motor to "positive command produces +eᵢ". In flight the demand is negated before mixing (`FRAME_REVERSE=1` on this hull, per the Mongla CLAUDE.md). After a detect run every axis therefore produces −d. That is the same failure as the 2026-08-07 flip, now reached from a run that reports SUCCESS. The rev-7 note says `FRAME_REVERSE` exists *instead of* per-motor signs, and `grep frame_reverse` shows only `task_control_loop.cpp:925` reads it.
**Fix:** multiply the expected vector by `(frame_reverse ? -1 : 1)` in detect and in the Motor Test path, or refuse MOTOR_DETECT while `FRAME_REVERSE=1`.
**Check:** a host-side unit test on the math. In water, dump `CAL mdir*` before and after, then do a MANUAL yaw check.

### H3. MOTOR_TUNE writes an inflated `MOT_SPIN_MIN` (used on every flight) and an RPM Ki in the wrong units
- `motor_tune.cpp:15` `RAMP_STEP 0.0020f` per 2 ms cycle is 1.0/s. The motor must exceed 200 rpm for `SPIN_HOLD_MS=200` (`:17`) before `s_out` is recorded, so the stored "break-away" is at least 0.2 above the real one, plus ESC start-up and filter lag. The pulse also goes through `mixer::oneToDshot(mtthr,1)` (`task_control_loop.cpp:822`), which already applies the old `MOT_SPIN_MIN` lift and the expo curve. The measurement is therefore circular: with any old floor the motor spins at s_out≈0.005 and the result is about 0.2–0.3 regardless. `finishAndSave()` writes that to `g_params.mot_spin_min` (`:61`) and persists it. The raw DShot path (the default, `RPM_LOOP=0`) uses it in `oneToDshot` (`mixer.cpp:112-127`), so the smallest non-zero demand becomes about 20–30 % thrust. That destroys the low-end resolution that precision alignment depends on. *Unverified magnitude.*
- `Ki = Kp/(0.5·Tu)` (`:157`) is in 1/s. The Pico uses `g_rpm_ki` as an "integral step per cycle" at 2 kHz (`pico/main.cpp:51,246`), which makes it 2000× too aggressive. Ki clamps at 1.0/cycle, which is 2000/s.
- `s_acc_n++` counts a motor whose PI phase failed (`:160`), so the kp/ki averages are diluted toward zero.
**Fix:** use a ramp-and-hold staircase. Record the level *at the first* rpm crossing minus about `ramp×latency`. Drive the pulse raw, bypassing lift, expo and battery scaling. Convert Ki with `Ki_cycle = Ki·dt_pico`. Count only valid motors.
**Check:** in air with props on and the vehicle restrained, run MTUNE, log `MOT_SPIN_MIN` before and after, and compare with a manual slow ramp while watching RPM on `connect --watch`.

### H4. Pico/ESC loss is invisible to arming, failsafes and MAVLink, and Bluejay then needs about 0.3 s of neutral to come back
- `thrusters.link_ok` is read only by the OLED, buzzer and LoRa (`grep link_ok`). `arming::canArm` (`arming.cpp:33-91`) does not check it, and no failsafe reacts to it. The Pico reboot detector posts only to the OLED (`task_dshot_rmt.cpp:158-160`, `ui_log::set`). The link-drop edge also goes only to `ui_log` (`:250-253`). A vehicle with a dead Pico arms, reports ARMED and runs every verb on the wire while producing no thrust. `reportEscNotDetected` returns early when `!thr_link_ok` (`mav_stream.cpp:122`), so it is silent too.
- After any Pico reset (watchdog), ESC power cycle (the 2nd-board kill MOSFET) or Bluejay stall-out, the ESC re-arms only after `Rcp_Stop_Cnt ≥ 10` (Bluejay `Bluejay.asm:607-612`, "RC pulse zero for ~300 ms"). Three failed starts send it back to `arming_begin` (`Bluejay.asm:1047-1068`). In 3D mode, 1048 and 48 decode as zero (`Isrs.asm:232-339`), so neutral does count. A vertical thruster that holds a steady non-zero demand (depth hold against buoyancy) never re-arms. The Pico's stall flag (`pico/main.cpp:353-357`) only produces a warning.
**Fix:** gate arming on `link_ok && esc_present == expected_mask`. Raise a STATUSTEXT CRITICAL on a Pico reboot or link drop. Set `MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS` health. Add a failsafe action (surface or disarm) for link loss while armed. On the Pico: when a channel is faulted or has just reappeared, force 1048 for 400 ms and then resume (bounded retries).
**Check:** unplug the Pico UART while armed, or power-cycle one ESC, and watch MAVLink. Today nothing arrives beyond ESC_TELEMETRY zeros.

### H5. BNO085 axis remap is done on Euler angles, so body roll saturates at ±90°. It breaks `style_roll`.
`bno085.cpp:112-117,292-293`: ZYX Euler angles are computed in the *sensor* frame, then roll and pitch are swapped and yaw negated. Body roll is therefore the sensor's `asinf` pitch, which is bounded to ±90°. At 120° of real body roll the output reads roll = 60° and pitch ≈ 180°. The swap is exact only for single-axis rotations. Conjugating by the mount rotation gives Z-X-Y order, not Z-Y-X. `stunt::update` holds the passive pitch axis with `(0 − meas_pitch)·ANG_PIT_P` (`stunt.cpp:37`). Halfway through a STYLE roll that becomes about −π × 4.5 ≈ −14 rad/s of pitch demand, so the rolling hull is told to pitch at full authority. `style_roll` runs on the board (movement STYLE → `stunt::start(ROLL…)`, `movement.cpp:109-112`). Combined roll and pitch in normal flight carry a second-order error. *Unverified in water.*
**Fix:** rotate the quaternion, `q_body = q_sensor ⊗ q_mount` with `q_mount` = 180° about (1,1,0)/√2 = (0, ½√2, ½√2, 0). Then take Euler angles once, or better, give the stunt code a quaternion or rate-integrated attitude.
**Check:** on the bench, roll the board by hand past 90° about the body x axis and watch ATTITUDE. Pitch will jump by 180°.

---

## MEDIUM

**M1. BNO085 over I2C on ESP32 is a documented bad pairing.** Adafruit lists the BNO08x as violating I2C setup time during clock stretching, and says it "does not work well" with ESP32 or ESP32-S3 ([Adafruit "Troublesome Chips"](https://learn.adafruit.com/i2c-addresses/troublesome-chips); root-cause analysis in [circuitpython#10034](https://github.com/adafruit/circuitpython/issues/10034)). That matches the "r8 freeze", the reset counters and the 50 ms Wire timeout (`main.cpp:108-116`). The bus is shared with the Bar30. Fix: move to SPI (the BNO085 supports SPI with INT/RST), which is a hardware revision. Interim: give I2C0 its own mutex shared with `bar30`, as `task_sensor_read.cpp:216` already suggests.
Also *unverified*: `sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_MAG)` (`bno085.cpp:152,200`) leaves gyro dynamic calibration off. The GRV yaw drift budget depends on gyro ZRO calibration, and CEVA's calibration procedure enables all three. Measure the drift with and without `SH2_CAL_GYRO`.

**M2. The move brake is an open-loop impulse sized from the commanded speed, not the reached speed.** `PH_BRAKE` pushes `-s_uf·g·s_speed` (`movement.cpp:192`) for `brakeMs(s_speed)` (`:187`). The global-timeout and abort paths size the duration from `s_cur_speed` but still push with `s_speed` (`:143,177`). The hull's velocity after 0.4 s and after 10 s at the same command differ by roughly the hull time constant (seconds), but the brake is identical, so short legs over-brake and back up. Fix: brake with a first-order velocity model `v̂ += (k·u − v̂)·dt/τ` (τ from a pool coast-down), or with the downward-camera velocity once uplinked.

**M3. No depth ceiling (max-depth fence).** DIVE clamps only the lower bound (`movement.cpp:106`). DEPTH_HOLD's stick can push the target down without limit (`depth_control.cpp:44-47`). The runaway guard follows the setpoint (`task_control_loop.cpp:563`). ArduSub has `FENCE_ALT_MIN`/`SURFACE_DEPTH`. Add `DEPTH_MAX` (for example pool depth minus 0.3 m) and clamp every setter.

**M4. Direction reversal near neutral goes through Bluejay's brake-and-restart path.** In bidir mode a sign change sets `Flag_Dir_Change_Brake`. Power is limited to `Pwm_Limit_Beg` until the motor falls below about 5000 eRPM (roughly 714 T200 rpm). Then an 18-rotation initial-run phase follows (`Bluejay.asm:963-990`), and the motor stops and restarts below about 1330 eRPM (`:951`). Vertical thrusters that dither around zero in depth hold or roll hold get a direction-dependent dead zone and delay, so expect a small limit cycle. *Unverified.* Mitigation: hysteresis on sign flips in `oneToDshot`, or a small `MOT_SPIN_ARM` idle, and measure with RPM logging.

**M5. The Pico debug `Serial.printf` can block core 0 for up to 1 s, which trips the 250 ms watchdog.** In arduino-pico `SerialUSB::write` blocks while `tud_cdc_connected()` and the buffer is full, with a 1 s timeout (`SerialUSB.cpp:156-176`). About 400 B go out every 500 ms (`pico/main.cpp:396-412`). A laptop that holds the Pico's port open without reading (a paused terminal, a crashed script) causes a watchdog reset and then H4. Fix: send only when `Serial.availableForWrite() >= len`, or drop the output.
Related interlock: the Bluejay `init_no_signal` path enters the bootloader if the line stays **high** for about 150 ms (`Bluejay.asm:496-508`). The bidir idle is high with a pull-up (`bidir_dshot_x1.cpp:83,131`). A Pico stall that holds the line high goes to the ESC bootloader, which exits on a timeout but costs re-arm time. The 250 ms watchdog is what currently prevents this, so **do not raise `wdt_begin(250)`**. Better: drive the pins low on a stall.

**M6. An aborted autotune leaves the partially tuned gains live.** `abort()` means "keep gains tuned so far" (`autotune.cpp:495`), and `applyGains` writes `g_params` as each phase finishes (`:214-223`). A safety-monitor abort ("rate limit") during an angle phase keeps the rate gains that were just derived, and the next operator `PREFLIGHT_STORAGE` persists them. The ArduPilot pattern snapshots the original gains, applies the tuned set only on completion, and restores on stop (`AC_AutoTune.cpp:91-94,134-146`). The relay also switches on the raw gyro sign with no relay hysteresis (`autotune.cpp:423`). Åström–Hägglund uses a hysteretic relay, so as written chatter drives the plant and is filtered only in the measurement.

**M7. SD log is not usable for tuning, and it does not really run at 20 Hz.** A `Record` holds quaternion, gyro, depth, DShot values, mode, armed and two voltages (`sd_log.h:16-26`). It has no setpoints, no PID P/I/D/FF terms, no depth target, RPM, events, failsafe reason, `link_ok` or leak, and no schema beyond a record size. The loop shares a task with blocking `LoRa.endPacket()` (`lora_mission.cpp:91,98,109`), a 70 ms busy RX window (`task_lora_sd.cpp:151-154`) and two relay frames every 120 ms, so the effective log rate is probably about 5 Hz (*unverified*). Write failures are never detected (`s_ok` never drops). With log999 present, `FILE_WRITE` truncates it (`sd_log.cpp:25-29`). Compare ArduPilot: self-describing `FMT` records (`LogStructure.h:189`), per-loop PID logs (`AC_AttitudeControl_Logging.cpp:12,31`), `CTUN` with target and actual depth (`ArduSub/Log.cpp:276`), and `ERR`/`EV` events on every failsafe (`failsafe.cpp:54,282`).

**M8. Unpinned libraries on the safety path.** `lib_deps = https://github.com/bastian2001/pico-bidir-dshot.git` and `adafruit/Adafruit BNO08x` have no version (`platformio.ini`). The Pico source admits that enum names may differ between versions (`pico/main.cpp:18-22`). The DShot library changes build by build. Pin both to a tag or commit.

---

## LOW

- **L1.** An active `DO_MOTOR_TEST` takes precedence over a failsafe SURFACE (`task_control_loop.cpp:799` runs before the mixer branch) for up to the 3 s keep-alive (`mav_commands.cpp:439-442`). AGENTS.md states the opposite precedence.
- **L2.** The leak input is one `digitalRead` at 10 Hz with no debounce (`analog_mon.cpp:64`). A single glitch surfaces the vehicle and disarms it 5 s later. ArduSub latches the event and logs it (`failsafe.cpp:264-301`).
- **L3.** Barometer auto-zero after a reboot while submerged shallower than about 0.9 m (below 1100 mbar) latches the current depth as the surface (`task_sensor_read.cpp:93-111`). This only happens when no calibration is stored.
- **L4.** Control and DShot have equal priority 5 on the same tick (`config.h:361,385`), so their order is not guaranteed and adds 0–2 ms of transport jitter. Use `xTaskNotifyGive` from control to DShot.
- **L5.** SROT_MOVE parameters are range-unchecked float→unsigned casts: `(uint32_t)(primary*1000)` (`movement.cpp:129,136`), `(uint8_t)p[3]` for a negative ARC rate (`mav_commands.cpp:397`). These are undefined behaviour in C++. There is also no cap on STYLE count or dive depth.
- **L6.** `millis() < test_expire` (`task_control_loop.cpp:794`) and `millis() < s_recovery_until` (`bno085.cpp:240,345`) are not wrap-safe. That only matters at 49.7 days.
- **L7.** EDT is never enabled: no DShot cmd 13 is sent. The `d=`/EDT branch (`pico/main.cpp:309-318`) is dead, and the comment claiming Bluejay interleaves EDT frames is wrong. Betaflight sends it repeated 10× (`dshot_command.c:196-205`). This also leaves ESC temperature, voltage, current and STRESS unused.
- **L8.** Doc drift: `srot-esc-flasher/docs/ESC_FLASHING.md:15` says the Pico "drives that motor open-loop until an ESC reports telemetry". No such fallback exists. `:121` says `RPM_LOOP` defaults to 1, but `config.h:224,601` makes it 0.
- **L9.** The ESP32 never checks the Pico's echoed `seq` (`thruster_link.cpp:185-188`). A Pico applying stale commands looks healthy. The cfg frame's floats are not NaN-checked on the Pico (`pico/main.cpp:487-489`). This is harmless while `RPM_LOOP=0`.
- **L10.** The IMU going invalid mid-TURN gives a frozen yaw, so the turn never completes. It brakes at the 60 s default timeout and then reports ACCEPTED (`movement.cpp:61,176`). This falls under the known "ACK at 100 %" item.

**Checked and correct** (compared against the references): DShot300 timing is 40 PIO cycles at 12 MHz = 3.33 µs/bit, with 1-bit high 70 % and 0-bit 35 % (`bidir_dshot_x1.pio:12-14`). Telemetry sampling is at 5/4 rate (32 cycles/bit). The inverted checksum matches (`bidir_dshot_x1.cpp:170-177`). GCR decode and checksum 0xF match betaflight `dshot.c:206-221`. eRPM = 60e6/period and RPM = eRPM/7 for the 14-pole T200. Both 3D bands, 1049–2047 and 48–1047 with each band running low to high, match Bluejay `Isrs.asm:232-256`. Start-up sends DShot 0 before the link comes up. The Pico's 150 ms link timeout sends 0. The UART framing (CRC16-CCITT, fixed length, resync on CRC failure) is sound.

---

## Failsafe matrix (as coded)

| Trigger | Detection / latency | Action | Terminal state | Gaps / conflicts |
|---|---|---|---|---|
| Leak (`LEAK_EN=1`) | GPIO35 at 10 Hz, no debounce; ≤100 ms | mode→SURFACE, zero translation and yaw, depth target 0 (open-loop 0.35 ascent if no baro) | disarm after 5 s shallower than 0.25 m (baro OK only) | Defeated by running autotune (H1). Test override wins (L1). `LEAK_EN` defaults to 0 (known). |
| Thruster battery low | ESP-NOW V < `FS_BAT_VOLTAGE` for 3 s, armed only | SURFACE | as above | Inert without the 2nd board. Electronics pack (PM1) has **no** action. |
| GCS heartbeat lost | any heartbeat older than 5 s | SURFACE | as above | Autotune (H1). Fires immediately if armed and no heartbeat was ever seen (`gcsLinkOk` returns false when never seen). |
| Companion lost | companion seen, then older than 5 s | SURFACE | as above | Autotune (H1). |
| Pico link lost | ESP32 500 ms (flag only); Pico 150 ms → DShot 0 | thrust stops; **no mode, arm or MAVLink action** | stays armed, silent | H4 |
| Pico reboot / ESC reboot / ESC stall-out | uptime backwards (OLED only) / not detected | none. ESC needs ~0.3 s neutral to re-arm | thruster dead while demand ≠ 0 | H4 |
| Control-loop stall | 10 DShot cycles (20 ms) | disarmed frame to the Pico, not latched | resumes on next publish | Counts cycles, so a whole-core-1 stall (e.g. a 50 ms I2C timeout at priority 6) holds the last command until the Pico's 150 ms timeout. |
| IMU lost | not valid for 300 ms | attitude demands zeroed; depth and translation kept | armed | TURN never completes (L10). No surfacing. |
| Barometer lost | `baro_valid` false or stale for 1.5 s | DEPTH_HOLD/AUTO/PATTERN→STABILIZE (throttle = decayed stick ≈ 0); SURFACE goes open-loop | armed, no depth control | Move ACKs ACCEPTED (known). Held depth used for up to 1.5 s. |
| Tumble / rate / NaN / depth runaway | every cycle, **automatic modes only** | disarm | disarmed | No check in STABILIZE, DEPTH_HOLD or MANUAL. ArduSub has a crash check at 30° error for 2 s (`failsafe.cpp:372-422`). |
| Kill switch (2nd board) | ESP-NOW | display only (known) | — | Cutting ESC power leads into H4. |
| ESP32 panic / brownout / DTR reset | reboot | Pico 150 ms → 0; board returns **disarmed**, MANUAL | disarmed, floating | Barometer auto-zero at shallow depth (L3). Reset reason goes to the OLED and MAVLink. |
| Internal pressure / temperature | — | none | — | ArduSub `failsafe_internal_pressure_check` / `_temperature_check` (`failsafe.cpp:202-261`). |

---

## Owning the stack: upgrade proposals

| # | Proposal | Cost | Benefit |
|---|---|---|---|
| 1 | **Single failsafe evaluator with action params** (the ArduSub `FS_*_ENABLE` = warn/surface/disarm pattern). It runs *after* every mode and tuner decision and preempts tuners, tests and scripted modes. Add Pico-link and ESC-presence triggers. | 1–2 days plus bench | Closes H1, H4 and L1 structurally. One place to audit. |
| 2 | **Thruster health as a first-class input**: arming gate, `SYS_STATUS` MOTOR_OUTPUTS bit, STATUSTEXT on reboot or link loss, and Pico-side re-arm (neutral 400 ms on fault) | 1 day, both firmwares | No more "armed and silent"; recovers from a jammed prop. |
| 3 | **Quaternion mount transform**; stunts on quaternion or rate integration | half a day plus tests | Correct attitude at any angle (H5). |
| 4 | **BNO085 on SPI** (next board revision); meanwhile an I2C0 bus mutex | PCB revision | Removes the r8/reset class of faults (M1). |
| 5 | **Autotune as a mode with original-gain snapshot and restore** (the AC_AutoTune pattern), hysteretic relay, and gains applied only on completion | 1 day | Safe aborts (M6, H1). |
| 6 | **Fix MOTOR_TUNE** (staircase, raw pulse, Ki units) and forbid it writing `MOT_SPIN_MIN` without operator confirmation | half a day | Stops the silent loss of low-end resolution (H3). |
| 7 | **Self-describing binary log** (FMT header, 100 Hz attitude and PID terms, 20 Hz depth, RPM, event records), in its own task; LoRa TX made non-blocking via the DIO0 TX-done interrupt | 2–3 days | Tuning from logs, plus post-incident forensics (M7). |
| 8 | **Deterministic pipeline**: sensor → control → DShot chained with task notifications | half a day | Removes 0–2 ms of jitter (L4). |
| 9 | **Enable EDT** (cmd 13 ×10) and forward ESC temperature, voltage, current and stress into `ESC_TELEMETRY` | half a day | Real ESC health; detects desyncs. |
| 10 | **Pin lib_deps**; CI builds all three envs | 1 hour | Reproducible safety-critical builds (M8). |
| 11 | **Velocity-model brake and `DEPTH_MAX` fence** | 1 day | Repeatable leg ends and a bounded dive (M2, M3). |

Overall: the DShot and wire-level implementation is correct against betaflight and Bluejay. The risk sits in the layers above it: precedence between tuners and failsafes, calibration routines that ignore frame configuration, and Pico/ESC health that is never reported to the vehicle's own decisions.
