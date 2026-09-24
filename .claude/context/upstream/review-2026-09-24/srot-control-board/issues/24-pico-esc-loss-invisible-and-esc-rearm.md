# Pico or ESC loss is invisible to arming, failsafes and MAVLink, and a Bluejay ESC that resets never re-arms under a steady demand

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
There are two halves, and together they produce the vehicle state "ARMED, every verb accepted, no thrust, nothing reported".

1. **The ESP32 knows the thruster path is down and tells no decision-maker.** `thrusters.link_ok` is read only by the OLED, the buzzer, LoRa and the MAVLink ESC-presence summary. `arming::canArm()` does not check it, no failsafe reacts to it, and `SYS_STATUS` has no motor-output bit. The Pico reboot detector and the link-drop classifier write only to the OLED (`ui_log::set`). `reportEscNotDetected()` returns early when the link is down, so the one MAVLink report that could say something goes silent exactly then.
2. **After any ESC reset, Bluejay needs about 300 ms of continuous neutral before it re-arms, and nothing provides it.** This covers a Pico watchdog reset, an ESC power cycle (the second board's kill MOSFET) and a Bluejay stall-out. A vertical thruster that holds a steady non-zero demand (depth hold against buoyancy) never gets that window, so it stays dead. The Pico's stall detector sees it, but only raises a status bit and a warning.

This issue is a defect report plus a proposal ("thruster health as a first-class input").

## Evidence
At commit `f1d3ba9` (Bluejay at `bird-sanctuary/bluejay@0368d11`).

**Arming never looks at the thruster path.** `src/control/arming.cpp:12-60` checks `ARMING_CHECK`, IMU, leak, thruster battery and "calibrating", and nothing else:
```cpp
bool canArm(const char** reason) {
    // ARMING_CHECK == 0 → skip pre-arm checks (arm always allowed).
    if (g_params.arming_check < 0.5f) return true;
    bool imu_ok = false, leak = false, aux_fresh = false;
    ...
    if (!imu_ok) { if (reason) *reason = "IMU not healthy"; return false; }
```
`grep -rn link_ok src/` (ESP32 side) finds only `task_ui_status.cpp:109`, `task_buzzer.cpp:73`, `task_lora_sd.cpp:137`, `mav_stream.cpp:100-105` and the writer `task_dshot_rmt.cpp:136`.

**A Pico reboot and a link drop only reach the OLED** (`src/tasks/task_dshot_rmt.cpp:115-121` and `:206-213`):
```cpp
        if (fresh) {
            uint32_t up = thruster_link::picoUptime();
            if (prev_uptime > 3000 && up + 500 < prev_uptime) {
                char b[32]; snprintf(b, sizeof(b), "PICO REBOOT#%lu", (unsigned long)++reboot_cnt);
                ui_log::set(b);
            }
        ...
        if (prev_link && !link_now) {
            thruster_link::resyncRx();
            ...
            char b[32]; snprintf(b, sizeof(b), "Flap#%lu %s", (unsigned long)thruster_link::flapCount(), why);
            ui_log::set(b);
```

**The MAVLink presence report is silent when the link is down** (`src/comms/mav_stream.cpp:122`):
```cpp
    if (!(s.armed && s.thr_link_ok)) return;
```
`sendSysStatus()` (`mav_stream.cpp:281-286`) advertises gyro, accel, pressure, AHRS and the control bits, but no `MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS`.

**The Pico's stall detector only sets a flag** (`src/pico/main.cpp:353-357`):
```cpp
            bool commanded = armed && present && (rv[i] != 0) &&
                             (rv[i] > 1048 + STALL_MARGIN || rv[i] < 1048 - STALL_MARGIN);
            if (commanded && meas_mag < RPM_FAULT_RPM) {
                if (s_nospin_ms[i] == 0) s_nospin_ms[i] = now;
                else if (now - s_nospin_ms[i] > RPM_FAULT_MS) { fault |= (1 << i); status[i] |= TL_ST_ALERT; }
```
On the ESP32 that becomes "Thruster N STALLED" (`task_dshot_rmt.cpp:148-157`). The command is not changed.

**Bluejay's re-arm requirement** (`src/Bluejay.asm:607-612` @ `0368d11`):
```asm
; Make sure RC pulse has been zero for ~300ms
arming_wait:
    clr  C
    mov  A, Rcp_Stop_Cnt
    subb A, #10
    jc   arming_wait
```
After more than three consecutive failed starts, the ESC leaves run mode and goes back to `arming_begin`, which waits for that window again (`Bluejay.asm:1034-1038`, `:1063-1068`):
```asm
    ; Check max consecutive stalls and exit if stall counter > 3
    clr  C
    mov  A, Startup_Stall_Cnt
    subb A, #3
    jnc  exit_run_mode_is_stall
    ...
    ljmp arming_begin                   ; Go back and wait for arming
```
The round-2 review also reads `Modules/Isrs.asm:232-339` to show that in 3D mode both DShot 1048 and 48 decode as zero, so the Pico's neutral does count towards `Rcp_Stop_Cnt`. That part is cited from the review and was not re-read line by line here.

## Failure scenario
1. **Dead Pico at arm time.** The Pico is unpowered, or its UART is unplugged. The operator arms and it succeeds. The vehicle reports ARMED, and every `SROT_MOVE` is accepted and runs to its timeout. The board's own view of the thrusters is "link down", and only the OLED and buzzer say so. The companion sees nothing beyond `ESC_TELEMETRY` zeros, which mongla already knows can never prove a thruster is alive (958/958 zero frames with nothing attached).
2. **Pico watchdog reset at depth.** The Pico resets, possibly because of the debug `printf` issue in this batch. Its DShot lines idle, and the Bluejay ESCs lose signal and drop back to arming. The ESP32 keeps sending the depth-hold demand, which for the vertical group is a steady non-zero value. The ESCs never see 300 ms of zero, so the vertical thrusters stay off. The ESP32 shows "PICO REBOOT#1" on the OLED and nothing on MAVLink. The depth loop winds up against a thruster that is not there.
3. **Kill switch or second-board MOSFET cycle.** The same thing happens after thruster power comes back. (Round-1 draft 11 covers the kill switch being display-only.)
4. **A jammed prop.** Bluejay stalls out three times and returns to `arming_begin`. After the jam clears, the thruster stays dead for the rest of the dive, because the depth loop's demand never returns to exactly neutral.

## Suggested fix
**Firmware defect fixes (ESP32):**
- `canArm()`: refuse while `!thrusters.link_ok`, and while `esc_present != expected_mask` (from the frame's motor count), with a PreArm reason. Let `ARMING_CHECK=0` bypass the second check only.
- Send a STATUSTEXT CRITICAL on a Pico reboot and on a link-drop edge, not only `ui_log::set`. Use the existing "Flap#… PICO-SILENT / ESP32-STALL / LINK-NOISE" classification as the text.
- Add `MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS` to `sensors`, and clear its health bit when the link is down, when an expected ESC is absent, or when any stall fault is set.
- Remove the early return in `reportEscNotDetected()` for the link-down case, or send a distinct "thruster link down" line.
- Add a failsafe action for thruster-link loss while armed. The natural parameter is `FS_THR_ENABLE` = warn / surface / disarm, in the style of ArduSub's `FS_*_ENABLE` (see the single-failsafe-evaluator proposal, draft 39). With no thrust, "surface" means "disarm and float" on a positively buoyant hull, so the action needs a conscious default.

**Firmware fix (Pico): ESC re-arm.** When a channel has just reappeared (present again after absence, or on the first telemetry after a Pico boot), or has a stall fault, force DShot 1048 on that channel for 400 ms, then resume the commanded value. Bound the retries (for example 3 per minute) and report each one. This is what Bluejay's `arming_wait` needs, and the attitude and depth loops cannot provide it by themselves.

**Proposal: thruster health as a first-class input.** Treat the thruster path like the IMU and the barometer: a health bit in shared state, set by the DShot task from the link, presence, stall and reboot signals, and consumed by arming, the failsafe evaluator, `SYS_STATUS` and the move primitives (a move should not ACK completion with a dead thruster in its group). EDT (checklist item L7 in draft 37) would add ESC temperature and status to the same bit later.

## How to verify (bench)
Props off, vehicle restrained:
1. Unplug the Pico UART, then try to arm. **Today:** it arms. **After:** "PreArm: thruster link down".
2. Armed, pull the Pico's reset (or its UART) for 1 s. **Today:** nothing on MAVLink. **After:** a STATUSTEXT CRITICAL, `SYS_STATUS` motor-outputs unhealthy, and the configured failsafe action.
3. Armed in DEPTH_HOLD with a steady heave demand, power-cycle one ESC. **Today:** the ESC beeps its arming tones and then stays silent, and its RPM stays 0. **After:** the Pico holds that channel at 1048 for 400 ms, the ESC arms, and its RPM follows the demand again.
4. Injection: remove the 400 ms neutral window and confirm step 3 fails again.

## Severity: High
A vehicle with no thrust stays armed, accepts and "completes" moves, and tells neither the companion nor the ground station. After a brief ESC reset, the thrusters that matter most (depth hold) do not come back.

## Related
- PR #10 (ESC presence), PR #4 (Pico ESC voltage/current/temperature), PR #20 (mixer saturation feedback).
- Round-1 draft 11 (the kill switch is display-only). Cutting ESC power leads straight into the re-arm half of this issue.
- Draft 30 in this batch (the Pico debug `Serial.printf` can trip the Pico watchdog), which is one way to get a Pico reset.
- Draft 39 in this batch (single failsafe evaluator), where the link-loss action belongs.
- mongla_ws CLAUDE.md: "a message count can never prove a thruster is alive". This is the board-side half of that rule.

---
_Generated by [Claude Code](https://claude.ai/code)_
