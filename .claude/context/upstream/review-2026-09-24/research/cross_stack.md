# Mongla cross-stack review: seams, failure modes, time, latency

Read-only, 2026-09-24. Firmware lines refer to HEAD of branch `claude/zealous-pascal-sdgidr`
(5 commits ahead of `origin/main`). Everything is **traced in code**, not observed, unless a
measurement is cited.

## 0. The findings that matter most (new, not in the "already found" list)

| # | Finding | RPN |
|---|---|---|
| A | **A failsafe-cancelled move is reported as a completed move.** When the board leaves AUTO for any reason (leak/battery/companion failsafe → SURFACE, baro loss → STABILIZE, disarm), `movement::cancel()` runs (`task_control_loop.cpp:479`). `mv_done_seq` is then latched as if the move had completed (`:980`), and `updateMove` sends `COMMAND_ACK ACCEPTED, 100` (`mav_stream.cpp:623-625`). The host maps that to `SUCCEEDED` without checking the mode or arm state (`srot_fc.py:917-922`). So the mission moves on to its next leg. | 280 |
| B | **The next `SROT_MOVE` pulls the board out of SURFACE.** The firmware sets `c.mode = AUTO` whatever the current mode is (`mav_commands.cpp:393`), and the host's `move()` has no SURFACE guard (`srot_fc.py:867-886`). The vision verbs *do* have that guard (`vision_verbs.py:147-163`). With a persistent leak, each leg flips to AUTO for about 2 ms and then back to SURFACE, and A reports it as a success, so the whole mission "completes". With low thruster battery, `s_bat_low_since` is reset while in SURFACE (`task_control_loop.cpp:747`), so each new move gets at least 3 s of AUTO: the vehicle re-dives. The operator's own `surface` verb is undone the same way. | 240 |
| C | **A Pico reset is invisible to the companion, and RPM goes stale.** The reboot and the link flap reach only the OLED, the buzzer and the LoRa flags (`task_dshot_rmt.cpp:158-161,246-253`). `reportEscNotDetected` returns early when the link is down (`mav_stream.cpp:122`). RPM is only written while `fresh` (`task_dshot_rmt.cpp:167-176`), so during the outage ESC_TELEMETRY keeps sending the *last* non-zero RPM. The board stays armed with its loops integrating, and thrust comes back as a step when the link returns. A common cause is documented: a 5 V dip resets the Pico *and* glitches the USB-UART bridge (`HARDWARE.md:90-96`). | 252 |
| D | **The leak and battery failsafes are off as configured, and only a report-only tool says so.** `LEAK_EN` defaults to 0 (`params.cpp:144`) and was measured at 0 on this vehicle (`srot_protocol.py:973-976`). With it off the leak input is not even read (`task_sensor_read.cpp:119`). `ESPNOW_EN` defaults to 0 (`params.cpp:269`), which disables the thruster-battery failsafe and the kill-switch state. `SAFETY_GATES` is consumed only by `srot_connect.py:606`, which always exits 0. `bringup_check` checks only FS_GCS (`bringup_check.py:827-851`), and `arm()` checks neither (`srot_fc.py:501-518`). | 240 |
| E | **Recovering the link reboots the flight controller.** Opening the tty asserts DTR, which resets the ESP32 (`port_guard.py:5-24`). The manager never reconnects: the reader loop just logs faults (`auv_manager_node.py:1284-1296`, and its message says "the link may be FINE", which is wrong for an unplugged cable). There is no respawn and no systemd unit. Any automated recovery would reset an armed board mid-dive. Any recovery design has to fix DTR in hardware first. | 150 |
| F | **Board IMU stamps are the transmit tick, not the sample time.** `sendAttitude(s, now)` and `sendScaledImu(s, now)` use `millis()` from `update()` (`mav_stream.cpp:767-812`), while `imu_stamp_ms` already exists (`state_types.h:130`). The "board clock is perfect, sd 0.000" measurement (system-harmony §4) is the 100 Hz scheduler, not the sensor timing. | 105 |
| G | **A backward clock step on the Pi freezes the EKF.** `Retro.run` refuses any event older than the oldest in its buffer (`retro.py:76-78`), IMU predictions included (`localization_node.py:283`, return value ignored). A backward step of more than 2 s refuses everything until wall time catches up: the EKF is dead for \|step\| seconds. | 126 |

## 1. FMEA, whole system

S = severity, O = occurrence at a pool session, D = detectability (10 = undetected). Each 1-10;
RPN = S×O×D. "FS" = board failsafe path `task_control_loop.cpp:636-747`. GCS_FAILSAFE_MS =
5000 (`config.h:648`); surface-disarm needs `depth_ok` and < 0.25 m for 5 s
(`:729-738`, `config.h:662-663`).

| Failure | Board detects (latency) | Host detects (latency) | Action → vehicle | Gap | S | O | D | RPN |
|---|---|---|---|---|---|---|---|---|
| USB-C unplug | companion HB silence, **5 s** (`mav_commands.cpp:107-110`) | reader faults at once, logged as "link may be FINE"; `link_alive` 3 s (`srot_fc.py:90,487`) | FS → SURFACE → disarm 5 s after reaching < 0.25 m | no reconnect. Reconnecting resets the FC (E). With no baro, pushes up at 0.35 forever, by design (`:219,720-724`), with no time bound | 7 | 4 | 3 | 84 |
| Pi brownout | same, 5 s | none (host is dead) | same | **common cause**: a 5 V dip also resets the Pico (C) and the bridge. Electronics pack is unmonitored (PM1 reads ~1.35 V per CLAUDE.md), so there is no warning | 7 | 5 | 6 | 210 |
| Pi kernel panic | 5 s | none | same | no Pi hardware watchdog in `provision_vehicle.sh` | 6 | 2 | 3 | 36 |
| Manager crash | 5 s (HB comes from `heartbeat_tick`, 2 Hz, `auv_manager_node.py:1549`) | none; not respawned | same | a restart opens the port → **FC reboot** (E) | 6 | 3 | 3 | 54 |
| Mission crash | nothing: the manager keeps the HB alive | action goal runs to its end | AUTO station-keeps, **stays armed** | already found (no mission keep-alive) | 7 | 4 | 7 | 196 |
| Hailo hang | n/a | no inference watchdog. Verb zeroes lateral/forward by ≤0.8 s, LOST at 1.0 s (`motion_vision.py:315-318,359`) | DSL runs its LOST fallback (search) blind | `seeing` is latched, so it stays `ok` (`detector_node.py:550-559`). "Pipeline dead" reads as "target absent" | 5 | 3 | 6 | 90 |
| Camera USB drop | n/a | `_consec_fail` → `is_healthy` False, logged `BAD` (`camera_node.py:604`) | verbs → LOST/NO_CAMERA | retries only at startup (`camera_node.py:199-219`), never re-opens mid-run | 5 | 4 | 4 | 80 |
| ESP32 reset while armed | Pico link timeout **150 ms** → DShot stop (`pico/main.cpp:37,202-204,228`) | `time_boot_ms` drop at the 0.5 s telemetry tick (`srot_fc.py:1086-1123`) → aborts the command, reapplies rates and gain (`auv_manager_node.py:2033-2054`) | thrusters off; boots disarmed; `RST:` reason in STATUSTEXT | aborts only the *command*, not the mission; later legs fail as "arm first". Acceptable | 6 | 3 | 2 | 36 |
| Pico reset | link-down after 500 ms (`config.h:200`), shown on the OLED only | **none** (C); stale RPM | thrust drops, board stays armed; on relink demand steps back in; ESC may need a zero-throttle re-arm (Bluejay, unverified) | invisible; moves report success | 7 | 4 | 9 | 252 |
| One ESC dead / thruster stall | Pico fault after 300 ms no-spin (`pico/main.cpp:63-64`) → "Thruster N STALLED" ERROR (`task_dshot_rmt.cpp:180-198`), **never disarms** | "LOST telemetry" is parsed (`srot_fc.py:1039`); "STALLED" is not | mixer keeps commanding it; loss of authority (on the 5-thruster CAD hull, a lost DOF) | host has no reaction to STALLED | 6 | 4 | 5 | 120 |
| IMU failure | 300 ms debounce → attitude control off, **still armed** (`task_control_loop.cpp:760-790`); SYS_STATUS health cleared | `_ahrs_healthy` ≤ 0.5 s (`srot_fc.py:2086`) | an AUTO move keeps going with no heading hold, then ACK ACCEPTED (A) | host arms on it but does not abort a running move | 6 | 2 | 4 | 48 |
| Baro failure | `DEPTH_STALE_MS` 1.5 s or jitter → AUTO forced to STABILIZE (`:353-375`) | `_baro_healthy` ≤ 0.5 s | move cancelled, **reported ACCEPTED** (A); vehicle neutral at depth. In SURFACE, open-loop ascent with no end | A; no bounded end state | 8 | 4 | 7 | 224 |
| Leak | **disabled** (`LEAK_EN=0`, D). If enabled: < 2 ms → SURFACE (`:641`) | health reporter FAILED (LEAK_EN=0), log only | none as configured; if enabled, SURFACE, then the next leg drags it into AUTO (B) | D + B | 10 | 3 | 8 | 240 |
| Low batt, thruster pack | needs `ESPNOW_EN=1`; 3 s hold (`config.h:645`) → SURFACE | none | SURFACE; re-dive on each move (B) | D + B | 7 | 4 | 7 | 196 |
| Low batt, electronics | **nothing** | nothing (only `vcgencmd` at bringup) | Pi brownout → the row above | no failsafe on either side | 7 | 3 | 8 | 168 |
| LoRa loss | none: the companion keeps `gcsLinkOk` (`task_lora_sd.cpp:27`) | Bondor: cached state 1-5 s, then "LoRa lost" (`groundstation/main.cpp:70-71,481-494`) | none | the operator's radio disarm is gone, silently for 5 s | 4 | 6 | 2 | 48 |
| Bondor crash | none | n/a | none | fine | 2 | 3 | 2 | 12 |
| Kill switch pulled | display-only, does not gate arming (`config.h:141-143`). Reads 0 when ESP-NOW is stale (`espnow_link.cpp:49`, ask H filed) | refuses arm only when the cut is *known* (`srot_fc.py:520-549`) | armed with dead thrusters. Depth and yaw integrators wind up; **re-insertion steps thrust in**. Moves "succeed" | no disarm and no integrator freeze on kill | 7 | 4 | 6 | 168 |
| SD card full | write result ignored, `s_ok` stays true (`sd_log.cpp:40-44`). After `log999` exists, `FILE_WRITE` truncates it (`:24-29`; arduino-esp32 `FILE_WRITE`="w") | none | log silently lost | forensics gone exactly when needed | 3 | 3 | 9 | 81 |
| Pi clock step | n/a | ClockMap drops pairs on a jump of < −0.5 s or > +2 s (`flow_timing.py` `STEP_*`), ~2 s on arrival stamps; smaller steps are absorbed as "skew" | EKF frozen for \|step\| (G). `capture_monotonic` falls back to age 0, which flatters (`stamps.py:59-63`). `time.time()` freshness already found | no NTP/RTC policy in provisioning | 6 | 3 | 7 | 126 |
| (cross-cutting) failsafe cancel read as success | — | — | A | — | 8 | 5 | 7 | 280 |

**Re-arm/re-dive:** B is the only live path; nothing re-arms by itself.

**Fixes, in RPN order** (verify every one on the bench with props off):

1. **A.** Firmware: latch `mv_done_seq` only when the move reaches `PH_DONE` itself. A
   cancel caused by a mode change should ACK `MAV_RESULT_CANCELLED`, as preemption already
   does (`mav_stream.cpp:582-591`), with `result_param2` = cause. Host: on ACCEPTED, also
   require mode AUTO and armed; otherwise return FAILED with the last `Failsafe:` text.
   *Verify:* set `LEAK_EN=1`, short the leak pin mid-`move_forward`, expect CANCELLED.
2. **B.** Firmware: refuse `SROT_MOVE`/`SET_MODE` out of SURFACE while `s_fs_surface` and
   its cause hold. Host: give `move()` the same SURFACE refusal as `_require_srot_vision_mode`.
   *Verify:* the same rig; the second move is refused.
3. **C.** Send a STATUSTEXT on Pico link/reboot edges (`queueStatusText`). Suppress RPM
   when `!fresh`. Freeze integrators, or disarm, on link loss while armed. *Verify:* lift
   the Pico's RX; expect a message within 600 ms and no RPM.
4. **D.** `bringup_check --srot` grades `SAFETY_GATES` (CRIT → exit non-zero), and `arm()`
   refuses on LEAK_EN=0 unless overridden. *Verify:* injection test.
5. **Kill.** When kill is known-true and the vehicle is armed, freeze the integrators or
   disarm (pairs with ask H).
6. **E.** Hardware: defeat the EN auto-reset first; add host reconnect only after that.
7. **SURFACE without baro:** time-bound the open-loop ascent, then disarm.

## 2. Time

**The flow as it runs today:**

```
BNO085 400 Hz ─► sensor task 500 Hz ─► g_state ─► Task_MAVLink 100 Hz: stamp = millis() NOW
  └ ATTITUDE/SCALED_IMU2.time_boot_ms = transmit tick (mav_stream.cpp:767-812)
UART 115200, 1 KB TX FIFO (main.cpp:96) ─► CP2102 USB ─► pyserial
  └ pymavlink _timestamp = time.time() (CLOCK_REALTIME) when the reader drains, 200 Hz poll (auv_manager_node.py:1262,1299)
ClockMap(board_s, host_recv_s): lower-envelope line fit, 20 s window (flow_timing.py; auv_manager_node.py:952,1780-1800)
  └ /mongla/imu, /mongla/imu_rates header.stamp = to_host(board) (wall)
Camera: kernel CLOCK_MONOTONIC buffer stamp → stamp_wall via mono→wall offset (v4l2_mailbox.py:454-534; camera_node.py:485-497)
EKF: header stamps; retrodiction horizon 2 s (localization_node.py:172; retro.py)
Vision uplink: capture → ClockMap.to_board → LANDING_TARGET (not consumed by fw)
```

**Clock domains:** board `millis()`; host `CLOCK_REALTIME`, used for pymavlink and every ROS
stamp; host `CLOCK_MONOTONIC`, used for camera capture and every freshness gate. There is no
TIMESYNC: `grep TIMESYNC` over the firmware `src/` finds nothing, and 0 of 12 requests were
answered (flow_timing.py).

**Error sources, quantified:**

| source | size | status |
|---|---|---|
| stamp = transmit tick, not sample (F) | BNO report ≤ 2.5 ms + sensor task ≤ 2 ms + BNO fusion latency (unknown): **~2-5 ms late, ±1.3 ms jitter** | uncorrected. Fix: send `imu_stamp_ms`, or better the SH2 µs timestamp |
| `millis()` quantisation | sd 0.29 ms | minor |
| transport queueing | measured sd 6.67 ms, p2p 35 ms. Code explains it: NVF burst 400 B = 35 ms at 115200; the aligned 1 s burst is **836 B = 73 ms** into a 1 KB FIFO | removed by ClockMap (resid 0.53 ms, system-harmony §4) |
| minimum delay d_min (40 B frame 3.5 ms + USB 1 ms + reader poll 0-5 ms) | **~4.5-9 ms, constant bias** | a lower envelope *includes* d_min, and a one-way method cannot see it. Needs a round trip |
| reader 5 ms poll (`auv_manager_node.py:1299`) | uniform 0-5 ms on `_timestamp` | stamping in the reader on `recv` is free: use pyserial read time |
| skew | crystal ≤ 40 ppm. The documented −31.76 ms/6 s is **5300 ppm, physically implausible**; likely queue growth in the envelope | ClockMap skew is unconstrained. **Clamp to ±300 ppm** and log |
| exposure/2 | 7.85 ms, varies with auto-exposure | handled (`exposure_offset_s`) |
| host wall step/slew | slew ≤ 500 ppm; steps → G | detected at > 0.5/2 s only |

**Recommendation.** Keep the lower envelope, and add the other direction with MAVLink
TIMESYNC (msg 111, https://mavlink.io/en/messages/common.html#TIMESYNC).

- **Board side** (upstream PR, about 20 lines). Reply `tc1` = board µs taken at *receive*,
  not at reply. This is ArduPilot's rule: `timesync_receive_timestamp_ns()` uses the UART
  driver's `receive_time_constraint_us()` for the packet length
  (https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp,
  `handle_timesync` ≈ L3796, L3776). Drop the reply if the TX FIFO lacks space; ArduPilot
  does the same (`HAVE_PAYLOAD_SPACE`).
- **Host.** Send 2 Hz (28 B each way, 0.5 % of the link). Keep samples with RTT < 10 ms.
  That is PX4's `MAX_RTT_SAMPLE`, and it rejects the 1 KB downlink queue asymmetry. Filter
  offset and skew with PX4's gains (α,β 0.05 → 0.003 over 500 samples). Reset after 10
  consecutive > 100 ms deviations, which is how PX4 handles a clock jump (PX4 commit
  e370d98, https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/timesync/Timesync.hpp
  L65-91, Timesync.cpp L45-151).
- **Cross-check.** The existing downlink envelope and a new uplink envelope bracket the
  offset. The half-difference removes d_min up to the asymmetry of the *minimum* delays
  (~1-2 ms), which one-way cannot do.
- **Clocks.** Take `host_recv` from `CLOCK_MONOTONIC` and convert to wall only at ROS
  publish. That makes G and the ClockMap step heuristics moot.
- **Verify.** Truth test: toggle a board GPIO on a scheduled `time_boot_ms` into a
  Pi GPIO interrupt, then compare the mapped time with the edge. Expect < 1 ms after
  TIMESYNC and ~5-9 ms of bias before it.

## 3. End-to-end latency: photon to thrust (vision verb, forward camera)

| stage | mean (range) | evidence |
|---|---|---|
| photon (mid-exposure) → kernel buffer stamp | **~15 (10-25) ms, UNMEASURED**: exposure/2 7.85 + readout + MJPEG + USB | flow_timing.py; the uvcvideo stamp is at frame arrival |
| kernel stamp → detection | 18.0 (pipeline), **22.1 median live**; downward 46.5 | measured-bars.md:58; motion_vision.py:277-281 |
| DDS → manager | 1.8 | camera-latency.md §4 |
| 50 Hz vision loop polling | 10 (0-20) | motion_rates.py:32 |
| MANUAL_CONTROL 23-42 B at 115200 + USB | 3 (2-4.6) | calc |
| Task_MAVLink 100 Hz RX poll | 5 (0-10) | config.h:332; task_mavlink.cpp:144-153 |
| control loop 500 Hz | 1 (0-2) | config.h:366 |
| DShot task 500 Hz + Pico UART 22 B @ 1 Mbaud | 1.2 | config.h:387; thruster_link_proto.h |
| Pico 2 kHz + DShot300 frame | 0.3 | pico/main.cpp:69 |
| **electronics subtotal** | **~58 ms mean, ~95 worst** | |
| ESC + T200 to thrust, τ | **UNKNOWN**; the sweep makes it dominant (τ 0.1 s → 5× heading error) | pr-o-thruster-step-response.md |

The "18.0 ms photon → detection" in measured-bars/system-harmony is really
*kernel-stamp* → detection (pipeline-hardening.md: "capture → pump store 4.04"). The same doc
set also says "full chain to detections 47.5 ms" (camera-latency.md §4). The label should say
"capture-stamp", and a photon measurement (LED in view, GPIO-timed) is owed.

**Dominant terms.** τ (unmeasured) comes first, then camera plus inference (~37 ms). Of the
terms we own, the **two polling stages (~15 ms mean)** are 5× the serial time. Actions:

- Run the vision tick on detection arrival instead of a timer: −10 ms.
- Have Task_MAVLink block on the UART RX event queue instead of a 10 ms tick: −5 ms.
- Raising the baud to 921600 saves only ~2.7 ms on the uplink. Its real value is on the
  downlink: burst jitter 73 → 9 ms.
- Native USB CDC is **not available**. The ESP32 is a classic DevKit (`platformio.ini:23-25`)
  with no USB peripheral. Only a board respin, or the RP2350 as the USB bridge, gets it.
- Board-side servo (issue #9 / ask E) removes loop + uplink + RX poll (~18 ms). More
  importantly, it lets the board de-rotate the bearing with its 500 Hz gyro history at the
  capture instant. At `MOVE_YAW_RATE` 45°/s, 40 ms of staleness is 1.8°, which is **~25 px**
  at 640 px / 46.7°. That needs the TIMESYNC-grade mapping from §2.

## 4. Link budget, 115200 8N1 = 11 520 B/s

The frame is payload + 12 B (v2, unsigned). Trailing-zero truncation is applied per
https://mavlink.io/en/guide/serialization.html#payload_truncation. Rates are the host
`SROT_MESSAGE_RATES` (`auv_manager_node.py:115-135`) over the firmware defaults
(`mav_stream.cpp:652-661`).

| downlink | B/s |
|---|---|
| ATTITUDE 40 B × 50 Hz | 2000 |
| SCALED_IMU2 36 × 50 | 1800 |
| NAMED_VALUE_FLOAT, 15 names × 2 Hz (burst 400 B) + diag 7 × 0.5 Hz | 893 |
| ESC_STATUS ×2 + ESC_TELEMETRY ×2 at 5 Hz (idle truncated / spinning) | 575 / 1250 |
| VFR_HUD 32 × 10 | 320 |
| SCALED_PRESSURE2, SYS_STATUS, BATTERY×2, POWER, HEARTBEAT | 369 |
| **idle total** | **5957 = 51.7 %**. This matches upstream PR #17's measured 51.8 %, an independent check |
| + moving (ACK + MV_* ~353), armed (DEPTH_ERR/OUT 116), ESC spinning | **~7100 = 61.6 %** |

The uplink is HEARTBEAT 42 B/s plus MANUAL_CONTROL at 50 Hz, 1150-2100 B/s (10-18 %).
`LANDING_TARGET` at 25 Hz would add 1050-1800 B/s. **The firmware drops it**
(`srot_fc.py:2380-2386`), so it is pure waste on the wire whenever `vision_uplink_camera` is set.

measured-bars §4 "serial load 18.8 %" is either uplink-only or taken at default rates. As
written it contradicts the above. Label the direction.

**Headroom.** Downlink moving: 61.6 % at 115200, **7.7 %** at 921600 (the CP2102 supports
it; unverified on this unit), and < 1 % on USB FS CDC (not available, see §3). Note that
`tx()` **silently drops** a frame that does not fit the 1 KB FIFO (`mavlink_bridge.cpp:76-85`),
with no counter, and the aligned 1 s burst is 836 B.

**Wasteful messages:** `MIX_VERT`/`MIX_VSGN` (a pure function of parameters, sent at 2 Hz
forever); the deprecated `LEAK` NVF; rarely changing NVFs (`STUNT_PRG`, `ATUNE`, `GAIN`,
`COMP_SEEN`, `YAW_REF`, `BARO_HEALTH`, `MAGACC`), where on-change plus 0.2 Hz saves ~700 B/s;
ESC_STATUS duplicating ESC_TELEMETRY, with V/I/T hard-zero; SCALED_IMU2 carrying mag at 50 Hz
and repeating ATTITUDE's gyro; POWER_STATUS `Vcc=5000`, a **constant presented as a
measurement** (`mav_stream.cpp:330`). Also stagger the stream phases so NVF does not land on
the 1 s boundary.

## 5. Contract drift

**The guard does not run here.** `test_srot_protocol_drift.py` looks only for
`…/Mongla_others/srot-control-board` (`:27-60`). In this layout **all 28 tests SKIP**
(measured: `28 skipped`). With `SROT_FW_DIR=/home/user/srot-control-board`: 27 passed,
1 xfailed. A guard that skips in CI and on the vehicle is not a guard. Make the skip a
failure when `CI` or `MONGLA_REQUIRE_FW=1` is set, and add the `/home/user/<repo>` sibling
layout to the search.

**Live mismatch.** `CMD_SROT_FLARE_ORDER = 31020` (`srot_protocol.py:114`) and the NVF names
`FLARE_ORD`/`FLARE_NON`/`FLARE_AGE` (`srot_fc.py:409-420`) have **no firmware counterpart**:
`grep FLARE` over `src/ include/ shared/` is empty. The command gets the `default:`
UNSUPPORTED (`mav_commands.cpp:647-648`), and `/mongla/flare_order` stays silent forever,
which a mission reads as "no order". It is not in the drift test, and I found no filed
upstream ask.

**What the test does not cover** (checked by hand; no live mismatch unless stated):

- NVF names: the test checks 7 (`:466-474`). The host also reads `YAW_REF`, `COMP_SEEN`,
  `DEPTH_CMD`, `STUNT_PRG`, `BARO_HEALT` (10-char truncation of `BARO_HEALTH`, correctly
  mirrored), `HEAP`, `STK_*`, `MV_*`, `MIX_*`, `AT_*`, `CURR` and `FLARE_*`. All but
  `FLARE_*` are emitted today. Fix: derive the list from every `_named_value(`/`named.get(`
  in `srot_fc.py` and assert against the `sendNamed(` set.
- STATUSTEXT strings parsed: `Thrusters wired…`, `Thruster N LOST telemetry`
  (`srot_fc.py:1030-1039`), `arm first` (`:970`), `otor detect`, `Disarmed:` (`:1481`).
  They match today and none is tested. The host parses **none** of the `Failsafe:` strings,
  so the cause of a board-initiated SURFACE is lost (it feeds A).
- CRC extras / raw layouts live in `srot_fc.py`, not `srot_protocol.py`:
  `_ESC_STATUS_CRC_EXTRA = 10` (`:85`, matches the firmware's `mavlink_msg_esc_status.h`)
  and the SYS_STATUS extended offsets. That breaks the "single copy" rule, and nothing
  compares them.
- Two copies of the leak bit: `SYS_STATUS_SENSOR_LEAK = 2` (`srot_protocol.py:531`) and
  `MAV_SYS_STATUS_SENSOR_LEAK = 0x02` (`:845`). `SYS_STATUS_HAS_EXTENDED_HEALTH = False`
  (`:844`) is stale now that `_sys_status_ext_bits` decodes the bytes by hand.
- Parameter names: the 16 literal names the host reads/sets all exist in `params.cpp` except
  4 ArduSub-only names (pixhawk path). `SERVO{}_ROLE`/`_FUNCTION` are generated at runtime
  (`params.cpp:334+`), so a grep cannot check them, and the 0- vs 1-based index is
  **unverified**.
- Untested timing constants the host relies on: `MANUAL_FRESH/DECAY_MS`, `RATE_MIN_MS`,
  `FS_SURFACE_*`, `DEPTH_STALE_MS`.

## Unverified

No hardware run; O scores are judgement. Unmeasured: photon→stamp latency, thruster τ,
Bluejay re-arm, CP2102/921600, SERVO index base, the 5300 ppm skew. Link figures are
code-derived (the idle total agrees with PR #17).
