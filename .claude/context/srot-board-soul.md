# The SROT board, read end to end — and what it means for `duburi_ws`

Written 2026-09-03 against `srot-control-board` @ `f1d3ba9`, **Hengla v0.2.0,
behaviour rev 14**, with every claim below either read in their source or
measured on the live board. Where the two disagreed, the board won.

This is not a summary of their docs. It is the **half of their design that
changes ours** — the places where a number on our side is only correct because
of a number on theirs.

---

## 1. The signal chain, end to end

What actually happens to a command from `motion_vision` to thrust. Every stage
was verified in source; the ones we did not model are marked.

```
u            our normalised command, -1..1
wire         u * 1000                       MANUAL_CONTROL x/y/z/r (int16)
             CLAMPED to +-1000 (z: 0..1000) at entry, SILENTLY TRUNCATED
sp           (wire/1000) * GAIN             pilotGain(), live, 0.10..1.00
sp          *= sp_auth                      staleness ramp, 1.0 -> 0 between
                                            MANUAL_FRESH_MS 1000 and DECAY 1500
demand       ((1-EXPO)*sp + EXPO*sp^3)      <-- WE DO NOT MODEL THIS
             * PILOT_SPEED                  EXPO 0.30, SPEED 1.0 (measured)
mixer        block-diagonal, per-group saturation scale
oneToDshot   MOT_THST_EXPO 0.65 inverse-quadratic, MOT_SPIN_MIN 0.15 floor
             DShot3D: 1049..2047 fwd, 48..1047 rev, 1048 neutral
```

**The consequence, stated as a number.** The SMALL-SIGNAL gain our loop sees is
`GAIN * (1 - PILOT_EXPO)`. At the boot gain that is `0.5 * 0.7 = 0.35`; at full
gain it is `0.70`. Never 1.0. And the response is mildly cubic, so a `kp` tuned
at large pixel error is too soft near the deadband and vice versa — which is
precisely the terminal-alignment regime `precision-alignment.md` is about.

`applyExpo` is applied **again inside `attitude::stabilize`** to the yaw stick,
so the vision yaw axis is shaped twice over: once as a stick, once as a rate
demand. And a yaw stick below **0.02** is treated as CENTRED, at which point the
board's own heading hold takes over — a 2 % deadband we never accounted for.
Throttle has a **0.05** deadband and moves the depth TARGET at
`MAX_CLIMB_MS * stick * dt` rather than commanding heave directly.

## 2. `GAIN` cannot be set for the session that sets it

`s_gain_live` lazily adopts `JS_GAIN_DEFAULT` **on first use**
(`mav_commands.cpp:218`), and `mav_stream.cpp:850` calls `pilotGain()` in the
telemetry loop, which starts right after `params::init()`. So the value is
latched within the first telemetry tick from whatever NVS held **at boot**. A
later `PARAM_SET` updates NVS and never touches `s_gain_live`.

Measured, and the numbers agree exactly: the board had `JS_GAIN_DEFAULT = 1.0`
stored, **live `GAIN` = 0.500**, and 4.77 h of uptime — it booted before that
value was written. Our `set_default_gain()` at startup is therefore a **no-op
for the run it happens in**, which is not what its name or our docs imply.

**There is an in-session path, and it is proven.** Button functions are
dispatched on the PRESS EDGE of `MANUAL_CONTROL.buttons`, and `JS_GAIN_INC = 42`
steps the live gain by `GAIN_STEP = 0.10`. Map a spare `BTNn_FUNCTION` to 42,
pulse the bit, read `GAIN` back:

```
GAIN before 0.5 -> "Gain 60%" ... "Gain 100%" -> GAIN after 1.0
```

Five presses, no firmware change, board announcing each step. `BTN7_FUNCTION`
defaults to 0 so it is free to borrow; restore it afterwards.

## 3. What the board will and will not answer — measured, not read

`AUTOPILOT_VERSION.capabilities = 8206` = MAVLINK2 | COMMAND_INT | MISSION_INT |
PARAM_FLOAT. That is a claim; these are measurements
(`tools/srot_mav_probe.py`):

| asked | answer |
|---|---|
| `REQUEST_AUTOPILOT_CAPABILITIES` 520 | ACCEPTED + message |
| `GET_MESSAGE_INTERVAL` 510 | ACCEPTED + `MESSAGE_INTERVAL` — **we can verify a rate took** |
| `PARAM_REQUEST_LIST` 21 | **239 parameters** — a whole-config snapshot per run |
| `MISSION_REQUEST_LIST` 43 | `MISSION_COUNT` |
| `REQUEST_PROTOCOL_VERSION` 519 | UNSUPPORTED |
| `DO_SEND_BANNER`, `REQUEST_FLIGHT_INFORMATION` | UNSUPPORTED |
| `LOG_REQUEST_LIST` 117 | **silent** — the SD log cannot be pulled over the link |
| `FILE_TRANSFER_PROTOCOL` 110 | **silent** — no MAVFTP |
| `TIMESYNC` 111 | **silent** — board time and host time cannot be aligned |

**`REQUEST_MESSAGE` (512) ACCEPTS every id and emits seven.** 183 of 190 return
ACCEPTED and send nothing — including ids `-1` and `-2`, which do not exist. The
seven real ones: ATTITUDE, VFR_HUD, COMMAND_ACK, SCALED_IMU2, SCALED_PRESSURE2,
AUTOPILOT_VERSION, NAMED_VALUE_FLOAT. Filed as PR D.

## 4. The leak bit IS readable, and the leak failsafe is OFF

We asked them (TASKS §3) to move LEAK off the multiplexed `NAMED_VALUE_FLOAT`,
because one message per msgid makes observing a flooding hull a lottery. **They
did, in rev 3** — `SYS_STATUS` extended health, `MAV_SYS_STATUS_SENSOR_LEAK = 2`
— and we never adopted it. Our own comment says we cannot, because pymavlink
2.4.49's `SYS_STATUS` has 13 fields and no extensions.

That much is true (verified). What does not follow is that the bits are
unreachable: they are in the payload, and we now record raw frames. Decoded off
the live board, from their own vendored offsets:

```
payload len 40      (MAVLink v2 truncated the trailing zeros; LEN is 43)
idx 30      ff      battery_remaining = -1, so the base ends at 31
idx 31..34  02..    present_extended = 2   LEAK sensor present
idx 35..38  00..    enabled_extended = 0   <-- LEAK_EN is OFF
idx 39      02      health_extended, TRUNCATED TO ONE BYTE
```

Two things follow, and the second is a pre-water gate:

1. A deterministic leak read is available today: take `get_msgbuf()`, slice the
   payload by the DECLARED length, **zero-pad to 43** and unpack three uint32 at
   31/35/39. The padding is not optional — v2 truncation delivered `health` as a
   single byte here, so a naive `unpack_from` reads past the end or throws.
2. **`LEAK_EN = 0.0` on this board**, confirmed by param read as well as by the
   bit. The firmware gates BOTH the pre-arm refusal (`arming.cpp:32`) and the
   leak failsafe on it. As configured, a leak would neither block arming nor
   surface the vehicle.

## 5. Live configuration, read off the board

| param | value | what it means |
|---|---|---|
| `LEAK_EN` | **0.0** | leak failsafe and pre-arm refusal DISABLED |
| `MOT_BAT_V_MAX` | **0.0** | battery feedforward OFF — a timed move travels less on a flat pack |
| `ESPNOW_EN` | 1.0 | the 2nd board link IS up, so the feedforward's only prerequisite is met |
| `THR_TRIM_EN` | 0.0 | correct on a bench (a prop in air would drive the gains to their clamps) |
| `FRAME_REVERSE` | 1.0 | all six axis demands negated |
| `MAG_YAW_REF` | 1.0 | heading is absolute; `YAW_REF` reads 2 = LOCKED |
| `DEPTH_P/I/D` | 0.5 / 0.1 / 0.01 | **measured in water 2026-08-07**, not reasoned |
| `BARO_JIT_MAX` | 25.0 | raised from the 15 default |
| `RPM_LOOP` | 0.0 | correct — an RPM setpoint inside the attitude loop oscillates |
| `ARMING_CHECK` | 1.0 | pre-arm checks active |

`MOT_BAT_V_MAX` is the cheap one: their own doc calls it "usually the better
first move — it needs no water and no learning period", and its prerequisite
(`ESPNOW_EN = 1`) is already satisfied. Until it is set, every timed leg is
pack-state dependent.

## 6. Two corrections to our own record

**`MANUAL` is not a vision mode.** Their contract: *"MANUAL (mode 19) is raw
passthrough with no stabilization at all — no heading hold, no attitude hold. It
is the escape hatch, not a driving mode. Fly STABILIZE."* `MANUAL_CONTROL`
carries no roll/pitch field, so in MANUAL nothing corrects an attitude
disturbance and the bounding box moves for reasons unrelated to position. The
round-27 mode gate accepted MANUAL; it no longer does.

**Reboot-on-open is NOT reproducible today.** Round 26 recorded it as a fixed
property and "the highest-severity item for water": `time_boot_ms` restarting at
545 on three consecutive opens. Re-measured on the Pi through the CH341 adapter,
three consecutive bare pymavlink opens — uptime rose monotonically
(17163787 → 17166787 → 17169787 ms) across 4.77 h of continuous run. **No
reboot.** So it is conditional on something we have not isolated — adapter,
driver DTR/RTS handling, or wiring — not a property of the board. Do not rely on
either behaviour, and **keep `port_guard`**: two openers on one tty is wrong
regardless of whether it also resets the MCU.

## 7. Capability we own and do not use

Implemented in firmware, no driver method on our side:

| command | id | why it matters |
|---|---|---|
| `DO_MOTOR_TEST` | 209 | **GATE 0's procedure.** Per-motor, ARMED, keep-alive at >=2 Hz, window `p4` clamped 600..3000 ms, auto-expires. Runnable from `duburi_ws` today |
| `DO_START_MAG_CAL` | 42424 | the mag calibration behind `YAW_REF` |
| `ACCELCAL_VEHICLE_POS` | 42429 | 6-point accel cal |
| `PREFLIGHT_STORAGE` | 245 | p1=1 save · **p1=2 factory reset, wipes params AND learned motor directions** |
| `DO_SET_RELAY` | 181 | PCA channels 9-16 only; 1-8 need `DO_SET_SERVO` with >=1500 us |
| `USER_1/2/3` | 31010-12 | yaw / pitch / roll spin — RoboSub style points |
| `USER_4` | 31013 | multi-step pattern |
| `USER_5` | 31014 | relay autotune |

Modes we never enter: `ACRO`(1), `MOTOR_DETECT`(20), `AUTOTUNE`(21),
`MOTOR_TUNE`(22), `STUNT`(100), `PATTERN`(101).

`SERVOn_FUNCTION` is an **append-only device identity** stored on the board —
0 NONE, 1 TORPEDO, 2 DROPPER, 3 GRIPPER, 4 LIGHT, 5 CAMERA, 6 AUX — so the
payload map travels with the hull instead of going stale in a host-side table.
It is mirrored by hand in `srot_protocol.py`; inserting a value silently renames
every payload after it.

## 8. Numbers a mission depends on

- `MOVE_CRUISE_MAX 0.80` caps p3, so our 100 % gain is 0.80 of thrust.
- TURN completes at **|err| < 0.03 rad = 1.72 deg**, then locks the heading it
  was ASKED for, not where it stopped — so turns do not accumulate error.
- DIVE completes at ramp within 0.01 m **and** depth within **0.15 m**.
- Brake duration is `MOVE_BRAKE_K * cruise = 0.60 * speed` seconds at
  `MOVE_BRAKE_GAIN 0.55` reverse thrust, along the OUTGOING leg's axis.
- A re-sent `SROT_MOVE` **re-runs** — start is edge-triggered on `mv_seq`, there
  is no idempotency key.
- A new move **preempts** with no brake between them.
- Control / sensor / DShot loops all run at **500 Hz** (`config.h` §6's header
  still says 200 Hz for all three — doc drift, filed).
- `ESC_STATUS` (291) is **not in pymavlink's dialect**: it shows as
  `UNKNOWN_291` at 9.85 Hz and is present in every raw frame. Another reason the
  replay log stores bytes.

---

# The full sweep — all three repos, 2026-09-03

Read end to end: firmware 22.8 k lines, Bondor 6.9 k, ESC flasher 3.4 k. What
follows is only what changes how `duburi_ws` behaves.

## 9. ⛔ A move can report COMPLETE while it is still running

`snapshot()` caches last-good values for the thruster block and explains why:
*"a 3 ms miss is not rare … the previous behaviour transmitted all-zero RPM,
which made a perfectly healthy thruster read 0 in the GCS at random."*

**The `mtx_control` block two blocks above has no cache and no `else`**, so a
missed lock leaves the default-constructed `Snap`: `armed=false`, `mode=0`,
**`mv_active=false`**. And `updateMove` reads

```cpp
bool done = (s.mv_done_seq == s_seq) || (s_seen_active && !s.mv_active);
```

So one missed 5 ms lock on a mutex the 500 Hz loop takes several times per cycle
emits a terminal `ACCEPTED` at 100 %, latched by `s_resolved`. **Rev 13's
failure with a different cause** — except the hull is not dead, it is still
under way. Filed as **PR #8**. Until it lands, do not act on a single terminal
move ACK or a single post-arm heartbeat.

## 10. Absence, by message — the table to code against

| signal | what "no data" looks like |
|---|---|
| `SCALED_PRESSURE2`, `WTEMP`, `DEPTH_ERR`, `DEPTH_OUT`, `BATTERY_STATUS` | **suppressed** — correct, absence is the signal |
| `SCALED_IMU2.temperature` | `0` = MAVLink "not provided" sentinel |
| LoRa `wtemp_c` | `INT8_MIN` sentinel |
| **`VFR_HUD.alt`** | **`-0.0`, ungated** — reads as "at the surface" with no baro |
| **`VFR_HUD.throttle`** | **always 0** — `ControlState.out_*` has no writers |
| **`POWER_STATUS.Vservo`, `CURR`** | **0**, ungated (`BATT_CURR_MULT` is 0.0f) |
| **`ATTITUDE`, `ESC_*`** | **HELD** at the last good value, streamed at full rate |

So: gate depth on `SCALED_PRESSURE2` presence + `BARO_HEALTH`, never on
`VFR_HUD.alt`. Cross-check `ATTITUDE` against the `SYS_STATUS` health bits every
frame, because a dead IMU freezes the attitude rather than dropping it.

## 11. Two more capabilities we own and were not using

**Signed per-thruster RPM.** `ESC_STATUS` (291) is `int32` and signed;
`ESC_TELEMETRY_*` is `uint16` magnitude. pymavlink does not discard 291 — it
returns `MAVLink_unknown` carrying the whole frame (`UNKNOWN_291`), 434/434
CRC-valid on a recorded dive. ⚠ **That return happens BEFORE the CRC check**, so
validate `x25crc` with `crc_extra = 10` yourself. Built: `esc_status_rpm()`.

**Per-thruster presence.** `thrusters.esc_present` reaches the wire ONLY as
English — `"Thrusters wired: 1,2,4 (absent: 3)"` at the first arm, and
`"Thruster N LOST telemetry"` edge-triggered. Sent once each. Built:
`esc_presence()` / `thruster_health()`, parsed from the STATUSTEXT ring.

**Presence UNKNOWN is not presence OK.** An ESC without Bluejay has no
bidirectional DShot at all and reports nothing while its motor spins perfectly.

## 12. The boot burst

~13 STATUSTEXTs back to back (`task_mavlink.cpp:29-141`), of which a poller sees
one. Two change vehicle behaviour silently:

- **`"Params reset to build defaults"`** — a `PARAM_DEFAULTS_VER` bump rewrote
  every row, so `JS_GAIN_DEFAULT` is back to 0.5 (half authority on every
  translation) and `LEAK_EN` back to 0.
- **`"NVS reformatted"`** — additionally loses the sensor calibration.

`boot_warnings()` surfaces these; they belong in `bringup_check --srot` as
FAILs.

## 13. Corrections to what this document said before the sweep

- **GATE 0 is NOT closed by adding `DO_MOTOR_TEST`.** Two different mechanisms:
  `DO_MOTOR_TEST` (209) spins ONE motor at a chosen throttle and needs ARMED
  plus a ≥2 Hz keep-alive; **`MODE_MOTOR_DETECT` (20) is what writes
  `CAL_MDIRn`**, and per rev 6 it needs **water and a free-to-rotate hull** —
  it finishes FAIL below 0.05 rad/s, which is what an in-air run produces.
  Adding both driver methods is right; describing either as closing GATE 0 is
  not.
- **⚠ `MOTOR_DETECT` is not a neutral read on this board.** `MOT_n_DIRECTION`
  and `CAL_MDIRn` **multiply**, this hull has `MOT_1`/`MOT_8 = -1` set
  **runtime-only, deliberately unsaved**, and `FRAME_REVERSE = 1` — which a
  successful detect makes wrong. Running it changes stored calibration that
  then composes with two live overrides. Any driver method must require an
  explicit confirmation naming those three values.
- `DIVE`'s **`p3` is ignored** — the descent rate is always `MOVE_DEPTH_RATE`
  (0.20 m/s). The depth-*rate* loop is deferred roadmap work.
- **Missions are RAM-only** (`mission.cpp` is a plain array, no NVS) despite
  `AUTOPILOT_VERSION` advertising `MISSION_INT`. Re-upload after every reboot.
- **`AUDIT.md` is not the defect ledger** — it stops at rev 2 and cites R46-R56,
  which do not exist in it. `include/config.h`'s revision block is the ledger,
  and it runs to rev 14.
- **Bondor's `FW_BEHAVIOUR_REV_REQUIRED` is still 2** (`protocol.ts`); ours is
  10. Rev 13 is where a disarmed `SROT_MOVE` stopped faking `ACCEPTED` at 100 %.
  Their `FlightMode` enum also omits `STUNT` (100) and `PATTERN` (101), which
  their own LoRa bridge forwards.

## 14. What a Bluejay flash is a precondition for

From the ESC flasher's own docs: stock **BLHeli_S has no bidirectional DShot**,
so an un-flashed ESC reports no telemetry and the board reads "no ESC" while the
motor beeps and spins. That is the precondition for ESC presence detection, the
Pico's RPM loop, and constant-distance timed moves — *the whole `SROT_MOVE`
design*. Motor direction must be **Forward/Reverse (3D mode)**; the Pico emits
DShot 3D framing (`1048` neutral) and this is not optional. Until an ESC reports,
the Pico drives that motor open-loop with no integrator, so it is safe to arm
and test with stock ESCs.
