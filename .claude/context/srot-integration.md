# SROT control-board integration (branch `srot`)

> Migrating `duburi_ws` off Pixhawk/ArduSub onto the custom **SROT** board (firmware
> "Hengla": ESP32 flight core + RP2350 Pico RPM co-processor). A transport-and-verbs
> swap, not a rewrite — the board owns the primitives, the Jetson sends intent.
> Board-side source of truth: `Mongla_others/srot-control-board/{DUBURI_WS_INTEGRATION,
> JETSON_COMMS,ALGORITHMS,AUDIT,PARAMETERS}.md`.

## The offload boundary
**The board owns:** attitude + depth hold, thrust allocation (vectored 6DOF), timed
motion primitives **with on-board braking**, heading hold between/within legs, arming +
pre-arm checks, leak/low-batt/GCS-loss failsafes → SURFACE, ESC RPM telemetry.
**The Jetson owns:** perception, mission logic, *which* verb next, DVL position loops,
vision servoing, payload sequencing, logging.

## Architecture — the FlightController HAL (built, `duburi_control/fc/`)
```
fc/base.py         FlightController ABC + Telemetry + MoveResult DTOs
fc/srot_protocol.py THE wire constants (cmd 31000, type codes, modes, ACKs, MC scaling)
fc/srot_fc.py      SROT backend (MANUAL_CONTROL, SROT_MOVE + ACK relay, telemetry)
fc/pixhawk_fc.py   ArduSub backend (IS-A Pixhawk + intent methods) — unchanged behaviour
fc/factory.py      make_flight_controller('pixhawk'|'srot', master, log)
```
ROS param **`flight_controller`** (`pixhawk`|`srot`), default `pixhawk` until SROT is
pool-proven. Both backends coexist on the same vehicle for A/B. The ABC exposes intent:
`arm/disarm/set_mode/manual(fwd,lat,up,yaw)/move(verb,*,on_progress,abort_fn)/telemetry/
send_gcs_heartbeat`. `manual()` is the streamed servo primitive (vision + DVL loops);
`move()` is one on-board SROT_MOVE + its four-terminal ACK relay.

## The verb table (`fc/srot_fc.py:_build_params` + `MOVE_VERBS`)
**Clean SROT_MOVE (31000) collapses** — `move()` sends one command, relays the ACK:
| duburi verb | p1 type | params |
|---|---|---|
| `move_forward` | 0 forward | p2=duration, p3=speed(gain/100) |
| `move_left`/`move_right` | 2/3 strafe | p2=duration, p3=speed |
| `arc` | 9 | p2=duration, p3=speed, p4=signed yaw rate (`yaw_rate_pct`) |
| `yaw_left`/`yaw_right` | 4 turn (**relative**) | p2=∓degrees, p4=0 |
| `turn` | 4 turn (**absolute**) | p2=heading, p4=1 — **needs `MAG_YAW_REF=1`** |
| `set_depth` | 5 dive | p2=**−target** (duburi neg → SROT pos depth); refuses target>0 |
| `stop` | 6 brake | (also the ROS-cancel wire action) |
| `pause` | 7 hold | station-keep (no channel-release exists on SROT) |
| `style_roll` | 8 style | p2=count (roll only, 90°/s) |

**Not through `move()`:** `arm`/`disarm`/`set_mode` (COMPONENT_ARM_DISARM 400 / DO_SET_MODE
176); `surface` → `set_mode('SURFACE')` (mode 9); `style_yaw` → `USER_1` 31010 (STYLE is
roll-only); `move_forward_dist`/`move_lateral_dist` (DVL loop **stays in Python**, streams
`manual()`); `vision_align`/`vision_move` (perception loop stays, streams `manual()`);
`fire` (PayloadDriver USB, unchanged); `dvl_connect`, `calc_distance` (host-side).
`lock_heading`/`unlock_heading` — board holds heading on-board; an explicit lock to a
*commanded* heading is a `turn` absolute (needs `MAG_YAW_REF`), else no-op.

## Backend gotchas (why the SROT path differs)
- **No `SET_MESSAGE_INTERVAL`** — SROT rates are fixed on-board; the manager must **skip**
  `MESSAGE_RATES` pinning on the srot backend.
- **No RC-channel release / `NO_OVERRIDE`** — `MANUAL_CONTROL` always carries all 4 axes.
  `release_yaw` (vision) → send `r=0` and rely on the board's centred-stick heading hold.
- **No depth-setpoint stream** — `SET_POSITION_TARGET_*` is UNSUPPORTED. Depth = a mode
  (`DEPTH_HOLD` latches current) or a DIVE move. The `motion_vision` downward fill→depth
  descent must be re-expressed as periodic small dives (Phase 8).
- **Heartbeat is two things:** the 5 Hz *neutral-RC* `heartbeat.py` is **harmful** on SROT
  (neutral MANUAL_CONTROL fights an AUTO move) → disable it. The ≥1 Hz **MAVLink** HEARTBEAT
  (`SrotFC.send_gcs_heartbeat`) is **mandatory** — 5 s silence surfaces the vehicle.
- **`GAIN` boots at 0.5** → MANUAL_CONTROL is halved. `SrotFC.set_default_gain(1.0)` at
  bring-up; verify against the `GAIN` NAMED_VALUE_FLOAT.
- **Mode strings change** (ArduSub names → SROT 0/1/2/9/19/23). Audit missions/FSM for
  hardcoded `'ALT_HOLD'` checks — `set_depth` engages ALT_HOLD on Pixhawk, a DIVE-in-AUTO
  on SROT.
- **Transport (current, Bondor-era):** the SROT board plugs **straight into the dev-box /
  Jetson over USB Type-C @115200** — **no BlueOS / no UDP router**. `connection_config.
  resolve_srot_profile()` autodetects the USB-serial port (CP210x / CH340 / CH9102 / ESP32
  by-id, `ttyUSB*/ttyACM*` fallback); `-p mav_device:=/dev/serial/by-id/<yours>` overrides (a
  device path or any pymavlink conn string, so a future `udpout:192.168.2.2:14550` still
  works). Vehicle sysid/compid 1/1, source 255/190. **`flight_controller` defaults to `srot`
  on this branch** (`:=pixhawk` for the ArduSub/BlueOS path).

## ⚠ Bench bring-up runbook (board benchable now; each step gates the next)
Prereqs (operator, via **Bondor** — no duburi_ws code): ESCs on **Bluejay** (`DSHOT_BIDIR=1`,
`RPM_LOOP=1`) or ESC_STATUS RPM is absent; thruster **directions** verified (motor-detect);
**Bar30 fitted**; a **`.params` export** taken and committed to `config/srot/` (a
`PARAM_DEFAULTS_VER` bump wipes `CAL_*` — otherwise unrecoverable).

1. **Telemetry (props off):** `flight_controller:=srot`; confirm `/duburi/state` populates
   (yaw/depth/armed/mode) + `/duburi/esc_rpm`. Nothing actuates.
2. **arm/disarm (props off):** from the CLI. Confirm pre-arm STATUSTEXT surfaces on reject.
3. **`manual()` per axis:** teleop each of fwd/lat/up/yaw **individually**, verify direction
   vs `docs/THRUSTER_MAP.md`. A wrong sign is positive feedback — fix `MOT_n_DIRECTION` in
   Bondor, never in code.
4. **⚠ DEPTH LOOP — MANDATORY, the depth loop has NEVER run closed (AUDIT R1):**
   - Enter `DEPTH_HOLD`, raise & lower the sub by hand: verticals must push **back toward**
     the latched depth, not away.
   - Force SURFACE (trip the leak input) at depth: the demand must be **ascend**.
   Do **not** trust any DIVE / `set_depth` / vision-depth behaviour until both pass.
5. **`move()` collapse verbs:** each runs + reports ~3 Hz progress; ROS-cancel brakes (type 6);
   a preempting second move resolves the first as PREEMPTED (not a hang).
6. **Vision:** `motion_vision` port (Phase 8) — lat/yaw/fwd via `manual()`, depth via mode.

## What works on the SROT backend today (verb support matrix)
**WORKS (wired + unit-tested; bench-verify on the board):** `arm` / `disarm` / `set_mode`;
the collapse moves `move_forward` / `move_left` / `move_right` / `yaw_left` / `yaw_right` /
`turn` / `set_depth`(dive) / `stop` / `pause`(hold) / `style_roll`; `fire` (PayloadDriver);
telemetry → `/duburi/state` (yaw/depth/batt/mode/armed) + the GCS heartbeat.
**NOT YET on SROT (fall through to the facade → fail cleanly as `success=False`, they do
NOT crash the node):** `vision_align` / `vision_move`, `move_forward_dist` /
`move_lateral_dist` (DVL), `lock_heading` / `unlock_heading`, `calc_distance`, `arc`. These
need the `manual()`-streamed facade port (vision/DVL) or a host-side mapping (arc heading→rate,
heading-lock) — the next phase. Do not treat "wired + all green" as "every verb works on srot."

## Status (branch `srot`)
- **DONE:** the HAL foundation (`14eb27a`) + integration doc (`6f3aea5`) + **the manager
  wiring** (`7f3d6e8` + review fixes): `flight_controller:=srot` is the **default**, connects
  over **direct USB serial**, banner shows `SROT board · firmware Hengla · USB serial`, the
  collapse verbs route through `fc.move()` + the 4-terminal ACK relay, telemetry populates
  `/duburi/state`. Build clean; ~670 tests green. mavlink-reviewer + advisor signed off (arc
  dropped, `manual()` NaN-safe, verified fail-closed arm-abort, HEARTBEAT source-filtered).
- **NEXT:** the `motion_vision` port (lat/yaw/fwd→`manual()`, depth→mode) + the DVL-distance
  streamed path + `lock_heading` semantics; publish `/duburi/esc_rpm`; commit the Bondor
  `.params` export.
- **BENCH-GATED:** the runbook above (needs the board; first real validation — no SROT SITL).
  Plug in USB, `ros2 run duburi_manager start`, watch the banner + `/duburi/state`. Depth stays
  unproven until the hand-verification (step 4) passes.
