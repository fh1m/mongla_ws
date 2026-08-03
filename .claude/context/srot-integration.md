# SROT control-board integration (branch `srot`)

> **Firmware baseline for this branch: `srot-control-board` @ `22afc95`, `SROT_FW_BEHAVIOUR_REV 4`.**
> Read [`auv-architecture-2026.md`](auv-architecture-2026.md) first if you have not.
>
> **⚠ YAW IS ABSOLUTE FROM REV 4.** `ATTITUDE.yaw` and `VFR_HUD.heading` are a magnetic compass
> heading, not a value relative to wherever the board booted. A heading recorded against rev <= 3,
> or compared across a vehicle reset, is **not the same number**. Absolute `MOVE_TURN` (p4=1) now
> actually turns to the heading it is given — it needs `MAG_YAW_REF=1`, which is the new default.
>
> **⚠ DEPTH AND TEMPERATURE CAN NOW BE ABSENT.** `NAMED_VALUE_FLOAT("WTEMP")` and
> `SCALED_PRESSURE2` are **suppressed** when the barometer is unhealthy or stale, and
> `SCALED_IMU2.temperature` sends MAVLink's `0` "not provided" sentinel. Treat absence as "no
> data", never as zero. This is deliberate: before rev 3 a Bar30 whose calibration PROM was read
> during a reset race published fabricated pressure, depth AND temperature — `-51 C` and `+2.87 m`
> in air were both observed — and nothing marked them as wrong. A board whose PROM fails CRC now
> also **refuses `DEPTH_HOLD` / `AUTO` / `PATTERN`** rather than flying on invented depth.

> Migrating `duburi_ws` off Pixhawk/ArduSub onto the custom **SROT** board (firmware
> "Hengla": ESP32 flight core + RP2350 Pico RPM co-processor). A transport-and-verbs
> swap, not a rewrite — the board owns the primitives, the Jetson sends intent.
> Board-side source of truth: `Mongla_others/srot-control-board/{DUBURI_WS_INTEGRATION,
> JETSON_COMMS,ALGORITHMS,AUDIT,PARAMETERS}.md`.

## Transitional rig — SROT through BlueOS over UDP (verified on hardware 2026-08-03)

The SROT board has no Ethernet, so while the hull is still wired Pi-first the board hangs
off the **Raspberry Pi's USB** and reaches us as **UDP**, instead of the designed direct
USB-C-to-Jetson cable. This works and is fully verified — but read the link-quality
caveat at the end before planning a mission around it.

**Measured topology** (`.1`/`.2` were swapped in CLAUDE.md until 2026-08-03):

| Host | Address | Note |
|---|---|---|
| BlueOS Pi | **192.168.2.2** | Pi MAC OUI, BlueOS **1.4.2**, also the gateway |
| Topside / dev box | **192.168.2.1** | Blue Robotics' standard topside address |
| SROT board | `/dev/ttyUSB0` **on the Pi** | CH340 `1a86:7523` |

### Why Bridget, and not the Autopilot Manager

BlueOS's ArduPilot Manager will **never** adopt this board — it lists only known ArduPilot
boards (`/v1.0/available_boards` returns SITL alone) and the SROT reports
`MAV_AUTOPILOT_GENERIC` on a CH340. **Bridget** (port 27353) is the right service: a *raw
byte* serial↔UDP bridge with no MAVLink awareness. That matters twice — it needs no board
recognition, and it **injects no heartbeats of its own**, which a router like
mavlink-router would. The firmware feeds its GCS failsafe off *any* foreign heartbeat, so
an injecting router would hold that failsafe open on our behalf.

### Setup — one call, idempotent, safe to re-run

```bash
# create (HTTP 201). udp_listen_port is ALSO the source port Bridget sends from.
curl -s -X POST http://192.168.2.2:27353/v1.0/bridges -H 'Content-Type: application/json' \
  -d '{"serial_path":"/dev/ttyUSB0","baud":115200,"ip":"192.168.2.1",
       "udp_target_port":14550,"udp_listen_port":14551}'

curl -s http://192.168.2.2:27353/v1.0/bridges          # verify
curl -s http://192.168.2.2:27353/v1.0/serial_ports     # is the board even on the Pi?
# DELETE takes the SAME body; delete->recreate->reconnect verified working.
```

`ip` is where Bridget **sends**; `udp_listen_port` is what it **binds and sends from**.
Because it transmits from its listen port, a pymavlink `udpin:` — which replies to the
source of the last datagram — lands back exactly where Bridget listens, so the link is
**bidirectional with no extra config**. That is the one property to re-verify if these
numbers are ever changed: telemetry flowing proves nothing about the uplink.

### Host side — no code change needed

`resolve_srot_profile()` already takes any pymavlink connection string and only attaches
a baud when the target starts with `/dev/`, and `SrotFC` is transport-agnostic.

```bash
ros2 run duburi_manager bringup_check --srot --srot-device=udpin:0.0.0.0:14550
ros2 run duburi_manager connect --path udpin:0.0.0.0:14550 --watch
ros2 run duburi_manager start --ros-args -p mav_device:=udpin:0.0.0.0:14550 \
                                         -p yaw_source:=mavlink_ahrs
```

Pass the **same endpoint to all three** — `mode` still does not apply on srot.

### ⚠ Link quality: measurably worse than direct serial — bench use, not water

| | direct USB-C serial | via BlueOS/Bridget |
|---|---|---|
| `BAD_DATA` | **zero in 15 s** (bench 2026-08-02) | **~10–23 %** of frames |
| command round-trip | — | **11/12** (`AUTOPILOT_VERSION`) |

**Mechanism, measured:** Bridget chunks the serial stream at arbitrary byte offsets —
only **77 % of datagrams begin on a message boundary** (`0xFD`), so ~23 % start
mid-message, which matches the loss almost exactly. Reassembling the datagrams into one
continuous stream and re-parsing recovers most of it (20 % → 7.8 %, and most of that
residue is the known-undecodable `ESC_STATUS(291)`), which says the **bytes are largely
intact and it is a framing problem, not a wire problem**. It is *not* saturation: dropping
the telemetry rates did not reduce it (the link runs at ~73 % of 115200 either way).

**So: use this rig for bring-up, telemetry and bench verification. For an autonomous
in-water run, put the board back on the Jetson's USB-C cable** — the designed
architecture, and the one with zero observed frame loss. MAVLink has no retransmission,
so a lost `COMMAND_LONG` is a lost arm or a lost move with nothing in any log.

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
ROS param **`flight_controller`** (`pixhawk`|`srot`). **On this branch the launch default is
`srot`** — this paragraph used to say `pixhawk` while the transport section below said `srot`;
the launch files are the tiebreaker and they say `srot`. Both backends coexist on the same
vehicle for A/B (`:=pixhawk` for the ArduSub/BlueOS path). The ABC exposes intent:
`arm/disarm/set_mode/manual(fwd,lat,up,yaw)/move(verb,*,on_progress,abort_fn)/telemetry/
send_gcs_heartbeat`. `manual()` is the streamed servo primitive (vision + DVL loops);
`move()` is one on-board SROT_MOVE + its four-terminal ACK relay.

## The verb table (`fc/srot_fc.py:_build_params` + `MOVE_VERBS`)
**Clean SROT_MOVE (31000) collapses** — `move()` sends one command, relays the ACK:
| duburi verb | p1 type | params |
|---|---|---|
| `move_forward` | 0 forward | p2=duration, p3=speed(gain/100) |
| `move_left`/`move_right` | 2/3 strafe | p2=duration, p3=speed |
| `yaw_left`/`yaw_right` | 4 turn (**relative**) | p2=∓degrees, p4=0 |
| `turn` | 4 turn (**absolute**) | p2=heading, p4=1 — **needs `MAG_YAW_REF=1`** |
| `set_depth` | 5 dive | p2=**−target** (duburi neg → SROT pos depth); refuses target>0 |
| `stop` | 6 brake | (also the ROS-cancel wire action) |
| `pause` | 7 hold | station-keep (no channel-release exists on SROT) |
| `style_roll` | 8 style | p2=count (roll only, 90°/s) |

> **`arc` is NOT mapped** and is excluded from `MOVE_VERBS`. duburi's `arc` holds an
> ABSOLUTE target heading while curving; SROT's `MOVE_ARC` p4 is a signed yaw **rate** with
> no heading lock. Passing the heading as a rate would spin the hull, so `arc` is refused
> (see `UNSUPPORTED_VERBS`) until a host-side heading→rate arc lands.

**Not through `move()` — implemented:** `arm`/`disarm`/`set_mode` (COMPONENT_ARM_DISARM 400 /
DO_SET_MODE 176); **`surface` → `stop_motion()` + `set_mode('SURFACE')`** (mode 9), handled
in `auv_manager_node._run_srot_surface` because the facade's `surface` is `set_depth(0)` and
would die on the ArduSub-only ALT_HOLD gate; `fire` → **`SrotPayload`** (the board's PCA9685
over `DO_SET_SERVO`/`DO_SET_RELAY` — there is NO separate USB ESP32 any more), which
**refuses until `payload_fire_map` is configured** rather than guessing the wiring;
`dvl_connect`, `calc_distance`, `head`, `mission_reset`, `calibrate_depth` (host-side, work
unchanged).

**`move_back` is NO LONGER REFUSED.** `MOVE_BACK = 1` was always valid on the wire and your own
brake path was already commanding it; it was refused purely for a missing two-line
`_build_params` branch. That branch now exists, so `move_back` is a normal collapse verb
(p1=1, p2=duration, p3=speed) — the same shape as `move_forward`.

**REFUSED on srot (`srot_fc.UNSUPPORTED_VERBS`, checked in `execute_callback` BEFORE
dispatch → clean `success=False`):** `lock_heading`, `move_forward_dist` /
`move_back_dist` / `move_lateral_dist`, `vision_align` / `vision_move`, `arc`, `style_yaw`.
Each reaches a Pixhawk-only primitive (`send_rc_*` / `set_target_depth`) or the ALT_HOLD
gate. **They are refused rather than left to fall through because falling through was worse
than failing:** `lock_heading` returned `success=True` while holding nothing (and flooded 50
swallowed `AttributeError`/s for the lock's 300 s life), and `move_back` raised an
`AttributeError` even though `MOVE_BACK=1` exists on the wire. Removing a verb from that set
is how the port lands. `unlock_heading` still works (it only stops a lock + sends neutral).

## Backend gotchas (why the SROT path differs)
- ~~**No `SET_MESSAGE_INTERVAL`**~~ — **NO LONGER TRUE as of fw `SROT_FW_BEHAVIOUR_REV 2`.**
  `MAV_CMD_SET_MESSAGE_INTERVAL` (511) **and** `GET_MESSAGE_INTERVAL` (510) are implemented and
  bench-measured on COM19 (2026-08-01, USB serial, props off — an observed rate over a short
  window, not a guaranteed spec): ATTITUDE pinned from 10.8 Hz to **55.2 Hz**, VFR_HUD
  5.4 → 11.0 Hz. Treat 50 Hz as the number to design against. Rate
  pinning is now **enabled** on srot (`SROT_MESSAGE_RATES` in the manager). This matters
  concretely: `_imu_rates_tick` publishes at 50 Hz and was oversampling a 10 Hz stream 5×,
  feeding the optical-flow rotation compensation stale attitude between real samples.
  Two refusals remain, both deliberate: HEARTBEAT cannot be disabled (`interval < 0` on
  HEARTBEAT is answered **DENIED**, not silently ignored), and an unknown msgid is DENIED
  rather than accepted-and-dropped.
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

   > ⚠ **This gates EVERY AUTO move, not just the dive-dependent verbs.** `SROT_MOVE`
   > auto-enters `AUTO`, and the `AUTO` branch calls `depth::setTarget(md.depth_target)` +
   > `depth::update(...)` underneath **every** primitive (fw `task_control_loop.cpp:236-237`)
   > — there is **no depth-free path through AUTO**. So a plain `move_forward` with no
   > `set_depth` anywhere still runs the unverified loop, and a vertical runaway mid-leg is
   > indistinguishable from a buoyancy problem in the water. Earlier revisions of this line
   > scoped the gate to `DIVE` / `set_depth` / vision-depth, which reads as "skip these if
   > you are only driving forward". That is backwards.
   >
   > A successful **in-air** `move_forward` is not partial validation: at ~0 m the latched
   > target and the measurement agree, so the loop is never actually exercised.
5. **`move()` collapse verbs:** each runs + reports ~3 Hz progress; ROS-cancel brakes (type 6);
   a preempting second move resolves the first as PREEMPTED (not a hang).
6. **Vision:** `motion_vision` port (Phase 8) — lat/yaw/fwd via `manual()`, depth via mode.

## What works on the SROT backend today (verb support matrix)
**WORKS (wired + unit-tested; bench-verify on the board):** `arm` / `disarm` / `set_mode`;
the collapse moves `move_forward` / `move_left` / `move_right` / `yaw_left` / `yaw_right` /
`turn` / `set_depth`(dive) / `stop` / `pause`(hold) / `style_roll`; **`surface`** (SURFACE
mode); `unlock_heading`; `head`, `mission_reset`, `calibrate_depth`, `calc_distance`,
`dvl_connect` (host-side); `fire` **once `payload_fire_map` is set**; telemetry →
`/duburi/state` (yaw/**depth**/batt/mode/armed) + the GCS heartbeat.

**REFUSED with a clear message (`UNSUPPORTED_VERBS`, see the verb table above):**
`vision_align` / `vision_move`, `move_*_dist` (DVL), `lock_heading`, `arc`,
`style_yaw`. Do not treat "wired + all green" as "every verb works on srot."

> **`move_back` moved out of this list** (see above). `arc` and `style_yaw` remain genuine
> firmware gaps, not host gaps: `arc` needs an absolute-heading arc on the board, `style_yaw`
> needs a selectable STYLE axis + rate. Both are buildable on request — they are simply not
> built, and refusing is the honest state.

### Firmware behaviour rev 3 (2026-08-02) — absence became a signal

Rev 3 landed with the board **in the vehicle**, and its theme is that the firmware now
**refuses to report data it cannot stand behind**. Three consequences reach us:

1. **The Bar30's calibration PROM is validated (CRC-4) and no longer read in a race.**
   The vendored MS5837 driver's `reset()` compared `micros()` against a `millis()` baseline,
   so the mandatory ~2.8 ms post-RESET reload delay was **zero** and `initConstants()` read
   the PROM while the sensor was still reloading it. The coefficients were boot-time luck.
   That is not a display bug: `dT` feeds `offset` and `sens`, so a bad PROM corrupted
   **pressure and depth** silently, by an amount whose sign you cannot predict. On a vehicle
   whose depth loop has never run closed, that is what the first dive would have flown on.
2. **An unhealthy or stale baro now refuses `DEPTH_HOLD`/`AUTO`/`PATTERN`.** `SROT_MOVE`
   enters `AUTO`, so this means **every move verb is denied** — `move_forward` included.
   `bringup_check --srot` reads it via `_baro_health_verdict` off `SYS_STATUS`, because on
   the deck the symptom is "it arms and then does nothing".
3. **`WTEMP` and `SCALED_PRESSURE2` are suppressed when the baro is unhealthy**, and
   `SCALED_IMU2.temperature` sends MAVLink's `0` "not provided". Absence is the signal.
   ⚠ **`VFR_HUD` is NOT gated on `depth_ok`** — and `VFR_HUD.alt` is where we read depth. So
   our depth number keeps arriving on a board that has declared its baro dead. That is safe
   only because the same `depth_ok` flag *also* refuses AUTO (`task_control_loop.cpp:107`
   and `:348` use the identical condition), so no closed-loop move ever runs on it. Read
   health from `SYS_STATUS`, never infer it from the presence of a depth value.

**`FW_BEHAVIOUR_REV_REQUIRED` stays at 2, deliberately.** Rev 3's changes are additive for
this host, so a rev-2 board still runs it correctly; raising the floor would strand a
working vehicle for no safety gain.

**LEAK moved to `SYS_STATUS` extended health — and we cannot read it.** pymavlink 2.4.49's
`SYS_STATUS` has thirteen fields and no extensions, so the board's 40 bytes are parsed
against a 31-byte schema and the rest discarded; `onboard_control_sensors_health_extended`
is always `None`. Exactly the `ESC_STATUS(291)` trap in a new hat. The firmware therefore
keeps `NAMED_VALUE_FLOAT("LEAK")` as a deprecated duplicate and **the real fix is host-side**
(next section). `srot_protocol.SYS_STATUS_HAS_EXTENDED_HEALTH` records why, and a test fails
loudly when pymavlink catches up.

### Reading multiplexed `NAMED_VALUE_FLOAT` — why the reader hook is mandatory

The board rides `LEAK`, `WTEMP`, `STUNT_PRG`, `ATUNE`, `KILL`, `CURR` and `GAIN` on one
msgid, **all seven back-to-back inside a single 500 ms tick** (`mav_stream.cpp`, `iv_nvf`).
pymavlink keeps exactly one message per msgid. So by the time anything samples
`master.messages['NAMED_VALUE_FLOAT']`, the whole burst has already drained through the slot
and only the **last** name — `GAIN` — is left, and it stays there for the ~475 ms until the
next burst.

That means sampling the slot does not lose `LEAK` *occasionally*. **It loses it always.**
The uniform-lottery framing is wrong: the burst has a fixed order, and `LEAK` is first.

`SrotFC` therefore keeps its own per-name table with a freshness stamp, fed from
`auv_manager_node.reader_loop` — the only code that sees the names in between — via
`note_named_value()`. `_drain_named()` folds each message object **once**; that identity
check is load-bearing, because pymavlink never clears its slot and re-folding would re-stamp
a dead value as fresh forever, defeating the `max_age_s` the table exists to enforce.

### ⛔ Bench findings, 2026-08-02 — measured on the vehicle, not inferred

The board was on the dev box inside the AUV, disarmed, thrusters off. Read-only.
**Two faults block the water test and one host bug was fixed as a result.**

**1. The Bar30 is producing NOISE, and the board reports it HEALTHY.**

```
30 samples over 6 s, still on a bench:   press_abs 317 .. 874 mbar   (sea level ~1013)
                                         WTEMP       6 .. 30 C
                                         VFR_HUD.alt +0.9 .. +6.8 m  (in air)
SYS_STATUS ABSOLUTE_PRESSURE health bit: SET  ("healthy")
```

This is **not** an offset and **not** a drift — it is per-sample garbage, the signature of a
bad I2C read / loose connector, and it matches the firmware team's own last-session note that
"the Bar30 stopped responding... I believe this is the sensor connector, not firmware."

The important part is **why rev 3's protection does not catch it.** The firmware validates
each sample against a deliberately wide plausibility band (~`[300, 40000]` mbar, chosen loose
so a judgement call cannot ground the vehicle). Every one of those readings is individually
inside the band, so `SCALED_PRESSURE2` keeps streaming, `WTEMP` keeps streaming, and the health
bit stays set. **A per-sample band is structurally blind to variance.** `bringup_check`'s
`_baro_noise_verdict` checks peak-to-peak over a window, which is the thing a board-side
pre-arm check (one sample) cannot do.

**2. That phantom depth SATURATES the depth controller — and it is the arming blocker.**

```
disarmed, stationary:   DEPTH_ERR -3.0 .. -6.7 m     DEPTH_OUT -1.00 (full scale)
                        MIX_VERT  -1.00              MIX_VSGN  4
```

The firmware team reported "on arming, props off, nothing commanded, the four VERTICAL
thrusters spun to ~3000 RPM while the horizontals idled correctly" and could not explain it.
This is the explanation, and it is fully determined by their own source:

`mixer.cpp`'s matrix is **block-diagonal** — motors 5-8 (vertical) are non-zero only in
**roll, pitch, throttle**; motors 1-4 (horizontal) only in yaw/forward/lateral. The throttle
column is `-1` for all four verticals. So a heave demand of `-1.0` becomes `+1.0` on every
vertical and `0` on every horizontal the instant the outputs go live. **Verticals at full,
horizontals idle** — the reported symptom exactly, with no residual mystery.

> A hypothesis worth recording as **refuted**: this was first attributed to the wiped `CAL_*`
> level calibration feeding a phantom tilt into the attitude loop (which also drives only the
> verticals). The live board says otherwise — roll `-0.81°`, pitch `+1.77°`, both small. The
> attitude loop is fine; the depth loop is not. Reading the board settled in one probe what
> source-reading had got wrong.

**3. Host bug found by the same probe: `get_battery()` was a coin flip.**

The board streams `BATTERY_STATUS` **twice** — id 0 (PM1 electronics) and id 1 (PM2 thruster
pack) — at 2 Hz each, and pymavlink caches one message per **msgid**, not per instance. A live
sample showed the slot alternating between **1.35 V and 14.74 V**. `/duburi/state`'s battery
voltage was therefore whichever arrived last. Same failure as `NAMED_VALUE_FLOAT`, one layer
down; fixed the same way (`SrotFC.note_battery`, fed from the manager's reader thread).
`get_battery()` is now pinned to id 0 and `get_batteries()` returns both.

### Payload: the board owns the channel role, we only read it

Each PCA9685 channel's role is a **firmware parameter** — `SERVO{n}_ROLE`, n = PCA channel + 1
(`0` disabled, `1` PWM servo, `2` MOSFET/switch). **Measured on the vehicle 2026-08-02:
1-8 = SERVO, 9-16 = SWITCH.**

`duburi_ws` drives **switch channels only**. The PWM channels are the on-board manipulator
arm, and firing one from a mission would move the arm mid-drop. `SrotPayload.fire()` therefore
reads the role from the board and refuses anything that is not `2`, **failing closed on an
unreadable role**.

`payload_fire_map` is now plain numbers — `"1:9, 2:10"` (duburi channel : 1-based PCA channel).
The old `1:relay:0` / `2:servo:3` forms still parse, but encoding the role host-side duplicates
board state and goes stale **silently** on a re-role, which is exactly the failure that would
drive the arm.

`DO_SET_SERVO` addresses a channel by its own number whatever its role, because the firmware
reads the µs as a LEVEL for a role-2 channel (`mav_commands.cpp:471-479`) — so one code path
covers both, and the role check is the only thing distinguishing them.

⚠ **`preflight_roles()` needs the reader thread running.** `get_param` reads the pymavlink
cache and never calls `recv_match()` itself, so with no reader every role reads `None` and the
payload reports UNREADABLE — indistinguishable from a mis-roled board. The manager starts the
reader before `_preflight_payload`; `test_the_reader_thread_starts_before_the_payload_role_read`
pins that ordering.

### ⚠ A timed move is NOT voltage-independent (firmware, confirmed 2026-08-03)

`move_forward --duration N --gain G` is **purely timed** — no distance sensor, no estimate — and
at the firmware's shipped defaults **nothing compensates for battery voltage**. All three
mechanisms that would are off:

| Mechanism | Param | Default |
|---|---|---|
| slow per-thruster RPM trim | `THR_TRIM_EN` | **0** (recommended route; not water-validated) |
| mixer battery feedforward | `MOT_BAT_V_MAX` | **0** = off (also needs `ESPNOW_EN=1` + the 2nd board) |
| Pico closed-loop RPM | `RPM_LOOP` | **0** — ⚠ deliberately, it oscillates in the stabilisation path |

Throttle commands **volts, not thrust** (`RPM ~ duty·V_batt/Kv`, `thrust ~ RPM²`): a T200 at the
same PWM makes 3.71 kgf at 12 V and 6.7 kgf at 20 V. **So the same `move_forward` travels
further on a full pack than a flat one**, and a mission tuned at the start of a session drifts
as the battery drains.

The firmware's `ALGORITHMS.md §11.1` used to promise the opposite ("the same distance every
run, full or low battery") on the strength of an RPM loop that is disabled. Corrected in
`srot-control-board` `fd563cd`. **Plan timed legs at a roughly constant state of charge, or
enable `THR_TRIM_EN` and tune it**, until we have a distance source.

### Display conventions — one rule: match the board

**Heading is `0..360` on every surface.** The board wraps it explicitly for both its OLED and
`VFR_HUD.heading` (`mav_stream.cpp:254-256`), and `SrotFC.get_attitude` / `Telemetry.yaw_deg`
already do the same (`% 360.0`). `ATTITUDE.yaw` is signed radians on the wire, and rendering
that raw printed `yaw -162.23°` next to the board's `heading 197°` — the same angle,
disagreeing by exactly 360, on adjacent lines. **Roll and pitch stay signed** (`CAL_LVL_R/P`
are signed radians; a 3° list to port is not a 357° list).

`srot_format.py` is the single definition of every conversion — `connect`, its dashboard,
`--json` and the manager's `[SROT ]` block all call it, so they cannot drift again. That
module and `srot_changes.py` are pure and unit-tested without hardware.

`connect` prints our heading beside the board's `VFR_HUD` copy on purpose: two numbers that
must agree, side by side, so a future drift is visible in one glance rather than after a dive.

### Absence, on a bare board

With the board alone — no thrusters, no Bar30, no 2nd board — most of the vehicle is
legitimately absent, and every one of these is a place a `0` would have lied:

| Field | Absent because | Trap |
|---|---|---|
| depth / pressure / `WTEMP` | no Bar30 | ⚠ **`VFR_HUD.alt` is NOT gated on baro health** (`mav_stream.cpp:258`), unlike `SCALED_PRESSURE2`/`WTEMP` which *are* suppressed. The board streams `-0.000 m` with no barometer fitted at all, so read the `SYS_STATUS` health bit — never infer health from the presence of a depth value |
| `Vservo` / battery id 1 | `Vservo` **is** PM2, the thruster pack, which arrives over ESP-NOW from the 2nd board | absent ≠ 0 V |
| ESC temps | no Pico, no ESCs | an ESC reporting 0 °C is implausible → an all-zero temp row means nothing is attached. **RPM is different**: 0 RPM is legitimate for a stopped ESC, so only a *missing frame* is `--` |
| `DEPTH_OUT` / `DEPTH_ERR` | the depth loop does not run without a barometer | — |

`Vcc` is a **hardcoded `5000`** in the firmware (`mav_stream.cpp:328`), not a measurement — it
is labelled `(nominal, not measured)` so nobody debugs a 5 V rail off a constant.

An unwired PM1 pin **floats**: the change log caught it oscillating **1.20 ↔ 5.96 V** on the
bare-board bench. That is a floating ADC, not a pack, and it is exactly the kind of thing a
periodic snapshot shows as a plausible-looking single number.

### PlatformIO — we build the firmware now

`pio` 6.1.19 is installed in an isolated venv (`~/.platformio-venv`) so it cannot disturb the
ROS/`pymavlink` site-packages this workspace depends on:

```bash
~/.platformio-venv/bin/pio run -e esp32doit-devkit-v1   # the flight controller
~/.platformio-venv/bin/pio run -e pico                  # RP2350 thruster co-processor
~/.platformio-venv/bin/pio run -e second-board          # thruster-pack voltage TX
```

| env | built 2026-08-03 |
|---|---|
| `esp32doit-devkit-v1` | ✅ RAM 24.3%, Flash 29.6% |
| `pico` | ✅ (14m42s — RP2350 toolchain fetch) |
| `second-board` | ✅ (12s) |
| `groundstation-esp32`, `esp32_4way`, `esp32_4way_diag` | ❌ *"Nothing to build"* — **stale env definitions**, their `build_src_filter` points at `src/groundstation/` and `src/esp32_4way/`, which have never existed in that repo. Those sources live in `srot-ground-station/` and `srot-esc-flasher/src/esp32_4way/` |

⛔ **Build only. Do not flash without an explicit decision.** `app0` must stay at `0x10000`
(PlatformIO hardcodes the app offset and does *not* read it from the CSV — moving it makes
every upload land where the bootloader will not look), and the 20 KB → 128 KB NVS change means
`pio run -t erase` first, which **wipes the `CAL_*` block** — accel, mag and level calibration
plus detected motor directions, recoverable only from a Bondor parameter export.

### Host-side workarounds for firmware defects (see `srot-control-board/JETSON_FEEDBACK.md`)

> **Read `auv-architecture-2026.md` first.** Most of this section is now history. The firmware
> answered nine of the eleven `JETSON_FEEDBACK` items in its Round 6 (`AUDIT.md` R35–R44), and
> the workarounds below have been **removed from the code**, not just annotated. The version
> gate is `srot_protocol.FW_BEHAVIOUR_REV_REQUIRED`; the board reports its own via
> `SROT_FW_BEHAVIOUR_REV` in `include/config.h`.

- ~~**`MOVE_STOP` does not brake**~~ — **FIXED in fw REV 2, and our brake is GONE.**
  The firmware now captures the outgoing leg's axis and speed *before* zeroing them and
  restores both into `PH_BRAKE`, so `MOVE_STOP` decelerates on its own.
  `SrotFC._brake_last_leg` **has been deleted**; `stop_motion()` is a bare `MOVE_STOP`.
  ⚠ **This is the one change with a hull-relevant failure mode in BOTH directions:** run this
  host code against pre-REV-2 firmware and `stop` coasts again; run the old host code against
  REV-2 firmware and the hull is braked **twice** — a reverse leg on top of the board's own
  deceleration. Do not mix.
  **The interlock is `SrotFC.check_behaviour_rev()`**, which runs at connect and again inside
  `arm()`, and **refuses to arm** below `FW_BEHAVIOUR_REV_REQUIRED`. The board reports its
  revision in `AUTOPILOT_VERSION.middleware_sw_version` (request msgid 148); `0` means
  pre-2026-08-01 firmware and fails closed. Override: `allow_fw_behaviour_mismatch:=true`.
  `test_firmware_behaviour_rev_is_new_enough` checks the same number, but it **skips when the
  firmware repo is not checked out beside the workspace — i.e. it skips on the vehicle**.
  That is precisely why the runtime check exists.
  (The previous drift test could not catch this: it grepped the firmware's `Type::STOP` case
  for `abort()` and stayed **green** through the entire fix. Source-text greps across repos
  are not a version contract; a declared revision number is.)
- ~~**A failsafe mid-move never sends a terminal ACK**~~ — **FIXED in fw R35.** A failsafe now
  resolves the in-flight move with a terminal ACK instead of freezing `mv_active=true` and
  streaming `IN_PROGRESS` forever. `_ack_budget_s` is retained but **reframed as a backstop**:
  it is no longer the only terminator, so it does not have to be tight enough to be the
  liveness guarantee. (Separately fixed in fw R44, found on hardware: a resolved move could
  emit a terminal ACK *repeatedly* — ~100 ACCEPTEDs for one DIVE — because the completion path
  used `s_seq = 0` as its "already answered" sentinel and sequence 0 is a legal sequence. It
  now latches an explicit `s_resolved` flag. This bug predates the srot branch.)
- **A FIFTH ACK result exists** — `TEMPORARILY_REJECTED` (3) on a state-mutex miss, not in
  `JETSON_COMMS.md`'s four-result table. Treated as terminal + reported as retryable.
- **Depth sign** — `VFR_HUD.alt` already arrives NEGATIVE-below-surface (our convention, same
  as Pixhawk AHRS2). We were negating it a second time, which made `/duburi/state.depth_m`
  positive when submerged and silently disabled every depth guard in the stack (they all
  compare against a negative constant, so none of them errored — they just stopped firing).
- **⚠ pymavlink's DEFAULT dialect is MAVLink *1* (`dialects.v10.ardupilotmega`), where
  `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` (11030/11031) DO NOT EXIST.** Measured in-vehicle: the
  board's ESC frames arrive and are reported as `UNKNOWN_291` / `UNKNOWN_11030` /
  `UNKNOWN_11031` — zero RPM that looks exactly like an ESC or wiring fault. `MAVLINK20=1`
  must be set **before** pymavlink is imported. `pixhawk.py` has always done this, which is
  the only reason it worked; `fc/srot_fc.py` now does it too, so importing the srot backend
  on its own (`bringup_check`, a test, a script) no longer silently loses ESC telemetry.
  The drift test now asserts against `mavutil.mavlink.mavlink_map` — **the map the vehicle
  actually decodes with** — not against the `dialects.v20` module, which is a different
  object and was passing while the runtime path decoded nothing.
- **`ESC_STATUS` (291) is in NO pymavlink dialect** (upstream removed 290/291 from `common`),
  and pymavlink drops unknown msgids **silently**. This finding was correct and still stands —
  but the firmware now **also** emits `ESC_TELEMETRY_1_TO_4`/`5_TO_8` (11030/11031), which
  decode fine, so the fallback you already wrote is live. **`/duburi/esc_rpm` is now
  published** (`_publish_srot_telemetry`), and LEAK is surfaced on an edge latch rather than
  per-tick spam. Note `MAV_CMD_SET_MESSAGE_INTERVAL` needs msgid 291 as a *number*:
  `mavutil.mavlink.MAVLINK_MSG_ID_ESC_STATUS` **does not exist** and referencing it is an
  `AttributeError` at import, so `srot_protocol.MSG_ID_ESC_STATUS = 291` is a literal.
- **`NAMED_VALUE_FLOAT` multiplexing is a real hazard** (your finding, and a good one):
  pymavlink caches exactly one message per msgid, so with MV_STATE / LEAK / WTEMP / GAIN all
  riding NAMED_VALUE_FLOAT, whichever arrived last wins and **LEAK detection is
  probabilistic**. Not yet fixed either side. Firmware proposal: move LEAK onto `SYS_STATUS`
  sensor-health bits, where it is a dedicated bit that cannot be overwritten by a temperature
  reading. Say the word and it lands.
- **⚠ params may not have persisted before firmware `8cb4203`** (fw R14: the NVS partition
  was too small, writes silently failed). Assume `set_default_gain()`'s `JS_GAIN_DEFAULT=1.0`
  never stuck → `MANUAL_CONTROL` at half authority. **Reflash + read back `GAIN`.**

## Status (branch `srot`)
- **DONE:** the HAL foundation (`14eb27a`) + integration doc (`6f3aea5`) + **the manager
  wiring** (`7f3d6e8` + review fixes): `flight_controller:=srot` is the **default**, connects
  over **direct USB serial**, banner shows `SROT board · firmware Hengla · USB serial`, the
  collapse verbs route through `fc.move()` + the 4-terminal ACK relay, telemetry populates
  `/duburi/state`. Build clean; ~670 tests green. mavlink-reviewer + advisor signed off (arc
  dropped, `manual()` NaN-safe, verified fail-closed arm-abort, HEARTBEAT source-filtered).
- **READINESS FIXES (this pass):** real abort brake, leg-derived ACK deadline, the 5th ACK
  result, `surface` revived, `UNSUPPORTED_VERBS` guard, depth sign, collapse verbs now take
  `duburi.lock` + the disarmed gate, fail-loud payload map, protocol drift test.
  **Verdict: ready for tethered bench work; the two depth checks below still gate any dive.**
- **VISION IS NOW SPEC'D, NOT PORTED.** The `motion_vision` port is **superseded**: rather than
  re-expressing the 20 Hz host loop as `manual()` streaming, the loop **moves to the board**.
  We stream one `LANDING_TARGET` (149) per frame as a **bearing in radians** and the board
  closes every axis at 500 Hz. Spec: `Mongla_others/srot-control-board/VISION_API.md`; our
  side: [`vision-control-split.md`](vision-control-split.md). Until the firmware implements it,
  `vision_align`/`vision_move` stay in `UNSUPPORTED_VERBS` and the host loop is unchanged.
- **NEXT:** the DVL-distance streamed path + `lock_heading` semantics; commit the Bondor
  `.params` export. `/duburi/esc_rpm` is **done** (was blocked on the firmware emitting
  11030/11031 — it now does). Once the board serves vision: **measure the two cameras' FOV at
  640×480** → angle conversion → uplink → re-point the two verbs. That measurement is the
  critical path and it is a bench task, not a code task — see `auv-architecture-2026.md` §"The
  one thing blocking vision".
- **Cross-repo rules:** [`cross-repo-contract.md`](cross-repo-contract.md) (mirrored as
  `AGENTS.md` in each sibling repo).
- **BENCH-GATED:** the runbook above (needs the board; first real validation — no SROT SITL).
  Plug in USB, `ros2 run duburi_manager start`, watch the banner + `/duburi/state`. Depth stays
  unproven until the hand-verification (step 4) passes.
