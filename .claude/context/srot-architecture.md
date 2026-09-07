# The srot board / Hengla firmware — architecture, why it is built this way, and where we disagree

> **Status:** written during the round-40 exhaustive read of `srot-control-board`,
> `srot-ground-station` (Bondor) and `srot-esc-flasher`. Every claim below is
> tagged **READ** (traced in source, with `file:line`) or **MEASURED** (a number
> taken off the live board or computed from live parameters). Nothing here is
> taken from their docs alone — where a doc and the source disagree, that
> disagreement is itself recorded.
>
> This is not a defect list. It is what the firmware *is*, why its authors chose
> that, what the choice costs, and what we should do on our side because of it.

---

## 0. The two names, and why both are load-bearing

**SROT is the board. Hengla is the firmware.** (`config.h:14-33`, READ.) They are
not interchangeable: SROT names the hardware, the NVS namespaces, the repos and
`MAV_CMD_SROT_MOVE`; Hengla names the software. `HENGLA_FW_VERSION_STR` is
`"Hengla v0.2.0"`; the wire reports `MAV_AUTOPILOT_GENERIC` on a
`MAV_TYPE_SUBMARINE` frame.

That autopilot id is a deliberate, documented cost: QGroundControl's and BlueOS's
setup pages are ArduPilot-gated and therefore stay empty. **Bondor is the ground
station** — not QGC. Their own comment records that an `APM_COMPAT_*` block once
spoofed "ArduSub V4.1.0" to unlock those pages and was removed rather than left
to mislead.

**Our opinion: correct, and we should stop treating QGC compatibility as a goal.**
An honest generic-autopilot id that makes a GCS refuse is better than a spoofed
one that makes a GCS show a *wrong* setup page for a vehicle whose parameters only
resemble ArduPilot's by naming convention.

---

## 1. The shape of the machine

**Dual-core ESP32 + RP2350 co-processor.** (`config.h` §6, READ.)

| core | tasks | rate | priority |
|---|---|---|---|
| **Core 1 — flight** | `Task_SensorRead` | **500 Hz** | 6 |
| | `Task_ControlLoop` | **500 Hz** | 5 |
| | `Task_DShot_RMT` | **500 Hz** | 5 |
| **Core 0 — comms** | `Task_MAVLink` | 100 Hz | 2 |
| | `Task_UI_Status` | 30 Hz | 1 |
| | `Task_LoRa_SD` | 20 Hz | 1 |
| | `Task_Buzzer` | 200 Hz | 1 |

The split is the whole design: **nothing on Core 0 can delay the flight loop.**
A GCS burst, an OLED redraw or an SD write cannot stretch a control period.
`Task_SensorRead` sits *above* the control loop in priority so a transient
overrun cannot pile up behind stale data.

**500 Hz is chosen, not inherited.** The comment at `config.h:365` states why:
2 ms is the fastest whole-millisecond period on the 1 kHz FreeRTOS tick, so the
loop is tick-exact and `CONTROL_LOOP_DT` is honest. "400 Hz" would still run at
2 ms but with a `dt` that lies to every integrator.

> ⚠ **DOC DRIFT (READ).** `config.h:323`, the §6 header, says *"Rates: control
> loop 200 Hz; sensor read 200 Hz; DShot output 200 Hz."* All three are 500.
> Already filed upstream.

### Communication is through one global, one mutex per sub-struct

`SystemState` (`state_types.h`) holds six sub-structs — sensors, control,
thrusters, indicators, aux, cal — each with its own `SemaphoreHandle_t`. The
rules are stated and enforced: take one mutex, copy fast, give it back, never
nest, and **real-time callers pass a short timeout and skip the cycle on a miss**
(`StateLock(m, pdMS_TO_TICKS(2))`).

The default `StateLock` timeout is **20 ms and deliberately bounded, not
`portMAX_DELAY`** — a comms RX handler must never freeze forever on a contended
mutex.

**Our opinion: this is the right structure for this problem**, and notably better
than a queue-per-signal design would be, because every consumer wants a
*coherent snapshot* rather than a history. The one thing it cannot express is
"this field is stale", which the firmware then has to bolt on per-field — see §2.

---

## 2. "Absence is the signal" — the firmware's own best idea

The most valuable single principle in this codebase, and it is theirs:

> `mav_stream.cpp:813` — **"Absence is the signal."**

Applied consistently, a field that cannot be trusted is **suppressed**, not
zeroed, because a consumer cannot distinguish a measured zero from a missing one.
Traced instances (READ):

| signal | suppressed when | rev |
|---|---|---|
| `SCALED_PRESSURE2` | baro unhealthy or stale (it has no validity field) | 3 |
| `NAMED_VALUE_FLOAT("WTEMP")` | same | 3 |
| `DEPTH_ERR` / `DEPTH_OUT` | the depth controller is not running | 8 |
| `BATTERY_STATUS` id 0 / id 1 | `pm1_present` / `pm2_present` false | — |
| `SCALED_IMU2.temperature` | sends `0`, MAVLink's documented "not provided" sentinel | 3 |

The same idea is carried in the state structs as a **value + stamp + valid
triple**: `imu_stamp_ms`/`imu_valid`/`imu_ever_valid`, `baro_stamp_ms`/
`baro_valid`, `aux_stamp_ms`/`aux_valid`, `sp_stamp_ms`.

`imu_ever_valid` deserves its own note (`task_control_loop.cpp:767`, READ): the
IMU-lost warning is gated on having *ever* had a fix, because at power-on the BNO
needs seconds and the naive debounce "reported a LOST IMU on every boot — crying
wolf about the one sensor you must be able to trust a warning from."

**This principle is the single most useful thing we have adopted from them**, and
it is the same rule that caught `BARO_HEALTH = 3` read as a health score, the
`esc_msgs` gate, and the all-zero ESC frames on our side.

### Where they do NOT apply their own rule

Two places, both ours to work around today:

1. **`KILL`** — `espnow_link.cpp:49` is `kill = f ? s_kill : false;` with the
   comment *"link lost → don't assert kill (display-only)"*, and
   `mav_stream.cpp:846` sends `sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f)`
   unconditionally. So a **missing** 2nd board and a **clear** kill switch are
   the same byte on the wire. The struct itself even carries `aux_valid` for the
   voltage on the same link, so the freshness exists and is simply not applied to
   this field. Filed as PR #12; handled on our side as a tri-state via
   `BATTERY_STATUS` id 1 presence as an ESP-NOW liveness proxy.
2. **`ESC_TELEMETRY.count` and ESC presence** — see §5.

---

## 3. ⛔ THE ACTUATOR IS A RELAY. This is the finding that matters most to us.

**MEASURED, on the vehicle's own board, 2026-09-07.** Not defaults — read back
over MAVLink with the manager stopped:

```
MOT_SPIN_MIN   0.15      PILOT_EXPO   0.30      JS_GAIN_DEFAULT  1.0
MOT_THST_EXPO  0.65      PILOT_SPEED  1.0       FRAME_REVERSE    1
MOT_SPIN_ARM   0.0       PILOT_YAW_RATE 160     RPM_LOOP         0
```

### The chain, traced end to end (READ)

Every control output on this vehicle — pilot stick, attitude PID, depth PID,
AUTO movement primitive, and **our vision-driven `MANUAL_CONTROL`** — passes
through exactly one mixing site, `task_control_loop.cpp:897-898`:

```
MANUAL_CONTROL.y
  → sp_lateral = (y/1000) · GAIN                      mav_commands.cpp:717
  → lat = ((1−PE)·l + PE·l³) · PS                     computeDemands
  → mixer::mix()   lateral column is ±1 on motors 1-4
  → mixer::toDshot() → oneToDshot():
        if |t| < 0.005      → NEUTRAL (1048), motor STOPPED
        else  thr    = thstExpo(t, 0.65)
              shaped = 0.15 + 0.85·thr                ← THE FLOOR
              dshot  = 1049 + shaped·998
```

### What that does, computed from the live values (MEASURED)

| host stick | axis demand | DShot band | **delivered thrust** |
|---|---|---|---|
| 0.71 % | 0.00497 | 0 % (DShot 1048) | **0 %** |
| 0.72 % | 0.00504 | **16.13 %** (DShot 1210) | **7.37 %** |
| 1 % | 0.0070 | 16.6 % | 7.62 % |
| 2 % | 0.0140 | 19.3 % | 8.51 % |
| 10 % | 0.0703 | 32.5 % | 15.1 % |
| 50 % | 0.3875 | 70.0 % | 46.3 % |

> **VERIFIED against the firmware's own arithmetic.** The band figures above began
> as a Python reimplementation — our model of their model. Both `thstExpo()` and
> `oneToDshot()` were then transcribed verbatim into C, compiled, and compared:
> **all 10 DShot integers match exactly** (1048, 1048, 1210, 1221, 1242, 1298,
> 1373, 1493, 1748, 2047). The battery-feedforward branch is inert here
> (`MOT_BAT_V_MAX = 0` → `batteryScale()` returns 0). The **thrust** column is
> still a model — it inverts their `thrust ≈ (1−e)·thr + e·thr²` comment and has
> not been measured, and cannot be until thrusters exist.

**There is a cliff at 0.714 % of full stick, and crossing it puts the motor at DShot 1210 — **16.13 % of the throttle band** —
instantly.** Between "nothing" and "7.4 % of every
thruster on the axis" there is no reachable value.

> **IMPLIED, on stated assumptions — not measured.** Converting the band figure
> to newtons needs three numbers this read did not establish: T200 at 5.25 kgf,
> four horizontals at exactly 45°, and a 20 kg hull. The mixer matrix is a
> **normalised demand mix of ±1, not geometry** (our own round-39 note), so the
> cos 45° is an assumption about the frame, not a fact read from the firmware.
> On those assumptions full lateral is ≈ 145.6 N and the smallest commandable
> lateral force is ≈ 10.7 N ≈ 0.54 m/s². **The defensible numbers are the band
> percentages above**, which follow from source and the live parameters alone.
> Nothing upstream should lead with the newton figure.

### Why the firmware is not wrong to have a floor

`MOT_SPIN_MIN` went `0.10 → 0.02 → 0.15` and the reasons are recorded and
measured (`config.h`, READ). At 0.02 an in-water test logged repeated
*"Thruster N STALLED: dshot=146 rpm=0"*. **Props have stiction, and a floor that
guarantees break-away is genuinely necessary.** The Pico's closed-loop path has
the identical floor for the identical reason (`pico/main.cpp:258-269`).

### Why it is nonetheless the wrong shape

A break-away floor applied to a **continuous stabilisation demand** converts the
actuator into a **relay**, and a relay inside a feedback loop limit-cycles by
construction. That is a control-theory fact, not a tuning opinion, and it
predicts every close-in symptom we have recorded:

- the hull oscillates when the bounding box is large and corrections are small
- "moves too fast to align" — because the smallest available move is 10.7 N
- the torpedo shot cannot be held still
- **and it explains why `vision.range_gain_floor` measured as useless (round 35)
  and is still shipped OFF.** Softening host gain cannot soften the output. It
  only pushes commands across the cliff less often, which converts a continuous
  correction into a lower-duty relay — *more* limit cycle, not less.

That last point is the important one: **we spent a round measuring a host-side
knob that could not work, because the quantisation is downstream of it.**

### The fix we will propose: make the floor hysteretic, not absolute

Stiction is a **break-away** phenomenon — it takes more force to start a stopped
prop than to keep a turning one turning. The floor is therefore only needed when
the prop is *stopped*.

**And the board already measures exactly that.** Per-thruster RPM crosses the
Pico link every cycle and lands in `g_state.thrusters.rpm[]`, where today it is
used only for stall detection and telemetry. So:

```
if the prop is stopped (|rpm| ≈ 0)  → apply MOT_SPIN_MIN to break away
if the prop is already turning       → no floor; allow arbitrarily small demands
```

This keeps the floor exactly where the water test proved it necessary, and
removes it exactly where precision matters — during a hold, when the thrusters
are already active. It is a few lines, it needs no new wire, and it uses a signal
they already have and currently spend on nothing but a warning.

> ⛔ **The fallback is not optional, and it sequences this behind PR #10.**
> On this vehicle RPM has been **exactly 0 in 958/958 recorded frames**
> (MEASURED). A floor gated on `rpm > threshold` is therefore a **no-op on any
> hull whose bidirectional-DShot telemetry is not working** — which per
> `ROADMAP.md:734` needs ESCs in 3D mode plus a ~2.2 kΩ pull-up each. Worse, if
> telemetry works on *some* thrusters, an axis whose four motors disagree about
> their floor produces **a torque nobody asked for** — a new failure mode
> introduced by the fix. So the rule must be stated as:
>
> **no valid RPM for a thruster → keep the floor.** Fail to today's behaviour,
> per-thruster. That makes the change safe to merge before the ESC-telemetry
> situation is resolved, and it makes presence-on-the-wire (PR #10) the
> precondition for this being anything but inert here.

**Rejected alternative, recorded so it is not re-proposed:** time-domain dither
at the 500 Hz loop rate does *not* work here. A 2 ms pulse is far shorter than
the T200's rotor time constant (~0.15 s), so the prop never spins up and the
dither delivers no average thrust. A dither slow enough to break away (≈ 2-3 Hz)
would be visible as pulsing and would sit inside the hull's own dynamics.

### What we must do on our side regardless of upstream

Our control law currently behaves as though it commands a continuous force. It
does not. Until the floor changes, **the honest model of this actuator is
quantised**, and the host should either:

- refuse to emit commands in the dead zone (they do nothing — silent no-op, the
  shape that has cost this project three rounds), or
- control the **duty** rather than the amplitude in the terminal regime.

Either way the first step is that `measured-bars.md` must carry the cliff
(0.714 % stick, 7.37 % thrust) as a hard bar, because every gain we tune above it
is tuning a number the hardware cannot deliver.

---

## 3b. ⛔ WORSE THAN THE STEP: the deadband bends the DIRECTION of the thrust

§3 is a *magnitude* problem. Mixed-axis commands — which is every real vision
align, since `lat` and `yaw` are corrected together — have a *direction* problem,
and it cannot be fixed by any choice of gains.

### Where it comes from (READ, then proved algebraically)

The four horizontal rows of the mixer (`mixer.cpp:26-34`) give, for a lat+yaw
command:

```
m1 = +(yaw + lat)      m3 = +(lat − yaw)
m2 = −(yaw + lat)      m4 = −(lat − yaw)
```

The motors **pair up**: `m1,m2` see `|yaw+lat|` and `m3,m4` see `|lat−yaw|`. So
the two pairs cross the 0.005 deadband *at different commands*.

**When `|lat − yaw| < 0.005`, m3 and m4 stop entirely and only the m1/m2 pair
runs.** That pair's yaw and lat coefficients are equal, so:

```
achieved_yaw = ( +1·T₁ − 1·T₂ ) / 4 = T₁/2        (T₂ = −T₁)
achieved_lat = ( +1·T₁ − 1·(−T₁) ) / 4 = T₁/2
⇒ achieved yaw/lat is EXACTLY 1.0, whatever was requested.
```

The horizontal group has **two attractor directions, ±45°**, and it snaps to them
inside a band. Symmetrically, `|lat + yaw| < 0.005` leaves only m3/m4 and locks
the ratio to −1.

### Measured consequence (computed from live parameters)

Lateral held at 2 % stick, yaw swept:

| yaw stick | commanded direction | achieved direction | error |
|---|---|---|---|
| 0.20 % | 8.13° | 1.68° | −6.45° |
| 0.60 % | 23.20° | 5.04° | −18.15° |
| 0.80 % | 29.74° | 6.72° | −23.02° |
| **1.00 %** | 35.53° | **45.00°** | **+9.47°** |
| 1.40 % | 45.00° | 45.00° | 0.00° |
| 1.60 % | 48.81° | 45.00° | −3.81° |
| **2.00 %** | 55.00° | **79.42°** | **+24.42°** |
| 6.00 % | 76.86° | 83.55° | +6.69° |

**Worst direction error: 25.6°.** And note the transition between the 0.80 % and
1.00 % rows: the achieved direction jumps **6.72° → 45.00°, a 38° discontinuity,
for a 0.2 % change in command.** The snap band at this lateral demand is 1.0 % of
stick wide — **71 % of the lateral demand itself**.

### Why this is the more damaging of the two

A gain error scales a correction; the loop still pushes the right way and a lower
gain still converges. **A direction error that is discontinuous and
non-monotonic in the command means the actuator can push somewhere other than
commanded, and can change where it pushes abruptly as the controller sweeps its
own ratio during an approach.** No `kp` stabilises that.

This is a complete, sufficient, second explanation for terminal-alignment
instability, it is independent of §3, and it predicts the specific thing we see:
**the hull yaws when asked to strafe, near the target, at small corrections.**

### What it changes on our side, immediately and without firmware

Our two-axis vision align commands `lat` and `yaw` **simultaneously**, straight
into this failure. Three responses, in order of cost:

1. **Do not command lat and yaw together in the terminal phase.** We already have
   the mechanism and already recommend it for a different reason:
   `precision-alignment.md` says drop the `yaw` axis at the hole and let
   `heading_lock` hold Ch4. **That advice is now much better founded than when it
   was written** — it was justified as avoiding vision-yaw wobble; it in fact
   avoids the mixer's ±45° attractor entirely, because a single-axis command
   keeps all four horizontals at equal magnitude (the §3 pure-lateral rows show
   ratio error exactly 0).
2. **Never emit a command inside the dead zone.** Below 0.714 % stick the
   thruster does nothing; a loop that believes it commanded a small correction
   and got none is the silent-no-op shape that has cost this project three
   rounds.
3. **Prefer sequential single-axis corrections to simultaneous ones** whenever
   the residual is small, which is exactly the regime where the attractor bites.

---

## 4. The live configuration of THIS hull, and one change nobody told us about

Read back over MAVLink with the manager stopped, 2026-09-07 (MEASURED):

| param | live | default | note |
|---|---|---|---|
| `MOT_SPIN_MIN` | **0.15** | 0.15 | the §3 floor — real, not a stale default |
| `MOT_THST_EXPO` | 0.65 | 0.65 | |
| `MOT_SPIN_ARM` | 0.0 | 0.0 | nothing turns on arm |
| `PILOT_EXPO` | 0.30 | 0.30 | small-signal authority = GAIN·(1−EXPO) |
| `PILOT_SPEED` | 1.0 | 1.0 | |
| **`PILOT_YAW_RATE`** | **160** | 45 | **3.6× the default** — full yaw stick is 160 °/s |
| `JS_GAIN_DEFAULT` | 1.0 | — | so live GAIN is 1.0, not the 0.5 that would halve every axis |
| **`FRAME_REVERSE`** | **1** | 0 | set on this hull; undocumented in `PARAMETERS.md` (filed) |
| `FRAME_CONFIG` | 2 | | |
| `MOT_BAT_V_MAX` | **0.0** | 0.0 | battery feedforward **OFF** — ledger item, still open |
| `THR_TRIM_EN` | **0.0** | 0.0 | slow RPM thrust normalisation **OFF** ("turn on in water") |
| `RPM_LOOP` | 0 | 0 | open loop — the recommended setting, see §5 |
| `RPM_MAX` / `RPM_MIN_TGT` | 3600 / 30 | | T200 ceiling at 16 V |
| `DEPTH_P` / `I` / `D` | 0.5 / 0.1 / 0.01 | | the in-water-measured values (2026-08-07) |

`PILOT_YAW_RATE = 160` is worth its own line: **our vision yaw gain is tuned
against a full-scale that is 3.6× the firmware default.** Any yaw gain reasoning
that assumed 45 °/s is wrong by that factor.

### ⚠ `MOT_n_DIRECTION` has CHANGED since round 26 and we were not told

| | M1 | M2 | M3 | M4 | M5 | M6 | M7 | M8 |
|---|---|---|---|---|---|---|---|---|
| round 26 banner (`CFG r14 … DIR=`) | −1 | 1 | 1 | 1 | **1** | **1** | **1** | **−1** |
| live, 2026-09-07 | −1 | 1 | 1 | 1 | **−1** | **−1** | **−1** | **1** |

Four of eight flipped — the entire **vertical** group (M5-M8) inverted. Somebody
ran `MOTOR_DETECT` or set directions by hand between those dates.

**Recorded, not chased.** But two consequences follow immediately: any vertical
behaviour we characterised before this change is stale, and this is precisely
the non-uniform flip pattern `FRAME_REVERSE`'s own comment warns about — except
here it is uniform *within* the vertical group, so the mixer's roll/pitch/throttle
patterns survive. **It is a whole-group inversion, which is the benign case.**
Worth confirming with the operator that it was deliberate.

---

## 5. Heading: absolute by a one-shot offset, then free-running for ever

This is the subsystem our missions lean on hardest and it is the one whose design
we understood least. Traced through `config.h`, `yaw_ref.cpp` and
`attitude_control.cpp` (READ).

### The design

**Attitude never uses the magnetometer.** Roll, pitch and yaw come from the
BNO085's **6-axis GAME rotation vector** — mag-free by construction — so
"thruster/metal fields can never move roll/pitch/yaw" (`config.h:160`). That is a
deliberate and, for an aluminium hull with eight thrusters, an excellent choice.
It is the same conclusion we reached independently on the Pixhawk path, where we
distrust ArduSub's compass inside the hull.

The magnetometer is then used **exactly once**:

```
yaw_ref::update()   — disarmed only, never while armed
  ├ require our own stored mag cal (NVS), else BNO accuracy ≥ 2
  ├ require |B| inside [MAG_FIELD_MIN, MAX]  (default 8–120 µT, deliberately WIDE)
  ├ tilt-compensate with the mag-free roll/pitch
  ├ accumulate ≥ 100 samples over ≥ 1000 ms
  ├ stability gate: circular resultant R ≥ cos(8°)
  └ offset = circularmean(mag_heading − game_yaw) + MAG_DECL     → State::LOCKED
```

`YAW_REF` publishes the state, and **only `YAW_REF == 2` (LOCKED) means
`ATTITUDE.yaw` is absolute.** The refusal is *sticky* by design — retrying forever
would flap the reported heading; `MAG_ALIGN` is the explicit retry.

### The design decisions worth admiring

- **The refusal quotes its own numbers.** `"Mag ref refused: |B|=155uT, want
  8-120"`. Three separate versions of this refusal fired for reasons the operator
  could not influence (the feature defaulted off; then a BNO accuracy flag that
  *never converges on this board* because `sh2_setCalConfig()` is rejected; then a
  free-air field band that rejected a perfectly good attenuated hull reading of
  14.7 µT). Each fix is recorded in the source. **This is the "silent success /
  unactionable failure" discipline applied better than we apply it.**
- **The stability gate is the circular resultant `R`, not max−min** — because
  max−min is wrong across the ±π wrap. We made *exactly this mistake* in the
  round-27 `.tlog` analyser, where "25.5° of raw jitter" on a smooth sweep was one
  359→0 crossing.
- **Never align while armed**, because the thrusters are the interference source.

### ⛔ The cost, and it is large for a competition run

**The offset is a constant, captured once, and never revisited. The 6-axis yaw
underneath it free-runs.** Their own comment states the consequence
(`config.h:544`, READ):

> *"This fixes the yaw REFERENCE, not the drift: the 6-axis yaw still creeps
> ~0.5-3 deg/min, so absolute heading degrades over a long dive. That is the
> accepted trade for immunity."*

At 0.5–3 °/min, a 15-minute RoboSub run accumulates **7.5° to 45° of heading
error**, silently, with `YAW_REF` still reporting LOCKED the whole time. Every
absolute `MOVE_TURN`, every `lock_heading`, and every dead-reckoned leg inherits
it. **Nothing on the wire says the heading has gone stale, because from the
firmware's point of view nothing has changed.**

This is the accepted trade and it is defensible — immunity to thruster fields is
worth a lot. But it means **absolute heading is a decaying asset with a known
half-life, and our mission planning must treat it as one.** Concretely: a mission
that turns to an absolute heading late in a run is trusting a number that may be
tens of degrees wrong.

> **MEASURED on this board — see §5b.** The datasheet range is 0.5–3 °/min; we
> measured our own board rather than inherit a range.

---

## 6. The two other deadbands we are driving into

§3 and §3b are about the mixer. There are two more quantisers *upstream* of it,
both in `attitude_control.cpp` / `depth_control.cpp` (READ), and both swallow our
vision corrections whole.

### 6a. The yaw stick deadband: below 2.86 %, STABILIZE HOLDS instead of turning

`attitude::stabilize()` branches on the **expo'd** yaw stick:

```
yaw_stick = applyExpo(stick_yaw, PILOT_EXPO)
if (|yaw_stick| > 0.02)  → rate command: des_yaw_rate = yaw_stick · PILOT_YAW_RATE
else                     → HOLD: des_yaw_rate = wrapPi(s_yaw_target − meas_yaw) · ANG_YAW_P
```

With `PILOT_EXPO = 0.30`, `|yaw_stick| > 0.02` needs **|stick| > 2.856 %**
(MEASURED, solved numerically from the live expo).

So:

- **A yaw command below 2.86 % of stick does not turn the vehicle at all.** It
  takes the ELSE branch, and the board *actively drives back* to the heading it
  captured. Our loop believes it commanded a small yaw; the vehicle holds.
- With `PILOT_YAW_RATE = 160` (live), the **smallest commandable yaw rate is
  0.02 × 160 = 3.20 °/s.** There is nothing between "hold" and "3.2 °/s".

For terminal alignment on a torpedo hole, where the residual bearing error is a
fraction of a degree, **the entire useful range is inside the deadband.**

**The good news, and it is genuinely good:** the ELSE branch is a real
heading-hold closed on the mag-free yaw with `ANG_YAW_P = 6.0`. So dropping the
vision `yaw` axis at the hole does not surrender heading — it hands it to a
controller that holds it properly. `precision-alignment.md` already recommends
this; **it is now justified by the firmware rather than by observed wobble.**

### 6b. The depth stick deadband is 5 %

`depth::update()` moves the depth target only when `|stick_throttle| > 0.05`
(`STICK_DEADBAND`, READ). Below that the target does not move at all. Our
`depth_step` logic steps the *setpoint* host-side, which is the right shape — but
any attempt to trim depth by holding a small throttle offset does nothing.

### 6c. ⚠ The depth loop has NEVER RUN CLOSED — and its sign was inverted

`depth_control.cpp` carries this, verbatim (READ):

> *"AUDIT R1 has stood since the beginning: this loop has NEVER run closed,
> because the Bar30 was not fitted during development."*

The sign **was inverted** and is fixed: the old code ran the PID in depth while
its output lives in altitude, so it "commanded the opposite of what it wanted, on
every axis of the depth loop." The fix is well-argued and cross-checked against
three independent sources. **But it is unvalidated.**

**The SURFACE failsafe depends on this sign.** A leak, a low thruster battery or a
GCS loss all route to `SURFACE`, which calls `depth::update()`. If the sign were
still wrong, the emergency ascent drives the vehicle *down*.

**They built us the tool to check it without water, and we have never used it.**
`depth::preview()` runs the same error expression through a *separate*
proportional-only instance and publishes it as **`DEPTH_CMD`**, readable
**disarmed, with nothing spinning**:

```
thumb over the Bar30 port (pressurise = "deeper")
  measured DEEPER than target   → DEPTH_CMD POSITIVE → ascend   ✓ correct
  measured SHALLOWER than target → DEPTH_CMD NEGATIVE → descend ✓ correct
  demand moves AWAY from target  → STILL INVERTED — DO NOT DIVE
```

They even record why the first version of `preview()` was useless (a full PID fed
a constant error winds its integrator to the rail and reports "inverted" for a
correct loop) — P-only is deliberate.

**This is a pre-water gate we can close on the bench today, and it is the highest-
consequence unvalidated sign in the stack.** It belongs in `bringup_check`.

---

## 5b. Heading, MEASURED — and the drift is NOT the number that matters

Five still-bench captures against `/duburi/state` at 22 Hz, board untouched. The
last three were run **back to back in one session** specifically to settle a
claim an earlier version of this section made and got wrong.

| capture | window | full-run slope | quarter spread | **p2p wander** |
|---|---|---|---|---|
| A | 98 s | **−0.386 °/min** | — | 2.05° |
| B | 480 s | +0.360 | 0.151 | 9.16° |
| C1 | 480 s | **+0.027** | 0.356 | 5.93° |
| C2 | 480 s | **+0.377** | 1.890 | 6.18° |
| C3 | 480 s | **+0.306** | 1.231 | 6.97° |

### ⛔ RETRACTED: the sign does not flip. That was estimator noise.

An earlier commit in this round claimed session-varying bias on the strength of
A (−0.386) against B (+0.360), and the memory entry said so. **The three
consecutive captures are all POSITIVE** (+0.027, +0.377, +0.306), and run A was
the *shortest* window and therefore the noisiest. There is no sign flip in the
data.

Mean of the three consecutive: **+0.237 °/min, sd 0.185** — that is **1.3 σ from
zero with n = 3.** The drift is not resolvable to better than a few tenths of a
degree per minute by this method over eight minutes.

**Also retracted, now by data as well as by argument:** the claim that "the
quarters are the better estimator". Run B's quarters agreed to 0.151 °/min and
that was a **fluke** — the consecutive captures give quarter spreads of 0.356,
1.890 and 1.231. A shorter window has *less* lever arm, so a wander-induced slope
contributes **more** variance per window, not less. Four numbers agreeing once,
with no error bar on any of them, was never evidence.

**What survives:** the drift magnitude is **≤ ~0.4 °/min and may be much
smaller** — comfortably better than the firmware's own stated 0.5–3 °/min.

### The robust number is the WANDER, and it is reproducible

| | p2p over 8 min |
|---|---|
| C1 / C2 / C3 | 5.93° / 6.18° / 6.97° |
| mean | **6.36°, sd 0.54 — reproducible to 9 %** |

**≈ 6–7 ° of heading wander on a motionless board, every time.** Unlike the drift
this reproduces cleanly across captures, and it is **larger than most alignment
tolerances in this stack while being present before the vehicle has moved.** It
competes directly with the mixer's ±45° attractor (§3b) as an explanation for why
terminal alignment is hard.

Not yet separated: sensor vs room (thermal vs mechanical). Two captures an hour
apart, or one with the board on foam, would distinguish them.

### The mission budget

Over a 15-minute run: **≈ 3.5° of drift + ≈ 6° of wander ≈ 10° of heading
uncertainty**, and none of it is correctable from a stored per-board constant —
the mag reference is captured once at boot and never revisited (§5). Treat
absolute heading accordingly: prefer relative turns and vision-referenced
headings, and be suspicious of any mission step that turns to an absolute heading
minutes in.

### Method notes, both earned the hard way this round

**A first 480 s capture reported "+38.4 °/min" and was contaminated** — 274° of
total yaw change, +63 °/min in one quarter and ≈0 in the other three: the hull had
been moved. The split/quarters check caught it; a single slope would have been
reported, exactly as in round 26's bogus +51.9 °/min.

**And then the same check misled me in the other direction** by making a fluke
look like a confirmation. The lesson is not "use quarters" — it is that **an
estimator you have not characterised should not be used to support a claim about
a difference between two of its outputs.** Three consecutive captures cost 24
minutes of wall clock and settled it; the argument would not have.

## 6. The two other deadbands we are driving into

§3 and §3b are about the mixer. There are two more quantisers *upstream* of it,
both in `attitude_control.cpp` / `depth_control.cpp` (READ), and both swallow our
vision corrections whole.

### 6a. The yaw stick deadband: below 2.86 %, STABILIZE HOLDS instead of turning

`attitude::stabilize()` branches on the **expo'd** yaw stick:

```
yaw_stick = applyExpo(stick_yaw, PILOT_EXPO)
if (|yaw_stick| > 0.02)  → rate command: des_yaw_rate = yaw_stick · PILOT_YAW_RATE
else                     → HOLD: des_yaw_rate = wrapPi(s_yaw_target − meas_yaw) · ANG_YAW_P
```

With `PILOT_EXPO = 0.30`, `|yaw_stick| > 0.02` needs **|stick| > 2.856 %**
(MEASURED, solved numerically from the live expo).

So:

- **A yaw command below 2.86 % of stick does not turn the vehicle at all.** It
  takes the ELSE branch, and the board *actively drives back* to the heading it
  captured. Our loop believes it commanded a small yaw; the vehicle holds.
- With `PILOT_YAW_RATE = 160` (live), the **smallest commandable yaw rate is
  0.02 × 160 = 3.20 °/s.** There is nothing between "hold" and "3.2 °/s".

For terminal alignment on a torpedo hole, where the residual bearing error is a
fraction of a degree, **the entire useful range is inside the deadband.**

**The good news, and it is genuinely good:** the ELSE branch is a real
heading-hold closed on the mag-free yaw with `ANG_YAW_P = 6.0`. So dropping the
vision `yaw` axis at the hole does not surrender heading — it hands it to a
controller that holds it properly. `precision-alignment.md` already recommends
this; **it is now justified by the firmware rather than by observed wobble.**

### 6b. The depth stick deadband is 5 %

`depth::update()` moves the depth target only when `|stick_throttle| > 0.05`
(`STICK_DEADBAND`, READ). Below that the target does not move at all. Our
`depth_step` logic steps the *setpoint* host-side, which is the right shape — but
any attempt to trim depth by holding a small throttle offset does nothing.

### 6c. ⚠ The depth loop has NEVER RUN CLOSED — and its sign was inverted

`depth_control.cpp` carries this, verbatim (READ):

> *"AUDIT R1 has stood since the beginning: this loop has NEVER run closed,
> because the Bar30 was not fitted during development."*

The sign **was inverted** and is fixed: the old code ran the PID in depth while
its output lives in altitude, so it "commanded the opposite of what it wanted, on
every axis of the depth loop." The fix is well-argued and cross-checked against
three independent sources. **But it is unvalidated.**

**The SURFACE failsafe depends on this sign.** A leak, a low thruster battery or a
GCS loss all route to `SURFACE`, which calls `depth::update()`. If the sign were
still wrong, the emergency ascent drives the vehicle *down*.

**They built us the tool to check it without water, and we have never used it.**
`depth::preview()` runs the same error expression through a *separate*
proportional-only instance and publishes it as **`DEPTH_CMD`**, readable
**disarmed, with nothing spinning**:

```
thumb over the Bar30 port (pressurise = "deeper")
  measured DEEPER than target   → DEPTH_CMD POSITIVE → ascend   ✓ correct
  measured SHALLOWER than target → DEPTH_CMD NEGATIVE → descend ✓ correct
  demand moves AWAY from target  → STILL INVERTED — DO NOT DIVE
```

They even record why the first version of `preview()` was useless (a full PID fed
a constant error winds its integrator to the rail and reports "inverted" for a
correct loop) — P-only is deliberate.

**This is a pre-water gate we can close on the bench today, and it is the highest-
consequence unvalidated sign in the stack.** It belongs in `bringup_check`.

---

## 5b. Heading drift, MEASURED on this board — and the sign is not stable

Two still-bench runs against `/duburi/state` (no serial contention — the manager
owns the port and we read what it publishes), 22 Hz:

| run | duration | within-window drift | residual | p2p |
|---|---|---|---|---|
| A | 98 s | **−0.386 °/min** | 0.28° rms | 2.05° |
| B | 480 s | **+0.36 °/min** (quarters +0.38 +0.45 +0.30 +0.35, spread **0.151**) | 1.09° rms | 9.16° |

**Magnitude ≈ 0.4 °/min — better than the firmware's own stated 0.5–3 °/min**, and
at the good end of the BNO08X datasheet figure.

### ⚠ The two runs differ in SIGN — but that is NOT yet established as real

−0.386 then +0.36. The tempting reading is session-varying gyro bias. **We are not
entitled to that claim yet, and an earlier version of this section made it.**

The problem is the estimator, not the data. A straight-line fit to a series whose
dominant component is **not** a line returns a slope, always, and the wander here
is 2–9° peak-to-peak against a total drift of a few degrees. Two such estimates
differing does not demonstrate that the underlying bias changed sign.

The reasoning first written here — *"halves measure wander and quarters measure
bias"* — is **retracted as backwards.** A shorter window does not separate bias
from wander; it has less lever arm, so a wander-induced slope contributes *more*
variance per window, not less. Four quarters agreeing to 0.151 °/min, with no
error bar on any of them, is unremarkable rather than confirmatory.

**The discriminating test** is consecutive captures in one session with the board
untouched: if the per-run slope changes sign across captures minutes apart, the
instability is real; if it does not, run A vs run B was estimator noise.

**What is robust regardless of how that resolves, and is what mission design
should use:**

- **magnitude ≈ 0.4 °/min**, better than the firmware's own stated 0.5–3;
- **it cannot be corrected from a stored per-board constant** — the reference is
  captured once at boot and never revisited (§5), so whatever the bias is on a
  given run, nothing on the vehicle is measuring or removing it;
- **≈ 5–6 ° of accumulated heading error over a 15-minute run**, plus wander.

Sign stability: **OPEN.**

### ⛔ 9.16° peak-to-peak of wander on a STATIONARY board

Recorded separately because it is arguably the more actionable of the two numbers
and is easy to lose behind the drift figure. Over 8 minutes, motionless on a
bench, heading wandered **9.16° peak-to-peak** (2.05° over 98 s in the shorter
run). That is **larger than most alignment tolerances in our stack, and it is
present before the vehicle has moved** — it competes directly with the mixer's
±45° attractor (§3b) as an explanation for why terminal alignment is hard.

Not yet separated: sensor vs room. Two captures an hour apart, or one with the
board on foam, would distinguish thermal from mechanical.

### Method note, because the first attempt at run B was garbage

An earlier 480 s capture reported **+38.4 °/min** — and was contaminated: 274° of
total yaw change, +63 °/min in quarter 2 and ≈0 in the other three. **The hull was
moved.** This is exactly the round-26 trap (a "+51.9 °/min drift" that was the
board being handled), and what caught it was the **split-half / quarters check**,
not judgement. A single slope over a single window would have been reported.

**And the quarters are the better estimator, not the halves.** In run B the
quarters agree to 0.151 °/min while the halves disagree (+0.738 vs +0.084). That
is not a contradiction: a fit over a longer window is far more sensitive to a
slow *level* change between sub-windows than to the within-window slope, so the
half-fits are measuring wander and the quarter-fits are measuring bias. Our own
script's verdict line called run B "INCONSISTENT" on the half test and was
**wrong** — the correct reading is "consistent bias, plus wander". Recorded so the
heuristic is not trusted over the numbers next time.

---

## 7. Repo ownership — they have written the rule down, and it matches our practice

`srot-esc-flasher/AGENTS.md` (added `c30b843`, which our mirror was **stale on**
until this round — the flasher was the one repo behind, exactly as suspected):

> **We own, and commit directly to:** `srot-control-board`, `srot-ground-station`
> (the LoRa bridge **and Bondor**), `srot-esc-flasher`.
>
> **We NEVER commit to `duburi_ws`.** Not to `main`, not to `srot`, not "just a
> doc fix". … **That includes the mirrored constants in `fc/srot_protocol.py`
> even though we are the source of truth for the values — being the authority on
> a number is not the same as having write access to their tree.**
>
> **And the reverse.** If `duburi_ws` needs something changed in the firmware, in
> Bondor or in the ESC flasher, they open a **pull request here**. They should not
> push directly.

Their stated reason is the right one and worth repeating: *"a PR makes a
cross-repo change reviewable by the side that owns the consequences"*, and they
name the defects that were only catchable that way — the `MOVE_STOP` brake
double-applying, `SERVOn_FUNCTION` landing on both sides at once, a mirrored
constant drifting.

**This is exactly our standing practice** (we hold `push: true` on their repos and
have never used it), so nothing changes — but it is now a written contract on both
sides rather than a convention we happened to share, and it is worth knowing they
consider a direct push *destructive of the review step*, not merely impolite.

One nuance we should hold ourselves to: their rule says *pull request*, and we
have filed some findings as **issues** (#13 here, `srot-ground-station#3`). That
is deliberate and, we think, within the spirit — an issue is the right shape for a
**design proposal or a report**, a PR for a **change**. Where we have a concrete
patch we send a PR (#10 carries one); where the architectural decision is theirs
we send an issue with a sketch and say so (#11, #13).

### Mirror-currency check, run every srot round (item 9)

| repo | local | origin/main | behind |
|---|---|---|---|
| `srot-control-board` | `f1d3ba9` | `f1d3ba9` | 0 |
| `srot-ground-station` | `1adc14c` | `1adc14c` | 0 |
| `srot-esc-flasher` | `c30b843` | `c30b843` | 0 (was **1 behind**) |

---

## 8. The ESC chain — and how to decide whether our 958/958 zeros are expected

From `srot-esc-flasher/docs/ESC_FLASHING.md` (READ), which our mirror was stale on.

**Stock BLHeli_S does not support bidirectional DShot at all.** The ESCs must be
flashed with **Bluejay** (BB21, v0.21.0 `_H_` builds), and then:

- there is **no "bidirectional DShot" setting in Bluejay** — it negotiates the
  mode from the *inverted* DShot signal the Pico sends;
- **3D mode is mandatory**, because `levelToDshotRaw()` emits 3D bands;
- DShot300, signal → Pico GP6…GP13, common ground.

### ⛔ The test that decides our open question, and it needs no motors

> *"You do **not** need motors attached. Bidirectional DShot telemetry comes from
> the ESC's own MCU in response to every frame, so a powered ESC with just signal
> + ground answers and shows **detected** with **RPM 0**."*

So **"ESC present, RPM 0" and "no ESC" are distinguishable**, and the Pico's own
USB serial @115200 says which stage failed, per motor:

```
link=1 armed=0 bidir=1 rpm_mode=0 loop=1 | M1[rpm=0 pres=1 e=812 d=0 c=0 n=3] ...
```

| field | meaning if wrong |
|---|---|
| `bidir=0` | Pico is in plain DShot — **no telemetry exists** |
| `e=` | eRPM frames decoded; non-zero ⇒ the whole path works |
| `d=` | Extended DShot Telemetry frames — ESC alive, not reporting RPM this frame |
| `c=` | replies arriving but failing checksum ⇒ signal integrity / grounding |
| only `n=` rising | nothing coming back ⇒ wiring, common ground, or ESC unpowered |

**This closes a question we could not close from our side.** We have 958 CRC-valid
`ESC_STATUS` frames with every rpm exactly 0 and no way to tell "no ESCs attached"
(our belief — they are on a new hull) from "attached and broken". It is now a
decidable bench test, and it is a prerequisite for both PR #10 (presence on the
wire) and issue #13 (the hysteretic floor) being anything other than inert here.

### ⚠ One instruction in that guide is harmful — filed as `srot-esc-flasher#2`

Step 4 tells the operator to set **`RPM_LOOP = 1`** and states that it "already
defaults to 1". The firmware defaults it to **0** and argues at length for that:
closing a shaft-speed loop inside the attitude path made the vehicle oscillate
(*"a 1 degree disturbance produced spin-up / stop / spin-up cycling… that is not a
tuning problem, it is the architecture"*), and `PARAM_DEFAULTS_VER` was bumped to
4 specifically to remove it. `RPM_MAX` (doc 4000, firmware 3600) and `RPM_FF_A`
(doc 0.00025, firmware 0.000278) are stale in the same bullet.

**Our board reads `RPM_LOOP = 0` live**, so nothing is wrong today — but anyone
setting up the new hull's ESCs from that document would set it.
