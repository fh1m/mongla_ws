# SROT board integration — how the host drives Hengla

> **Warning: PRE-DIVE: read [`BUGS.md`](../BUGS.md) FIRST.** The axis
> configuration is currently **UNKNOWN** — `CAL_MDIR` is not `[+1]×8` as §8.2 assumed, and it
> multiplies with `MOT_n_DIRECTION`. §8.2's "config is correct" conclusion is superseded.
>
> **Firmware baseline (at the time of the original integration — SUPERSEDED,
> not current): `srot-control-board` @ `3bd247b`, `SROT_FW_BEHAVIOUR_REV 9`.**
> **FLASHED to the vehicle 2026-08-07**, all 232 params re-read afterwards with **zero
> drift** and `CAL_*` intact. The board now reports **rev 14** (root `CLAUDE.md` §2); the
> required floor is `FW_BEHAVIOUR_REV_REQUIRED` in
> `src/mongla_control/mongla_control/fc/srot_protocol.py` — read it there, not the number
> above, which is a historical baseline only.
>
> **Caution: "Rev 7 is additive — nothing that worked on rev 6 changes" is true of the FIRMWARE
> DEFAULTS and false of THIS HULL** (corrected on merge of PR #5, 2026-08-07). Five other
> commits shipped inside the single 6→7 bump, and two of them change how this vehicle
> moves: **`FRAME_REVERSE`** (new param, default 0, **set to 1 here**) negates all six axis
> demands, and **`MOT_1`/`MOT_8_DIRECTION` now default to −1**. Do not arm on the strength
> of "additive" — read
> [the motor-direction section](#rev-7-frame_reverse-changes-what-mot_n_direction-should-be)
> first. The rev number itself is honest (a stock rev-7 board really is a rev-6 board plus
> one statustext); what it does not carry is that a param on our board is now load-bearing.
>
> **What rev 7 adds:** `PREFLIGHT_STORAGE` now reports success as well as failure. The
> command has always been ACKed `ACCEPTED` the instant it is parsed, but the NVS write is
> **deferred to the Core-0 `update()`** — so the ACK has never meant "written", only
> "request received". Until rev 7 the *only* signal was a `CRITICAL` statustext on failure,
> with **silence on success**, which is indistinguishable from a dropped request or a wedged
> board. Rev 7 emits `"Params saved to flash"` (`MAV_SEVERITY_INFO`) on the success path.
>
> If you automate a save, wait for that statustext — not the ACK. Confirmed on hardware:
> ```
> COMMAND_ACK result = 0 (ACCEPTED)
> STATUSTEXT[6]: Calibration saved + verified on flash
> STATUSTEXT[6]: Params saved to flash        <- new in rev 7
> ```
>
> Rev 6 baseline was `ae1bc2f`, flashed 2026-08-06. MOTOR_DETECT still converges in one
> pass — see the motor-direction section below; nobody should hand-set `MOT_n_DIRECTION` again.
> Read [`srot-architecture.md`](srot-architecture.md) first if you have not.
>
> **YAW IS ABSOLUTE FROM REV 4.** `ATTITUDE.yaw` and `VFR_HUD.heading` are a magnetic compass
> heading, not a value relative to wherever the board booted. A heading recorded against rev <= 3,
> or compared across a vehicle reset, is **not the same number**. Absolute `MOVE_TURN` (p4=1) now
> actually turns to the heading it is given — it needs `MAG_YAW_REF=1`, which is the new default.
>
> **DEPTH AND TEMPERATURE CAN NOW BE ABSENT.** `NAMED_VALUE_FLOAT("WTEMP")` and
> `SCALED_PRESSURE2` are **suppressed** when the barometer is unhealthy or stale, and
> `SCALED_IMU2.temperature` sends MAVLink's `0` "not provided" sentinel. Treat absence as "no
> data", never as zero. This is deliberate: before rev 3 a Bar30 whose calibration PROM was read
> during a reset race published fabricated pressure, depth AND temperature — `-51 C` and `+2.87 m`
> in air were both observed — and nothing marked them as wrong. A board whose PROM fails CRC now
> also **refuses `DEPTH_HOLD` / `AUTO` / `PATTERN`** rather than flying on invented depth.

> How `mongla_ws` drives the **SROT** board (firmware "Hengla": ESP32 flight core + RP2350
> Pico RPM co-processor). The board owns the primitives; the Pi sends intent.
> Board-side source of truth: `Mongla_others/srot-control-board/{DUBURI_WS_INTEGRATION,
> JETSON_COMMS,ALGORITHMS,AUDIT,PARAMETERS}.md`.

## Bench history, 2026-08 — what the board taught us

The vehicle is **one USB-C cable** from the Pi to the board. In August the board was reached
through a serial→UDP bridge on a second computer while the hull was rewired; the bridge's own
quirks (a stale serial fd after every flash, ~8–9 % framing loss, port contention with Bondor)
belong to that rig and went with it. What stays here is what the rig taught us about the
**board** — `FRAME_REVERSE`, the double-inversion hazard, `FS_GCS_COMPID`, the Bar30 fix and
payload identity all still apply.

**Warning: Telemetry is not evidence of an uplink.** Probe a command path before trusting a
session — a link can stream perfectly in one direction and carry nothing in the other:

```python
import os; os.environ['MAVLINK20'] = '1'      # BEFORE importing pymavlink
from pymavlink import mavutil
m  = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
hb = m.wait_heartbeat(timeout=15)
m.mav.param_request_read_send(m.target_system, hb.get_srcComponent() or 1,
                              b'FRAME_REVERSE', -1)
print(m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5))   # None => uplink dead
```

### IN-WATER 2026-08-06: every axis reversed + flips in STABILIZE

> **Warning: This is a record, not a procedure to follow today.** It happened on the old
> eight-thruster hull (Dubomini), on firmware rev 6. Rev 7 added `FRAME_REVERSE`, which fixes
> the same symptom at the axis layer and is set to 1 on our hull. Setting all eight
> `MOT_n_DIRECTION = -1` on top of it **cancels** that fix and inverts every axis again.
> Read the parameters with `connect --watch` before changing any of them, and see
> [srot-control-board#34](https://github.com/RakibulIslam1/srot-control-board/issues/34)
> for why MOTOR_DETECT and `FRAME_REVERSE` cannot both be right.

Symptoms: in MANUAL on the joystick, forward/back, yaw and depth all drove the **opposite**
way; in STABILIZE the vehicle **flipped** after arming. Thruster directions had been
verified against the documentation, and level calibration had been done.

**One cause explains all five symptoms: the effective motor direction is globally
inverted.** Every axis reverses because every thruster pushes the wrong way, and the
attitude loop's corrective torque becomes *anti*-corrective — positive feedback — which is
a flip, not a drift. That is why STABILIZE flips while MANUAL "merely" reverses: MANUAL is
**direct passthrough with no attitude feedback at all** (`task_control_loop.cpp`:
`case FlightMode::MANUAL: roll = in.sp_roll; ...`), so the manual reversal *cannot* be an
IMU/orientation problem and there is no point calibrating level again.

**Why "we verified the thruster directions" did not catch it.** `DO_MOTOR_TEST` writes a
**direct per-motor override** (`thrusters.test_override` / `test_motor` / `test_throttle`)
that bypasses the **mixer matrix**. It does *not* bypass the direction flag —
`task_control_loop.cpp:789` is
`dshot[test_motor] = mixer::oneToDshot(test_throttle, in.dir[test_motor])`, so motor test
**does** apply `dir[]`. (An earlier version of this note said it "bypasses `mix()`
entirely" and proves nothing about sign; that was wrong in a way that matters, because it
talks you out of the single most useful test you have — see the vertical-group check below.)

What motor test genuinely cannot see is the **matrix**: which axes a thruster contributes
to and with what sign. So it validates each thruster in isolation and says nothing about
whether the eight agree with each other.

**The mixer matrix is NOT at fault, and the board says so itself.** It publishes a disarmed
self-check: `mix(throttle=+1)` and then counts how many verticals got the correct negative
sign. Read live on this vehicle:

| | read | expected | meaning |
|---|---|---|---|
| `MIX_VERT` | **−1.00** | −1 | thruster 5 correct for full ASCEND |
| `MIX_VSGN` | **4** | 4 | all four verticals correct |

The firmware's own comment: *"Anything less is a mixer matrix error, not a wiring one."*
It is 4 — so **not** a matrix error, which leaves the wiring/direction side.

**The fix is a PARAMETER, not a reflash.** Effective direction is
`CAL_MDIR<n>` (what MOTOR_DETECT found) **×** `MOT_<n>_DIRECTION` (the operator toggle),
combined in `task_control_loop.cpp`. As found on the vehicle:

```
MOT_n_DIRECTION = +1 for all 8
CAL_MDIRn       = [-1, +1, +1, +1, +1, +1, +1, -1]
```

**Caution:** That `CAL_MDIR` pattern is itself suspect — MOTOR_DETECT run on a vehicle whose
thrust is globally inverted will mis-detect. Do not trust it as a baseline.

**Procedure — OUT OF THE WATER, props clear, one step at a time:**

1. Set **all eight** `MOT_n_DIRECTION = -1` in Bondor (Setup → Motors → Thruster direction).
   This flips the global sense without touching firmware.
2. Re-verify in **MANUAL only**, at low `JS_GAIN`, one axis at a time. MANUAL has no
   attitude feedback, so it cannot flip — it is the safe mode to establish sign in.
3. Only once every MANUAL axis is correct, re-run **MOTOR_DETECT** so `CAL_MDIR` is
   re-derived against correct thrust, then Save.
4. **STABILIZE last**, and briefly — it is the mode that flips if any sign is still wrong.

Do not skip to STABILIZE to "see if it is fixed": with an inverted sign it is a
divergent loop, and the vehicle reaches full deflection before an operator can react.

### Right: FLASHED rev 6 — 2026-08-06, and what state the vehicle is in

Board moved to the dev box on USB (`1a86:7523` → `/dev/ttyUSB0`), erased, flashed, restored.
**Flashing needs the board on a USB host**, structurally: an ESP32 enters its bootloader only
when `esptool` toggles **DTR→EN** and **RTS→GPIO0**, and no serial-over-network bridge
conveys line control.

Measured on the way past: **0.0 % `BAD_DATA` on direct serial** (436 msgs/5 s), against
~8–9 % over the August UDP bridge.

| step | result |
|---|---|
| backup before erase | **231/231** params (3 needed index gap-fill — the bridge drops ~8-9 %) |
| erase | chip erase ok, NVS confirmed wiped (`CAL_LVL_R` 0.0, `JS_GAIN_DEFAULT` 0.5) |
| flash | 938 464 B, **hash verified**, boots MANUAL/disarmed, reports **rev 6** |
| restore | **228/228 confirmed**, 0 unconfirmed, saved to flash |
| power cycle ×2 | every spot-checked value persisted |

**Two corrections were made to the backup before restoring it, deliberately:**

- **`FS_GCS_COMPID` 0 → 191.** The backup captured **bench mode, saved to flash**. Restoring
  that verbatim would have put a bench escape hatch into a flight vehicle, where a dead
  companion can no longer be told from a live GCS and the hull station-keeps instead of
  surfacing. **Check this value before every water session** — Bondor's bench mode is
  runtime-only *until someone presses Save*, and someone did.
  **AND IT REGRESSED. Measured 2026-08-07: `FS_GCS_ENABLE=1`, `FS_GCS_COMPID=0` — bench
  mode is back in flash**, one day after being corrected to 191. This is not a one-off; it
  is what this param does when Bondor is used on the bench and someone saves. Prose in a doc
  did not hold it, so it is now a **graded preflight line** (`_gcs_failsafe_verdict`,
  section D) that FAILs — `bringup_check` had been reporting **0 FAIL on a vehicle in
  bench mode**. Fix and persist:
  ```bash
  # Bondor -> Parameters -> FS_GCS_COMPID = 191 -> Save
  # on fw rev >= 7 wait for STATUSTEXT "Params saved to flash", NOT the COMMAND_ACK --
  # the ACK means "request received"; the NVS write is deferred to the Core-0 update()
  ```
  **FIXED + VERIFIED over direct USB 2026-08-07** — written, re-read, `PREFLIGHT_STORAGE`,
  and confirmed by the rev-7 statustext:
  ```
  COMMAND_ACK = 0 (ACCEPTED -- request received, NOT written)
  STATUSTEXT: Calibration saved + verified on flash
  STATUSTEXT: Params saved to flash          <- the one that means the NVS write happened
  ```
  **Caution: Match `"Params saved"`, not just `"saved"`.** The board emits **two** lines and the
  *calibration* one comes first — our first attempt matched it, returned early, and reported
  a params write it had not seen. That recreates rev 7's own silence-means-success bug on the
  client side. `bringup_check` now grades this param (`_gcs_failsafe_verdict`) and
  reads `FS_GCS_COMPID=191 (scoped to us)`.
- **`MOT_n_DIRECTION` → −1 (all eight).** The factory default is +1, which is the state that
  produced the reversed axes in the first place, so restoring defaults would have handed
  back a known-wrong vehicle. All eight at −1 is the globally-uniform state verified in
  water. Effective direction is now `[-1] × 8`.

**Final state:** rev 6 · MANUAL · disarmed · motor dirs uniform · `CAL_LVL_R/P` restored ·
`FS_GCS_COMPID` 191 · `JS_GAIN_DEFAULT` 1.0 · payload roles restored (1-4 + 9-16 SWITCH,
5-8 SERVO) · **`DEPTH_OUT` 0.000 — the wound-up integrator is gone**, cleared by the erase.
`bringup_check` → **0 FAIL**.

**Still owed, and it is the whole point of rev 6: run MOTOR_DETECT in water, armed.** The
current directions are a *global* correction that happens to work; detect will derive each
thruster's true sign and store it in `CAL_MDIR`, which is the readable, per-thruster truth.
On rev 6 it converges in one pass from any starting state, and an inconclusive run now
reports FAIL and changes nothing instead of silently resetting all eight.

> ### BEFORE YOU RUN IT: `MOTOR_DETECT` and `FRAME_REVERSE` are redundant, and stack
>
> **A successful MOTOR_DETECT makes `FRAME_REVERSE = 1` wrong.** They are two different
> fixes for the same symptom, and after detect converges only one of them is still needed.
> Measured against `calibration.cpp` and `task_control_loop.cpp` on `d6f1da5`.
>
> **The detect run itself is safe.** `MOTOR_DETECT` drives thrusters through
> `driveTestMotor()` → the test-override branch's `oneToDshot(test_throttle, in.dir[motor])`.
> That path uses `in.dir[]` but **never reaches `mixer::mix()`**, and `FRAME_REVERSE` is
> applied inside the normal control branch immediately before the mixer
> (`task_control_loop.cpp:863`). So `FRAME_REVERSE` cannot contaminate what detect measures.
> Leave it as-is for the run.
>
> **The problem is what detect converges to.** Its compose form is
> `c' = c · agree` with `agree = c·p·s`, so `c' = p·s` and the effective direction
> `c'·p = s` — every thruster ends up **agreeing with the mixer's expected angular
> response**, i.e. `s = +1` uniformly. That is the point: after a clean detect the mixer's
> intent is what the hull actually does.
>
> But `FRAME_REVERSE = 1` then negates all six demands *after* the controllers, turning that
> corrected frame into a uniform `[-1]×8` flip — **every axis inverted again**, which is the
> original rev-6 complaint.
>
> ```
> after successful MOTOR_DETECT:   s = [+1]×8   mixer intent achieved
>   with FRAME_REVERSE = 0     ->  correct
>   with FRAME_REVERSE = 1     ->  s = [-1]×8   every axis backwards
> ```
>
> **Sequence for the water test:**
> 1. Read and record `MOT_1..8_DIRECTION`, `CAL_MDIR1..8`, `FRAME_REVERSE` **before arming**.
> 2. Run MOTOR_DETECT in water, armed, hull free to rotate. Require **SUCCESS** — a `FAIL`
>    is inconclusive and writes nothing, so a re-run is the correct response, not a param edit.
> 3. **Then set `FRAME_REVERSE = 0`** and save.
> 4. Re-verify MANUAL axes and only then STABILIZE.
>
> If the axes are still inverted with `FRAME_REVERSE = 0` after a SUCCESS detect, the
> inversion is **not** in the motor directions — look at the attitude/gyro convention next,
> and do not simply put `FRAME_REVERSE` back, because that hides which of the two is wrong.

### FOLLOW-UP: a global flip cannot fix a RELATIVE asymmetry

Reversing all eight `MOT_n_DIRECTION` fixed the manual axes, then STABILIZE span the
vehicle about **yaw**, and autotune reported `RollRate ... CLAMP`, `PitchRate FAIL: period
unstable`, `Autotune ABORTED: rate loop untuned`.

**Effective direction is `CAL_MDIR<n>` × `MOT_<n>_DIRECTION`.** Multiplying every element
by −1 changes the global sense but **cannot change which elements differ from each other**.
The asymmetry left by the earlier MOTOR_DETECT run (motors **1** and **8** opposite to
their groups) therefore rode straight through the flip:

```
before flip   eff = [-1,+1,+1,+1,+1,+1,+1,-1]     1 and 8 out of step
after  flip   eff = [+1,-1,-1,-1,-1,-1,-1,+1]     1 and 8 STILL out of step
```

(`CAL_MDIR` read back as all `+1` afterwards because "Calibration saved" folds the detected
sign into `MOT_n_DIRECTION` and normalises — the asymmetry moved, it did not go away.)

**Both symptoms map exactly onto those two motors**, using the mixer matrix:

| motor | group | rows it drives | consequence of being inverted |
|---|---|---|---|
| **1** FR horiz | yaw / fwd / lat | `yaw +1, fwd −1, lat +1` | yaw couple broken → **STABILIZE spins in yaw**; forward injects yaw torque |
| **8** RL vert | roll / pitch / thr | `roll −1, pitch +1, thr −1` | roll & pitch authority ≈ **halved**; heave injects roll+pitch torque |

That is precisely the autotune output. Halved roll authority makes the tune ask for a gain
the envelope refuses → `CLAMP`. Halved pitch authority *plus* heave cross-coupling makes
the limit-cycle period wander past `PERIOD_SPREAD_MAX = 3.0`
(`autotune.cpp`: `s_half_max > s_half_min * 3.0` → `"period unstable"`). **The autotune
failure is a symptom, not a separate fault — do not chase gains.**

**Fix applied 2026-08-06** (live, disarmed, confirmed by readback): `MOT_1_DIRECTION = -1`
and `MOT_8_DIRECTION = -1`, giving `eff = [-1] * 8` — uniform.

**The decisive physical check, and it needs no reasoning about mount geometry:** the
throttle column is `−1` for **all four** verticals, so under motor test at the same
positive throttle, thrusters **5, 6, 7 and 8 must all push the same way**. If one opposes
the other three, its direction flag is wrong. Do this before arming.

Then, in order: **MANUAL first** (no attitude feedback, so it cannot flip) — pure forward
must not yaw, pure throttle must not roll or pitch. **STABILIZE only after that.**
**Autotune last.**

**Caution: Do not re-run MOTOR_DETECT to "fix" this** — it is what introduced the asymmetry, by
detecting against thrust that was already globally inverted. And these two writes are
runtime-only: **Save on the Parameters tab only after in-water verification.**

### Rev 7: FRAME_REVERSE changes what MOT_n_DIRECTION should be

**Everything above this heading describes the rev ≤ 6 world and is now superseded on the
motor-direction question.** Read this before acting on the `[-1] × 8` advice above.

Rev 7 ships `FRAME_REVERSE` (`83ef62e`), and it solves the "every axis is backwards"
problem at the layer where it belongs. `task_control_loop.cpp` negates all six demands —
roll, pitch, yaw, throttle, forward, lateral — **after the controllers and before the
mixer**:

```cpp
if (g_params.frame_reverse > 0.5f) {
    roll = -roll; pitch = -pitch; yaw = -yaw;
    thr  = -thr;  fwd   = -fwd;   lat = -lat;
}
mixer::mix(roll, pitch, yaw, thr, fwd, lat, norm);
```

Placement is the whole point. The attitude PIDs keep operating in the sensor's frame, so
their signs, integrators and the autotune relay stay self-consistent and only the actuator
side flips. Negating at the *setpoints* instead would invert the pilot's stick but not the
stabilisation loops — the half-fix that leaves a hull flyable in MANUAL and divergent in
STABILIZE, which is exactly where this vehicle sat on 2026-08-06.

`DEF_FRAME_REVERSE = 0.0f`, so no other frame is affected. **It is set to 1 on this hull.**

**The two params now have genuinely separate jobs, and this is the bit to internalise:**

| param | job | correct value here |
|---|---|---|
| `FRAME_REVERSE` | whole-frame axis flip, applied once to the demands | `1` |
| `MOT_n_DIRECTION` | compensate an **individual** thruster wired backwards | `-1` on **M1 and M8 only**, `+1` on M2–M7 |

`502eb23` folds that second row into the firmware defaults, so a freshly-erased rev-7 board
comes up as-flown. Note this makes the **factory default correct** for the first time — the
"factory default is +1, which is the known-wrong state" reasoning above no longer applies.

#### Right: RESOLVED 2026-08-07 — the board is in the intended configuration

Read over the bridge, disarmed, MANUAL:

```
FRAME_REVERSE   = 1
MOT_n_DIRECTION = [-1, +1, +1, +1, +1, +1, +1, -1]     M1, M8 only -- as-flown
```

That is the rev-7 intent exactly: whole-frame flip on, per-thruster wiring compensated,
net sign `s = -(w · dir) = [-1] × 8` uniform. **The `[-1] × 8` we restored on 2026-08-06 is
NOT what is on the board** — someone corrected it (consistent with `502eb23`, which reports
reading M1/M8-only off a board on 08-07). Also confirmed: `CAL_LVL_R` 0.0396 / `CAL_LVL_P`
0.0131 intact, `MAG_YAW_REF` 1, `JS_GAIN_DEFAULT` 1.0, fw behaviour rev **7**.

**The hazard below did not occur. Keep the section** — it is the reason the check exists,
and the configuration can regress the moment anyone hand-edits motor directions again.
**Caution:** `FRAME_REVERSE` is still **unverified under thrust**: MANUAL axes first, STABILIZE second,
autotune last.

> **Warning: This pairing is correct only until MOTOR_DETECT succeeds.** `CAL_MDIR` is all `+1`,
> i.e. detect has **never** written to this board — so the first successful run is ahead of
> us, not behind. Detect converges every thruster to *agree* with its mixer column, at which
> point `FRAME_REVERSE = 1` inverts the corrected frame and every axis is backwards again.
> **After a SUCCESS detect, `FRAME_REVERSE` must go to 0 in the same session.** Full
> derivation and sequence: [the box in the MOTOR_DETECT section](#-before-you-run-it-motor_detect-and-frame_reverse-are-redundant-and-stack)
> (firmware team, PR #6).

#### The double-inversion hazard — check this before you arm

**`MOT_n_DIRECTION = [-1] × 8`, which the 2026-08-06 restore above deliberately wrote and
which this document told you to verify, is no longer the intended configuration** — and it
does not merely cancel the axis fix, it **reproduces the exact rev-6 fault**. Derived from
`mixer.cpp`, not asserted:

Write `w[m] = -1` for a thruster wired backwards (M1 and M8 here, `+1` elsewhere). Physical
thrust is `w[m] · MOT_m_DIRECTION · (M[m] · demand)`, and `FRAME_REVERSE = 1` negates
`demand`. So the net sign each motor contributes, relative to the mixer's intent, is
`s[m] = -(w[m] · dir[m])`:

```
w              = [-1, +1, +1, +1, +1, +1, +1, -1]    M1, M8 wired backwards
dir = correct  = [-1, +1, +1, +1, +1, +1, +1, -1]  -> s = [-1]*8   uniform: the frame flip, intact
dir = [-1]*8   = [-1, -1, -1, -1, -1, -1, -1, -1]  -> s = [-1, +1, +1, +1, +1, +1, +1, -1]
                                                          ^^ M1 and M8 out of step with M2-M7
```

That second row is the 2026-08-06 fault verbatim. Push it through the yaw column
`M[1..4][yaw] = [+1, -1, -1, +1]`:

```
intended (s uniform)   -[+1,-1,-1,+1] = [-1,+1,+1,-1]    a real yaw couple
actual   (s mixed)      [-1,-1,-1,+1]                    three thrusters pushing the same way
```

Which is 83ef62e's own sentence — *"three thrusters pushing the same way is not a torque at
all"* — and is why the vehicle span about yaw in STABILIZE. The vertical group breaks the
same way on roll (`M[5..8][roll] = [+1,-1,+1,-1]` → `[+1,-1,+1,+1]`), which is the halved
roll/pitch authority behind `CLAMP` / `period unstable`.

> ### Right: SETTLED 2026-08-07 — the board is in the GOOD state
>
> Read off the vehicle over COM8, **twice**: once as a pre-flash backup and again after
> flashing `d6f1da5`, with a full 232-param diff between them showing **zero drift**.
>
> ```
> MOT_1..8_DIRECTION : [-1, +1, +1, +1, +1, +1, +1, -1]
> CAL_MDIR1..8       : [ 1,  1,  1,  1,  1,  1,  1,  1]
> FRAME_REVERSE      : 1
> ```
>
> That is the **first** row of the derivation above — `dir = correct` → `s = [-1]×8`, the
> clean uniform frame flip, intact. **The feared `[-1]×8` double-inversion is NOT present**,
> so the residual-on-M2–M7 failure mode does not apply to this hull right now.
>
> Note `CAL_MDIR` is all `+1`, i.e. MOTOR_DETECT has never successfully written to this
> board — effective direction is `MOT_n_DIRECTION × CAL_MDIR = MOT_n_DIRECTION`. That is
> also why the box in the MOTOR_DETECT section above matters: the first successful detect
> will change this, and `FRAME_REVERSE` has to come off in the same session.
>
> The reasoning below is kept because it is correct and is the thing to re-run if the
> directions are ever touched again; only the "we could not check" is now resolved.

**~~We do not know which state the board is in, and we could not check~~** — the vehicle was
unreachable when this was written. The evidence points both ways and neither side is ours:

- We restored `[-1] × 8` on 2026-08-06 and it was saved to flash. **NVS beats defaults**, so
  `502eb23` changing the defaults does *not* correct a board already in service, and the
  rev-7 flash notes report "232 params re-read, zero drift" — i.e. whatever was stored
  stayed stored.
- But `502eb23`'s own message says all 232 params were read off a board over COM8 and only
  M1/M8 were negative. If that was this vehicle, someone had already corrected it.

**One param read settles it. Do it before arming, not after:**

```bash
ros2 run mongla_manager connect          # or Bondor -> Setup -> Motors
# expect: FRAME_REVERSE = 1
#         MOT_1_DIRECTION = -1, MOT_8_DIRECTION = -1, MOT_2..7_DIRECTION = +1
# if all eight read -1, the axis fix is CANCELLED and M2-M7 are inverted -- fix before arming
```

Then the physical check from the previous section still stands and still needs no reasoning
about mount geometry: the throttle column is `−1` for **all four** verticals, so under motor
test at the same positive throttle, **5, 6, 7 and 8 must all push the same way**.
`DO_MOTOR_TEST` bypasses the mixer *matrix* but **does** apply `in.dir[]`
(`task_control_loop.cpp:789`), so it is a real test of the direction flags — it just cannot
see `FRAME_REVERSE`, which is applied inside the mixed path only.

**Caution: `FRAME_REVERSE` is unverified under thrust** — the firmware team flagged this in PR #5
and props were off. MANUAL axes first (no attitude feedback, so it cannot flip), STABILIZE
only after that, autotune last.

### Right: Full-stack verification over direct USB — 2026-08-07

Board on this dev box, `/dev/ttyUSB0`, by-id `usb-1a86_USB_Serial-if00-port0` (CH340
`1a86:7523`). Everything below measured, not inferred.

| check | result |
|---|---|
| `bringup_check` | **0 FAIL, exit 0** — every SROT line PASS |
| fw behaviour rev | **7** (via `MAV_CMD_REQUEST_MESSAGE(148)`) |
| `FS_GCS_COMPID` | **191**, written and confirmed in flash (above) |
| barometer spread | **2.94 mbar** on USB vs **5.87** over the August bridge — visibly cleaner |
| Bar30 health | healthy; `AUTO`/`DEPTH_HOLD` available |
| manager startup | connects, banner correct, `/mongla/state` populating |
| payload roles | FIREABLE (switch) `[1,2,3,4, 9..16]` · arm/PWM `[5,6,7,8]` · **unreadable none** |
| telemetry | BAT 13.3–13.6 V · WTEMP 27.9 °C · `MAGACC 2` · LEAK dry · KILL clear · RPM all 0 |

**`mongla_ws` is compatible with the current firmware and needs no code change to fly it.**

**Caution:** Two things this does **not** prove, both still open:
- **The depth loop.** `DEPTH_OUT +0.00` disarmed is a *stale register*, not a settled loop
  (that is the §8.1 ask in the firmware PR). The two armed checks still gate every AUTO move.
- **`MAGACC 2` does not close the mag gate.** The firmware needs `>= 2`
  (`yaw_ref.cpp:185`, `need_acc = have_our_cal ? 0 : 2`) so the *accuracy* gate is satisfied —
  but nothing publishes `yaw_ref::State`, so **we cannot tell `LOCKED` from `REFUSED_CAL`**.
  If it never locked, `ATTITUDE.yaw` is relative to the BNO's boot orientation and absolute
  `MOVE_TURN` turns to a meaningless number. Accuracy is not a proxy: the firmware's own
  comment says the alignment is protected by the `|B|` band and the sample-agreement test,
  neither of which depends on the sensor's self-assessment. **Prefer relative turns until
  the state is observable** (§8.8 in the firmware PR).

### Bondor and the companion-lost failsafe — do not arm from Bondor alone

`FS_GCS_SYSID`/`FS_GCS_COMPID` (255/191) scope the companion-lost failsafe to **one named
source — mongla_ws**. Bondor is **255/190**, so its heartbeat never feeds that timer. The
board latches "companion seen" once the named source has appeared, so a Bondor-only session
on a *freshly booted* board is fine — but once mongla_ws has connected **in that power
cycle**, arming with only Bondor trips the failsafe ~5 s later: out of MANUAL into
**SURFACE**, and the four verticals spin up. Measured 2026-08-06; it reads as "arming ran the
motors on its own" and is neither a Bondor nor a firmware bug.

Config, tuning, calibration, param save/export and payload roles need **no arming** — do them
with Bondor alone. The one exception is motor direction (`DO_MOTOR_TEST` is gated on armed);
Bondor's **BENCH MODE** covers it by setting `FS_GCS_COMPID = 0` with a `PARAM_SET` only,
never `PREFLIGHT_STORAGE`, so a power cycle restores the flight value. **Never press Save
while bench mode is active**, and never take the wildcard into the water: a dead companion
could no longer be told from a live GCS, and the vehicle would station-keep instead of
surfacing. `bringup_check` grades `FS_GCS_COMPID` and FAILs on the wildcard.

### Payload identity comes from the board (`SERVOn_FUNCTION`)

Two orthogonal per-channel params, and keeping them apart is the safety property:

| | what it is | who enforces it |
|---|---|---|
| `SERVOn_ROLE` | **authority** — 0 off / 1 PWM (arm) / 2 switch (payload) | the board; `fire()` refuses anything that is not 2 |
| `SERVOn_FUNCTION` | **identity** — torpedo / dropper / gripper / light / camera / aux | nobody. It is a label. |

Setting a FUNCTION never makes a channel fireable. The firmware deliberately does not read
`servo_func`; it is NVS storage so the payload map **travels with the hull** instead of
living in a launch file that goes stale on a re-wire — and the failure mode of a stale
payload map is firing the manipulator arm during a drop.

Set it in **Bondor → Payload → Function**, then **Save** on the Parameters tab (it is
NVS-backed and does not persist without the save). mongla_ws reads all 16 roles *and*
functions in one traversal at bring-up and prints them:

```
[PAYLOAD] board roles: FIREABLE (switch) [9..16] | arm/PWM [1..8] | unreadable none
[PAYLOAD]   ch 9: SWITCH -- fire(9) will actuate (torpedo)
```

`payload_channels` survives only for per-instance names the board's fixed enum cannot
express (`torpedo_1` vs `torpedo_2`, which share one FUNCTION). When both are set and
disagree, the override is used **and a warning names both** — that warning is the
stale-copy detector.

Canonical numbers: `SROT_SERVO_FUNC_*` in the firmware's `include/config.h`, mirrored in
`fc/srot_protocol.py` and Bondor's `shared/protocol.ts`. **Append-only** — inserting a
value silently renames every payload after it. `test_srot_protocol_drift` referees all
three copies, including Bondor's, because Bondor's only gate is `npm run typecheck` and
that cannot see the firmware.

### Right: Bar30 fixed — and why that is NOT "depth verified"

The connector fault is **resolved**, measured on the vehicle 2026-08-03 against the
2026-08-02 failure:

| | 2026-08-02 (faulty) | 2026-08-03 (fixed) |
|---|---|---|
| `press_abs` | 317–874 mbar | 978.8–987.7, **sd 1.97** |
| water temp | 6–30 °C | 31.68–31.71 (**0.03** spread) |
| depth in air | +0.9 … +6.8 m | −0.05 … +0.07 m |
| `DEPTH_OUT` | pinned **−1.00** | **0.000** |

`bringup_check` now grades `barometer 983.7 mbar, spread 5.92` and `depth loop
settled` as PASS. The arming hazard from the phantom-depth saturation is gone.

**That is the barometer, not the loop.** On rev 4, `DEPTH_CMD` read `-0.329202` while
`DEPTH_OUT` and `DEPTH_ERR` read **exactly 0.000 across 90+ samples** — a live controller
with a −0.33 m command against a +0.01 m measurement cannot produce zero error, so the loop
was not running while disarmed. `check_depth_loop_settled` passing proves only
`|DEPTH_OUT| < 0.90`, which a stopped loop satisfies trivially.

**Re-measured on rev 5 (2026-08-06) the numbers are different, and the reading changed with
them.** Disarmed, 74 samples over 20 s: `DEPTH_OUT` **−0.115** and `DEPTH_ERR` **−0.029**,
both with **zero variance**, while `DEPTH_CMD` moved (−0.479..−0.297). So the loop is no
longer publishing a flat zero — it evaluated something and then froze. Frozen non-zero
outputs are *not* evidence of a healthy loop either: a live controller tracking a moving
command cannot hold a constant error to three decimals. Both readings support the same
conclusion for opposite reasons, which is why the conclusion has not moved:

**The depth loop has still never run closed in `AUTO`** (the 2026-08-06/07 water tests closed
`DEPTH_HOLD` only, and the transition to `AUTO` crashed the board), it gates *every* AUTO move (`move_forward`
included — there is no depth-free path through AUTO), and the two **armed** bench checks
in the runbook below remain the gate. "Bar30 fixed" must never be read as "depth verified".
**Caution:** Note `|−0.115| < 0.90`, so `check_depth_loop_settled` still passes — it is an
anti-saturation guard, never a proof the loop works.


## The offload boundary

**The board owns:** attitude + depth hold, heading hold, thrust allocation (vectored 6-DOF),
timed motion primitives **with on-board braking**, arming + pre-arm checks, leak / low-battery
/ companion-lost failsafes → SURFACE, ESC RPM telemetry.
**The Pi owns:** perception, mission logic, *which* verb next, vision servoing, payload
sequencing, localization, logging.

## The driver (`mongla_control/fc/`)

```
fc/base.py          Telemetry + MoveResult, the types the facade and the node share
fc/srot_protocol.py THE wire constants (cmd 31000, move types, modes, ACKs, MC scaling)
fc/srot_fc.py       SrotFC: MANUAL_CONTROL, SROT_MOVE + ACK relay, telemetry, payload
fc/port_guard.py    one process per serial port
```

`SrotFC` exposes intent: `arm` / `disarm` / `set_mode` / `manual(fwd, lat, up, yaw)` /
`move(verb, *, on_progress, abort_fn)` / `stop_motion` / telemetry / `send_heartbeat`.
`manual()` is the streamed axis demand the vision loops use; `move()` is one on-board
`SROT_MOVE` and its four-terminal ACK relay. There is one backend, so there is no selector.

## The verb table (`fc/srot_moves.py:_build_params` + `srot_fc.MOVE_VERBS`)

**One `SROT_MOVE` (31000) each** — `move()` sends one command and relays the ACK:

| mongla verb | p1 type | params |
|---|---|---|
| `move_forward` / `move_back` | 0 / 1 | p2=duration, p3=speed (gain/100) |
| `move_left` / `move_right` | 2 / 3 strafe | p2=duration, p3=speed |
| `yaw_left` / `yaw_right` | 4 turn (**relative**) | p2=∓degrees, p4=0 |
| `turn` | 4 turn (**absolute**) | p2=heading, p4=1 — **needs `MAG_YAW_REF=1`** |
| `set_depth` | 5 dive | p2=**−target** (mongla negative → SROT positive depth); refuses target>0 |
| `stop` | 6 brake | also the ROS-cancel wire action |
| `pause` | 7 hold | station-keep (there is no channel release on this board) |
| `style_roll` | 8 style | p2=count (roll only, 90°/s) |

**On the host, not through `move()`:** `arm` / `disarm` / `set_mode` (COMPONENT_ARM_DISARM
400 / DO_SET_MODE 176, with the mode **verified** on a heartbeat heard *after* the command,
because a silent refusal looks exactly like success and a cached heartbeat already in that
mode used to count as the answer); `surface` → `Mongla.surface()`: `stop_motion()`, then
`set_mode('SURFACE')`, then wait for the depth to read within `SURFACED_M` of zero. If it
never does, the verb **fails** and says the board is still ascending: SURFACE stays engaged,
but a surface nobody saw is not reported as reached; `fire` → `SrotPayload` (the board's
PCA9685 over `DO_SET_SERVO`), which takes the **board channel directly** (`fire(9)` =
`DO_SET_SERVO param1 = 9` = `SERVO9_ROLE`) and refuses any channel the board does not call a
SWITCH; `head`, `mission_reset`, `calibrate_depth`, `calc_distance`, `motor_test`;
`vision_align` / `vision_move` via `manual()` in STABILIZE.

**Warning: `surface()` is the last motion.** `stop` is itself a `SROT_MOVE`, which enters AUTO, so a
`stop()` after `surface()` takes the hull back out of SURFACE. `test_sauvc_srot_port.py`
refuses a board move after `surface()` in any mission.

**What is not here, and why.** A host heading lock, distance-closed moves (`*_dist`), `arc`
and a STYLE yaw spin belonged to the old backend. The board holds heading itself, there is no DVL
to close a distance on, `MOVE_ARC` takes a yaw *rate* where the old `arc` held an absolute
heading, and the board has no selectable STYLE axis. They were deleted rather than refused; a
mission that still names one fails at the node with `Unknown command`.

## How the board differs from what a host loop expects

- **Message rates are pinnable** (fw rev ≥ 2): `MAV_CMD_SET_MESSAGE_INTERVAL` (511) and
  `GET_MESSAGE_INTERVAL` (510) work; ATTITUDE pinned from 10.8 Hz to **55.2 Hz** on the bench.
  `SROT_MESSAGE_RATES` in the manager sets them. HEARTBEAT cannot be disabled, and an unknown
  msgid is DENIED rather than accepted-and-dropped.
- **`MANUAL_CONTROL` always carries all four axes.** There is no RC override and no per-axis
  release; an axis you are not driving is sent as 0 and the board's centred-stick hold keeps
  it.
- **No depth-setpoint stream** — `SET_POSITION_TARGET_*` is UNSUPPORTED. Depth is a mode
  (`DEPTH_HOLD` latches the current depth) or a DIVE move, so the vision loops never command
  depth: `set_depth` first, then align.
- **The ≥1 Hz MAVLink HEARTBEAT is mandatory** — 5 s of silence surfaces the vehicle — and it
  comes from the manager's heartbeat thread, which nothing in an action callback may block.
- **`JS_GAIN` boots at 0.5**, which halves MANUAL_CONTROL. `SrotFC.set_default_gain()` sets it
  at bring-up; verify against the `GAIN` NAMED_VALUE_FLOAT.
- **Modes are the board's** (STABILIZE 0, ACRO 1, DEPTH_HOLD 2, SURFACE 9, MANUAL 19, AUTO 23,
  STUNT 100, PATTERN 101, plus the tuning modes 20–22). A mode name the board does not have (`ALT_HOLD`, `POSHOLD`) resolves to
  nothing and `set_mode` refuses it.
- **Transport:** one USB-C cable at 115200. `connection_config.resolve_srot_profile()` finds
  the board (CP210x / CH340 / CH9102 by-id, `ttyUSB*`/`ttyACM*` fallback) and refuses to guess
  between two; `-p mav_device:=` overrides with a path or any pymavlink connection string.
  Vehicle sysid/compid 1/1; we are 255/191.

## Bench bring-up runbook (each step gates the next)

Prereqs (operator, via **Bondor** — no mongla_ws code): ESCs on **Bluejay** (`DSHOT_BIDIR=1`,
`RPM_LOOP=1`) or ESC RPM is absent; thruster **directions** verified (motor-detect);
**Bar30 fitted**; a **`.params` export** taken (a `PARAM_DEFAULTS_VER` bump wipes `CAL_*` —
otherwise unrecoverable).

1. **Telemetry (props off):** `ros2 run mongla_manager connect --watch`, then the manager;
   confirm `/mongla/state` populates (yaw/depth/armed/mode) and `/mongla/esc_rpm`. Nothing
   actuates.
2. **arm/disarm (props off):** from the CLI. Confirm the pre-arm STATUSTEXT surfaces on a
   reject.
3. **`manual()` per axis:** each of fwd/lat/up/yaw **individually**, direction checked against
   the firmware's `docs/THRUSTER_MAP.md`. A wrong sign is positive feedback — fix it on the
   board (`FRAME_REVERSE` / `MOT_n_DIRECTION`), never in code.
4. **DEPTH LOOP — MANDATORY, the depth loop has never run closed in `AUTO` (AUDIT R1; see *IN-WATER 2026-08-06* for the one `DEPTH_HOLD` run):**
   - Enter `DEPTH_HOLD`, raise & lower the hull by hand: verticals must push **back toward**
     the latched depth, not away.
   - Force SURFACE (trip the leak input) at depth: the demand must be **ascend**.

   > **Caution: This gates EVERY AUTO move, not just the dive-dependent verbs.** `SROT_MOVE`
   > auto-enters `AUTO`, and the `AUTO` branch calls `depth::setTarget(md.depth_target)` +
   > `depth::update(...)` underneath **every** primitive (fw `task_control_loop.cpp:236-237`)
   > — there is **no depth-free path through AUTO**. A plain `move_forward` with no
   > `set_depth` anywhere still runs the unverified loop, and a vertical runaway mid-leg is
   > indistinguishable from a buoyancy problem in the water.
   >
   > A successful **in-air** `move_forward` is not partial validation: at ~0 m the latched
   > target and the measurement agree, so the loop is never actually exercised.
5. **`move()` verbs:** each runs and reports ~3 Hz progress; ROS-cancel brakes (type 6); a
   preempting second move resolves the first as PREEMPTED, not a hang.
6. **Vision:** `vision_thrust_check` disarmed for every sign, then `vision_align` armed in
   STABILIZE, props clear.

### Firmware behaviour rev 3 (2026-08-02) — absence became a signal

Rev 3 landed with the board **in the vehicle**, and its theme is that the firmware now
**refuses to report data it cannot stand behind**. Three consequences reach us:

1. **The Bar30's calibration PROM is validated (CRC-4) and no longer read in a race.**
   The vendored MS5837 driver's `reset()` compared `micros()` against a `millis()` baseline,
   so the mandatory ~2.8 ms post-RESET reload delay was **zero** and `initConstants()` read
   the PROM while the sensor was still reloading it. The coefficients were boot-time luck.
   That is not a display bug: `dT` feeds `offset` and `sens`, so a bad PROM corrupted
   **pressure and depth** silently, by an amount whose sign you cannot predict. On a vehicle
   whose depth loop had never run closed, that is what the first dive would have flown on.
2. **An unhealthy or stale baro now refuses `DEPTH_HOLD`/`AUTO`/`PATTERN`.** `SROT_MOVE`
   enters `AUTO`, so this means **every move verb is denied** — `move_forward` included.
   `bringup_check` reads it via `_baro_health_verdict` off `SYS_STATUS`, because on
   the deck the symptom is "it arms and then does nothing".
3. **`WTEMP` and `SCALED_PRESSURE2` are suppressed when the baro is unhealthy**, and
   `SCALED_IMU2.temperature` sends MAVLink's `0` "not provided". Absence is the signal.
   **Caution: `VFR_HUD` is NOT gated on `depth_ok`** — and `VFR_HUD.alt` is where we read depth. So
   our depth number keeps arriving on a board that has declared its baro dead. That is safe
   only because the same `depth_ok` flag *also* refuses AUTO (`task_control_loop.cpp:107`
   and `:348` use the identical condition), so no closed-loop move ever runs on it. Read
   health from `SYS_STATUS`, never infer it from the presence of a depth value.

> **Warning: CORRECTED 2026-09-07. `FW_BEHAVIOUR_REV_REQUIRED` IS 10, NOT 2**
> (`srot_protocol.py:733`). The paragraph below was true when written and has
> been false since round 26, which is the dangerous kind of stale: it names a
> **safety floor**, and a reader who trusts it believes the host will fly a
> rev-2 board that it will in fact refuse. (The board itself now reports rev 14 —
> root `CLAUDE.md` §2 — but the *required floor* is still 10; read
> `FW_BEHAVIOUR_REV_REQUIRED` in source, not either number quoted from memory.)
>
> The floor was raised because revs 3-10 are **not** all additive, which is
> exactly what the original reasoning assumed: **rev 10 INVERTS YAW** and rev
> 13 requires ARMED for `SROT_MOVE`. A rev-2 board would take our yaw commands
> with the wrong sign. Raising the floor strands a vehicle that would
> otherwise turn the wrong way, which is the safety gain the paragraph said
> did not exist.
>
> Kept verbatim below, struck through, because the *reasoning* is the thing
> that was wrong and deleting it hides that.

~~**`FW_BEHAVIOUR_REV_REQUIRED` stays at 2, deliberately.** Rev 3's changes are additive for
this host, so a rev-2 board still runs it correctly; raising the floor would strand a
working vehicle for no safety gain.~~

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

### Bench findings, 2026-08-02 — measured on the vehicle, not inferred

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
sample showed the slot alternating between **1.35 V and 14.74 V**. `/mongla/state`'s battery
voltage was therefore whichever arrived last. Same failure as `NAMED_VALUE_FLOAT`, one layer
down; fixed the same way (`SrotFC.note_battery`, fed from the manager's reader thread).
`get_battery()` is now pinned to id 0 and `get_batteries()` returns both.

### Payload: the board owns the channel role, we only read it

Each PCA9685 channel's role is a **firmware parameter** — `SERVO{n}_ROLE`, n = PCA channel + 1
(`0` disabled, `1` PWM servo, `2` MOSFET/switch). **Measured on the vehicle 2026-08-02:
1-8 = SERVO, 9-16 = SWITCH.**

`mongla_ws` drives **switch channels only**. The PWM channels are the on-board manipulator
arm, and firing one from a mission would move the arm mid-drop. `SrotPayload.fire()` therefore
reads the role from the board and refuses anything that is not `2`, **failing closed on an
unreadable role**.

**There is no host-side channel map at all any more (2026-08-03).** `fire(N)` addresses
BOARD channel N — the same N as `DO_SET_SERVO param1` and as `SERVO{N}_ROLE`. The old
`payload_fire_map` routed a "host channel" 1..4 onto a PCA channel; it bought nothing and
cost a second numbering to keep in sync by hand, whose failure mode is driving the arm.
`payload_fire_map` is now a **hard startup error** — silently ignoring it would be worse,
because its old numbers (1..4) are now valid board channels that on the default role layout
ARE the arm. `payload_channels:="9:torpedo_1"` supplies labels for the log and nothing else;
a stale label can mislabel a line but cannot misdirect a shot.

**The "1-8 arm / 9-16 switch" split is a DEFAULT, not a rule.** It is only the initial value
of `SERVO{n}_ROLE` (fw `params.cpp:368-371`: `(c < 8) ? 1.0f : 2.0f`), and every channel is
independently re-rolable from Bondor. Any host table encoding that split is wrong the moment
someone uses the feature — which is why we read all 16 roles at bring-up and print them.

**`fire()` returns a typed outcome**, not a bool: `FIRED` / `REJECTED_ARM_CHANNEL` /
`DISABLED_CHANNEL` / `DENIED` / `NO_ACK` / `BUSY` / `NOT_READY`, carried to a mission in
`Move.Result.final_value`. `FireState` now returns `FAILED` (not an unconditional `SUCCEED`)
when the shot did not go out, and the vision mid-hold fire logs a refusal at ERROR instead of
discarding it.

**Caution: `FIRED` means the board ACCEPTED the command, not that a solenoid moved.** The firmware
streams no actuator readback of any kind and its PCA9685 driver sets its health flag with no
I2C probe, so a physically disconnected expander ACKs exactly like a working one. This is the
strongest claim the wire supports.

**Warning: The board does NOT yet enforce the role contract — we do.** `DO_SET_SERVO` on a role-1
channel writes `servo_us` and returns `ACCEPTED` (fw `mav_commands.cpp:475-481`): it moves the
arm and reports success. Until `MAV_CMD_SROT_PAYLOAD_FIRE` lands (spec:
`TASKS_FROM_DUBURI_WS.md` §7), the host-side role read is the only interlock. Also unfixed
board-side: **no failsafe de-energises a latched payload channel** — leak, disarm and GCS-loss
all leave it energised, so the host's `finally` OFF is the only thing that clears it.

`DO_SET_SERVO` addresses a channel by its own number whatever its role, because the firmware
reads the µs as a LEVEL for a role-2 channel (`mav_commands.cpp:471-479`) — so one code path
covers both, and the role check is the only thing distinguishing them.

**Caution: `preflight_roles()` needs the reader thread running.** `get_param` reads the pymavlink
cache and never calls `recv_match()` itself, so with no reader every role reads `None` and the
payload reports UNREADABLE — indistinguishable from a mis-roled board. The manager starts the
reader before `_preflight_payload`; `test_the_reader_thread_starts_before_the_payload_role_read`
pins that ordering.

### A timed move is NOT voltage-independent (firmware, confirmed 2026-08-03)

`move_forward --duration N --gain G` is **purely timed** — no distance sensor, no estimate — and
at the firmware's shipped defaults **nothing compensates for battery voltage**. All three
mechanisms that would are off:

| Mechanism | Param | Default |
|---|---|---|
| slow per-thruster RPM trim | `THR_TRIM_EN` | **0** (recommended route; not water-validated) |
| mixer battery feedforward | `MOT_BAT_V_MAX` | **0** = off (also needs `ESPNOW_EN=1` + the 2nd board) |
| Pico closed-loop RPM | `RPM_LOOP` | **0** — deliberately, it oscillates in the stabilisation path |

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
| depth / pressure / `WTEMP` | no Bar30 | **Caution: `VFR_HUD.alt` is NOT gated on baro health** (`mav_stream.cpp:258`), unlike `SCALED_PRESSURE2`/`WTEMP` which *are* suppressed. The board streams `-0.000 m` with no barometer fitted at all, so read the `SYS_STATUS` health bit — never infer health from the presence of a depth value |
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
| `esp32doit-devkit-v1` | RAM 24.3%, Flash 29.6% |
| `pico` | (14m42s — RP2350 toolchain fetch) |
| `second-board` | (12s) |
| `groundstation-esp32`, `esp32_4way`, `esp32_4way_diag` | *"Nothing to build"* — **stale env definitions**, their `build_src_filter` points at `src/groundstation/` and `src/esp32_4way/`, which have never existed in that repo. Those sources live in `srot-ground-station/` and `srot-esc-flasher/src/esp32_4way/` |

**Warning: Build only. Do not flash without an explicit decision.** `app0` must stay at `0x10000`
(PlatformIO hardcodes the app offset and does *not* read it from the CSV — moving it makes
every upload land where the bootloader will not look), and the 20 KB → 128 KB NVS change means
`pio run -t erase` first, which **wipes the `CAL_*` block** — accel, mag and level calibration
plus detected motor directions, recoverable only from a Bondor parameter export.

### Host-side workarounds for firmware defects (see `srot-control-board/JETSON_FEEDBACK.md`)

> **Read `srot-architecture.md` first.** Most of this section is now history. The firmware
> answered nine of the eleven `JETSON_FEEDBACK` items in its Round 6 (`AUDIT.md` R35–R44), and
> the workarounds below have been **removed from the code**, not just annotated. The version
> gate is `srot_protocol.FW_BEHAVIOUR_REV_REQUIRED`; the board reports its own via
> `SROT_FW_BEHAVIOUR_REV` in `include/config.h`.

- ~~**`MOVE_STOP` does not brake**~~ — **FIXED in fw REV 2, and our brake is GONE.**
  The firmware now captures the outgoing leg's axis and speed *before* zeroing them and
  restores both into `PH_BRAKE`, so `MOVE_STOP` decelerates on its own.
  `SrotFC._brake_last_leg` **has been deleted**; `stop_motion()` is a bare `MOVE_STOP`.
  **Caution: This is the one change with a hull-relevant failure mode in BOTH directions:** run this
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
- **Depth sign** — `VFR_HUD.alt` already arrives NEGATIVE-below-surface (our convention).
  We were negating it a second time, which made `/mongla/state.depth_m`
  positive when submerged and silently disabled every depth guard in the stack (they all
  compare against a negative constant, so none of them errored — they just stopped firing).
- **pymavlink's DEFAULT dialect is MAVLink *1* (`dialects.v10.ardupilotmega`), where
  `ESC_TELEMETRY_1_TO_4`/`_5_TO_8` (11030/11031) DO NOT EXIST.** Measured in-vehicle: the
  board's ESC frames arrive and are reported as `UNKNOWN_291` / `UNKNOWN_11030` /
  `UNKNOWN_11031` — zero RPM that looks exactly like an ESC or wiring fault. `MAVLINK20=1`
  must be set **before** pymavlink is imported. `fc/srot_fc.py` does it at import, so
  importing the driver on its own (`bringup_check`, a test, a script) never silently loses ESC
  telemetry.
  The drift test now asserts against `mavutil.mavlink.mavlink_map` — **the map the vehicle
  actually decodes with** — not against the `dialects.v20` module, which is a different
  object and was passing while the runtime path decoded nothing.
- **`ESC_STATUS` (291) is in NO pymavlink dialect** (upstream removed 290/291 from `common`),
  and pymavlink drops unknown msgids **silently**. This finding was correct and still stands —
  but the firmware now **also** emits `ESC_TELEMETRY_1_TO_4`/`5_TO_8` (11030/11031), which
  decode fine, so the fallback you already wrote is live. **`/mongla/esc_rpm` is now
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
- **params may not have persisted before firmware `8cb4203`** (fw R14: the NVS partition
  was too small, writes silently failed). Assume `set_default_gain()`'s `JS_GAIN_DEFAULT=1.0`
  never stuck → `MANUAL_CONTROL` at half authority. **Reflash + read back `GAIN`.**

## Status

The live status — what is verified, what is built, what water and the firmware still have
to decide — is [`ROADMAP.md`](../ROADMAP.md), the one status file. The integration history
(the srot branch, the merge to main) is in `git log`.

Cross-repo rules: [`cross-repo-contract.md`](cross-repo-contract.md) (mirrored as `AGENTS.md`
in each sibling repo).
