# ArduSub canon -- modes, failsafes, controllers, parameters

> The first-principles reference for everything in our stack that
> ultimately talks to ArduSub. This is the *theory* doc -- the
> "why does ALT_HOLD do that?" doc. For the message-level cookbook,
> see [`mavlink-reference.md`](./mavlink-reference.md). For the
> parameter quick-list and known quirks, see
> [`ardusub-reference.md`](./ardusub-reference.md).
>
> Sources cross-checked against ArduSub docs and the ArduPilot
> source tree (Sub-stable-V4.5.x):
>
> * https://www.ardusub.com/
> * https://ardupilot.org/sub/
> * https://ardupilot.org/sub/docs/parameters.html
> * https://ardupilot.org/sub/docs/parameters-Sub-stable-V4.5.7.html
> * https://ardupilot.org/sub/docs/pilot-control.html
> * https://ardupilot.org/sub/docs/pilot-control-failsafe.html
> * https://ardupilot.org/sub/docs/sub-frames.html
> * https://ardupilot.org/sub/docs/sub-configuration-landing-page.html
> * https://ardupilot.org/sub/docs/common-mavlink-mission-command-messages-mav_cmd.html
> * https://ardupilot.org/sub/docs/common-MAVLink-high-latency.html

---

## 0. Mental model in one paragraph

ArduSub is ArduPilot's port for ROVs/AUVs. It runs at ~400 Hz on
the Pixhawk and exposes a finite set of **flight modes**. Each mode
is a fixed assignment from RC channels to controlled axes. The
companion (us) does NOT close any control loop on the AUV's pose --
ArduSub does. We just decide *what mode* to be in, and *what
six numbers* (Ch1..Ch6) to push at it, plus the optional absolute
depth setpoint when we want ArduSub to drive somewhere specific in Z.
Everything else (thruster mixing, motor saturation, PID, EKF) is
ArduSub's problem. Our entire job is "pick the mode, push the
sticks". That's why our code is small.

---

## 1. Frame & axes

We run **`vectored_6dof`** (8 thrusters, BlueROV2-Heavy layout).
Axes follow the standard aerospace body frame with X forward,
Y right, Z down (NED). Yaw is positive clockwise looking *down*.
"Depth = 0" means at the surface; "depth = -1.5" means 1.5 m below.

The thruster mixer is owned entirely by ArduSub; we never touch
individual motors. We only push the six logical sticks (Ch1..Ch6)
and ArduSub's mixer figures out the per-thruster PWM.

| Stick     | Axis (body)             | RC channel | Our constant      |
| --------- | ----------------------- | ---------- | ----------------- |
| Pitch     | rotation about Y        | Ch1        | `CH_PITCH = 0`    |
| Roll      | rotation about X        | Ch2        | `CH_ROLL = 1`     |
| Throttle  | translation Z (heave)   | Ch3        | `CH_THROTTLE = 2` |
| Yaw       | rotation about Z        | Ch4        | `CH_YAW = 3`      |
| Forward   | translation X (surge)   | Ch5        | `CH_FORWARD = 4`  |
| Lateral   | translation Y (sway)    | Ch6        | `CH_LATERAL = 5`  |

PWM convention: 1500 = neutral, 1100 = full negative, 1900 = full
positive. Our `percent_to_pwm(p)` maps `p∈[-100, 100]` linearly
into `[1100, 1900]`.

We deliberately avoid Ch1 (pitch) and Ch2 (roll). The
`vectored_6dof` frame can in principle drive both, but on Duburi the
buoyancy is tuned for "stay flat" and we want STABILIZE / ALT_HOLD
to keep the bow level. See [`axis-isolation.md`](./axis-isolation.md)
for the per-channel ownership rules.

---

## 2. Modes -- what we actually use

ArduSub has ~10 modes; we touch four. The IDs come straight from
`master.mode_mapping()`.

### 2.1 MANUAL (id 19)

Pure RC passthrough. ArduSub does **no** stabilisation, no depth
hold, no compass-aided yaw. Whatever you put on Ch1..Ch6 goes
through the mixer to the thrusters and that's it.

We use MANUAL only as a known-good baseline for unit testing
single-axis pushes. We **do not** run missions in MANUAL because
the AUV will pitch / roll under any thrust asymmetry, and depth
will drift as soon as the heave channel goes neutral.

### 2.2 STABILIZE (id 0)

Same as MANUAL except ArduSub's attitude controller (`AC_AttitudeControl`)
keeps roll & pitch level when Ch1/Ch2 are neutral. Throttle (Ch3)
is still raw -- you have to fly depth manually.

We touch this only when ALT_HOLD refuses to engage (e.g. EKF
not ready, depth sensor invalid). It's a fallback, not a target.

### 2.3 ALT_HOLD (id 2) -- the workhorse

This is what every mission lives in. ArduSub's full assist:

* Roll & pitch: stabilised level (Ch1/Ch2 ignored unless you push them).
* Yaw (Ch4): **rate command**. Stick deviation = desired yaw rate.
  Stick neutral = "hold current heading", and ArduSub closes the loop
  internally with `ATC_RAT_YAW_*`.
* Forward (Ch5) and lateral (Ch6): direct thrust passthrough; no
  position loop (we don't have a DVL on Duburi).
* Throttle (Ch3): **depth-rate command**. Stick deviation = ascend /
  descend rate. Stick neutral = "hold current depth", closed by
  ArduSub via the PSC cascade (see §3).

Everything we call "drive forward", "yaw to heading", "lock heading",
"set depth", etc. is **ALT_HOLD plus the right RC stick combination**,
optionally with `SET_POSITION_TARGET_GLOBAL_INT` when we want an
absolute Z setpoint instead of a Z-rate stick.

### 2.4 GUIDED (id 4)

Accepts external position targets. We *only* use this implicitly
inside ALT_HOLD via `SET_POSITION_TARGET_GLOBAL_INT` with the type-mask
that masks all axes except `z`. We do not enter pure GUIDED, because
without DVL the EKF has no XY observation and `SET_POSITION_TARGET`
in XY would either drift or be rejected.

The lesson: **stay in ALT_HOLD; use SET_POS_TARGET only for Z**.

### 2.5 SURFACE (id 9)

Emergency ascent to depth 0. We never command this, but it's worth
knowing because BlueOS's emergency button maps to it.

---

## 3. The depth controller cascade

When we send `SET_POSITION_TARGET_GLOBAL_INT` with `alt = -1.50`,
we are not commanding thrusters. We are handing a **z setpoint** to
the cascade:

```
target z (m)
   │
   ▼
PSC_POSZ_P              (position-error -> velocity-error)
   │
   ▼
PSC_VELZ_P              (velocity-error -> acceleration-cmd)
   │
   ▼
PSC_ACCZ_P / PSC_ACCZ_I (acceleration -> normalised throttle)
   │
   ▼
ArduSub thruster mixer (8 motors, vectored_6dof)
```

Inside the cascade the inputs are wired up like this (defaults from
ArduSub 4.5):

| Param          | Default | What it does                                                       |
| -------------- | ------- | ------------------------------------------------------------------ |
| `PSC_POSZ_P`   | 1.0     | m of error → m/s desired velocity                                  |
| `PSC_VELZ_P`   | 5.0     | m/s of velocity error → m/s² desired accel                         |
| `PSC_ACCZ_P`   | 0.5     | m/s² of accel error → throttle (normalised 0..1)                   |
| `PSC_ACCZ_I`   | 1.0     | accel integrator -- compensates buoyancy bias                      |
| `PSC_ACCZ_FF`  | 0.0     | accel feedforward -- typically 0 underwater                        |
| `PILOT_SPEED_DN` | 50    | cm/s -- maximum descent rate the throttle stick maps to            |
| `PILOT_SPEED_UP` | 50    | cm/s -- maximum ascent rate the throttle stick maps to             |

**Practical implication for our stack**: once `set_depth(z)`
returns, we stop pushing setpoints. ALT_HOLD's `PSC_POSZ_P` is
already holding `z` -- it doesn't need a fresh setpoint every
200 ms; it needs a setpoint *once*, then "stop bothering me". This
is why we deleted `DepthLock`: it was streaming the same setpoint at
5 Hz for no reason. The neutral throttle from `Heartbeat` (Ch3=1500)
is what tells ArduSub "no depth-rate command", and the cascade does
the rest.

---

## 4. The yaw controller (rate vs angle)

ALT_HOLD's yaw stick is **a rate command**, not an angle command.
Specifically the stick deviation is interpreted as:

```
desired yaw_rate = stick_dev * ACRO_YAW_P    [deg/s]
```

`ACRO_YAW_P` defaults to 4.5 (so a full-scale stick = 4.5 * 100 deg/s
= 450 deg/s in theory, capped by `ATC_RATE_Y_MAX` ≈ 75 deg/s in
practice).

Internally ArduSub closes the rate loop with `ATC_RAT_YAW_P/I/D`,
and when the stick is centred it engages a **hold sub-loop**: the
*current yaw at stick-release time* becomes the target, and the
attitude controller drives error to zero with
`ATC_ANG_YAW_P`. That's why "neutral Ch4 in ALT_HOLD" feels like a
yaw lock -- because it is one.

This is what `motion_yaw.py` exploits. We don't compute the heading
correction ourselves; we just push Ch4 deviation proportional to
heading-error percent until we're inside the deadband, then release
to neutral and let ArduSub's hold sub-loop catch it.

`HeadingLock` does the same thing continuously: every 50 ms it
reads our `read_heading` source, computes the error vs. the latched
target, and pushes a small Ch4 deviation. When error ≈ 0 it pushes
1500 -- which simply tells ArduSub "engage your hold sub-loop". So
even when our lock thread is "doing nothing", ArduSub is still
holding heading.

The reason we run our own lock at all (instead of relying purely on
ArduSub's hold) is **source flexibility**: ArduSub's hold uses its
EKF compass; we may want to lock to BNO085 yaw or to a vision
heading instead. Our lock is a thin proportional layer that
re-targets ArduSub's hold to whatever yaw source we picked.

---

## 4A. BNO yaw source: who reads what (mental-model fix)

`yaw_source:=bno085` does **NOT** push BNO yaw into ArduSub's EKF.
It only changes which sensor our **Python** loops read. ArduSub's
onboard 400 Hz attitude controller (the one that runs in
STABILIZE / ALT_HOLD-stick-release / yaw-hold-sub-loop) keeps using
its own compass-fed EKF -- it has no idea the BNO exists. To
actually feed BNO yaw into ArduSub's EKF you need
`EK3_SRC1_YAW=2|6` plus an `ATT_POS_MOCAP` / `GPS_INPUT.yaw` /
`ExternalNav` stream; that is parked as a future feature in
[`future-bno-into-ekf.md`](./future-bno-into-ekf.md).

So the right mental model is:

| Verb family | Reads absolute yaw? | Source today (when `yaw_source:=bno085`) |
| --- | --- | --- |
| `move_forward / _back / _left / _right` | yes -- heading-hold during translation | **BNO** (via `motion_writers.read_heading`) |
| `yaw_left / yaw_right / arc` | yes -- loop termination + correction | **BNO** (via `motion_yaw.read_heading`) |
| `lock_heading` (background daemon) | yes -- primary input | **BNO** (via `heading_lock._run`) |
| `set_depth` / `hold_depth` | no -- depth from AHRS2 (Bar30) | n/a |
| `vision_align_*` / `_hold_distance` / `_acquire` | no -- camera pixel error IS the closed-loop signal | n/a |
| `[STATE]` telemetry log | yes | **BNO** (via `_effective_yaw_deg`) |
| `Move.Feedback` pump | yes | **BNO** (via `_effective_yaw_deg`) |
| ArduSub onboard yaw-hold sub-loop (Ch4 neutral in ALT_HOLD/STABILIZE) | yes | ArduSub's own compass-fed EKF |

Three useful corollaries:

1. **STABILIZE will appear to "react to yaw" even if BNO is the source.** That is ArduSub stabilising to its own IMU (gyros + compass), not BNO. The thrusters moving in STABILIZE when you tilt the sub by hand is **not** a BNO test -- it's an "ArduSub is alive" test.
2. **The BNO test is `lock_heading` / `yaw_*`.** Run those and watch the [STATE] yaw match BNO. If you yank the BNO USB cable mid-`lock_heading`, you should see `[LOCK ] yaw source silent` and Ch4 release to neutral. That proves BNO is the closed-loop sensor.
3. **Why prefer ALT_HOLD over STABILIZE for everything.** `ALT_HOLD` honours both our depth setpoint (`SET_POSITION_TARGET_GLOBAL_INT`) and our Ch4 rate input. `STABILIZE` honours Ch4 rate but `set_depth` is silently dropped, so the sub sinks during a turn. There is no advantage to switching to STABILIZE during yaw or translation, and there is a cost (depth drift). Our facade enforces ALT_HOLD via `_ensure_yaw_capable_mode` and `_ensure_alt_hold` for exactly this reason.

For the operator-side validation recipe, see [`testing-guide.md`](./testing-guide.md) §2.4 "BNO sanity at the pool".

---

## 5. The failsafes that bite us

ArduSub has many failsafes (battery, EKF, GCS, GPS-glitch, ...). The
ones that matter for our companion-driven workflow are:

### 5.1 `FS_PILOT_INPUT` (the big one)

If ArduSub stops receiving `RC_CHANNELS_OVERRIDE` for more than
`FS_PILOT_TIMEOUT` seconds (default 3.0), it triggers
`FS_PILOT_INPUT`. The default action is **disarm**. Mid-mission
disarm = we lose attitude hold, depth hold, and motors. The AUV
flops to the surface (positively buoyant) or sinks (negatively
buoyant). Either way, mission over.

This is why `Heartbeat` exists. It pushes a six-1500 RC override
at `HEARTBEAT_HZ = 5` (one packet per 200 ms, comfortably inside
3 s) **whenever no other writer is active**. When a motion verb
runs, it pauses Heartbeat (because the verb is doing the writing
itself), then the verb releases and Heartbeat resumes.

Mitigations beyond Heartbeat:

* `FS_PILOT_TIMEOUT = 0` disables the timeout entirely. We do NOT
  do this -- losing the timeout means a wedged companion can't
  trigger the failsafe at all, which is a *worse* operational
  posture than a quick disarm-on-fault.
* `FS_PILOT_INPUT = 0` makes the failsafe action "do nothing". We
  do NOT do this for the same reason.

### 5.2 `FS_GCS_ENABLE`

Independent of `FS_PILOT_INPUT`. Triggered when the **GCS heartbeat**
stops (the MAVLink HEARTBEAT message from `MAV_TYPE_GCS`). pymavlink's
`mavutil.mavlink_connection` sends GCS heartbeats automatically as
long as the connection thread is alive. We don't typically have to
manage this -- but if we ever stub the connection out, we'd have to
remember to keep heartbeats flowing.

### 5.3 `FS_EKF_ACTION`

Triggered when EKF3 reports innovations above `FS_EKF_THRESH`. At
default settings this is set to "change to ALT_HOLD" -- which is
where we already are, so it's a no-op. We rely on this: a momentary
compass spike will not eject us from ALT_HOLD.

### 5.4 `FS_BATT_ENABLE`

We leave this off in SITL and on the bench, but always enable it in
water with a sane voltage threshold. The action "disarm" is the
right thing -- by the time the FS triggers, the battery cannot
finish the mission anyway.

---

## 6. RC override priority & arbitration

ArduSub priority for control input (highest first):

1. `RC_CHANNELS_OVERRIDE` (us, via pymavlink)
2. `SET_ATTITUDE_TARGET` / `SET_POSITION_TARGET_*` in GUIDED
3. AUTO mission step
4. Physical RC receiver (we don't have one)

We always sit at level 1. This means the `RC_CHANNELS_OVERRIDE`
packet is authoritative for whichever channels we set. **Channels
we leave at 65535 (`UINT16_MAX`) are released back to the next
priority level** -- effectively giving them up entirely (since we
have no physical RC).

This is a sharp tool. If we send `[65535]*8` we have just told
ArduSub "I'm not driving any stick" -- in MANUAL the AUV freewheels
(no thrust), in ALT_HOLD it engages every internal hold (heading
hold, depth hold, level attitude). For an emergency *no-input*
state, this is exactly what `pause` does. For a normal between-verbs
state, we instead push neutral `1500`s (Heartbeat) so ArduSub still
sees us as "driving the stick to the centre", which keeps
`FS_PILOT_INPUT` quiet.

The key invariant: **at any one moment, exactly one Python writer
owns the RC channels.** That writer is one of: Heartbeat, an active
motion verb, or HeadingLock. The handoff is controlled by
`Heartbeat.pause/resume` and by the `Duburi._command_ctx` context
manager. See [`heading-lock.md`](./heading-lock.md) and
[`axis-isolation.md`](./axis-isolation.md).

---

## 7. Parameters we depend on (and what they should be)

Below: only parameters whose defaults differ from what we want, or
that we read so often that they're worth pinning here. This is the
"don't nuke the param file" list. For the full param dump, see
ArduSub's `parameters-Sub-stable-V4.5.7.html`.

| Param                | Default | What we set / why                                                               |
| -------------------- | ------- | ------------------------------------------------------------------------------- |
| `ARMING_CHECK`       | 1       | `0` for SITL & competition. Pre-arm GPS/EKF checks block companion arm otherwise. |
| `FS_PILOT_INPUT`     | 1       | `1` (disarm) -- safer than `0`. Mitigated by Heartbeat.                          |
| `FS_PILOT_TIMEOUT`   | 3.0     | leave at 3.0 s; Heartbeat publishes at 5 Hz so we have ~15× margin.              |
| `FS_GCS_ENABLE`      | 0       | leave at 0 -- pymavlink sends our GCS heartbeat already.                         |
| `BRD_SAFETYENABLE`   | 1       | `0` to remove the physical safety-button requirement (companion-only ops).      |
| `MOT_PWM_MIN/MAX`    | 1100/1900 | leave at default; matches our `percent_to_pwm` mapping.                       |
| `EK3_SRC1_POSXY`     | 0       | `3` only if a DVL is connected. Otherwise leave at 0 and stay out of POSHOLD.   |
| `EK3_SRC1_POSZ`      | 1       | `1` (depth sensor / barometer). Don't change.                                   |
| `EK3_SRC1_YAW`       | 1       | `1` (compass). We do NOT push BNO085 to EKF; we just use it inside our lock.    |
| `PSC_POSZ_P`         | 1.0     | tune later if depth response is sluggish.                                       |
| `PSC_VELZ_P`         | 5.0     | tune later -- if we see depth oscillation in water, drop to 3.0 first.          |
| `PSC_ACCZ_I`         | 1.0     | governs buoyancy-bias rejection; raise if the AUV slowly sinks/rises in hold.   |
| `ATC_ANG_YAW_P`      | 4.5     | leave default; affects ArduSub's yaw hold sub-loop responsiveness.              |
| `ACRO_YAW_P`         | 4.5     | leave default; sets stick-to-yaw-rate gain. Don't touch.                        |
| `RC_FEEL_RP`         | 50      | leave default.                                                                  |
| `BARO_ALT_OFFSET`    | 0       | Calibrate at the surface before each session (`MAV_CMD_PREFLIGHT_CALIBRATION`). |

---

## 8. Boot and arm sequence (what really happens)

This is the chronological story from "manager process starts" to
"AUV is ready for a mission":

1. Manager opens MAVLink (`udpin:0.0.0.0:14550`), waits for the
   first `HEARTBEAT` from the autopilot. Until this arrives, no
   command works.
2. Manager calls `set_message_rate` for `AHRS2`, `ATTITUDE`,
   `GLOBAL_POSITION_INT`, `STATUSTEXT` to push them to ~10 Hz.
   Without this they default to 4 Hz and our control loop starves.
3. Manager starts `Heartbeat`. Within 200 ms we are pushing six
   1500s at 5 Hz. The `FS_PILOT_INPUT` clock is now permanently
   reset.
4. Mission (or CLI user) issues `arm`. We send
   `MAV_CMD_COMPONENT_ARM_DISARM(p1=1)` and wait for COMMAND_ACK.
   ArduSub runs its arming checks (which we pre-disabled if
   `ARMING_CHECK=0`) and either acks or denies.
5. Mission switches mode to ALT_HOLD via `MAV_CMD_DO_SET_MODE`. We
   wait for the mode to stick (poll `master.mode_mapping_bynumber`
   inside `_ensure_yaw_capable_mode`). EKF must be initialised at
   this point or the mode change is rejected with STATUSTEXT
   `"PreArm: ..."`.
6. From here the mission is just: pause Heartbeat → push verb
   sticks for the verb's duration → release sticks → resume
   Heartbeat. Repeat until done.
7. Mission ends. We send `disarm`. ArduSub releases motors.
   Heartbeat keeps pushing 1500s (harmless when disarmed).
8. Manager shuts down. Heartbeat thread joins. MAVLink connection
   closes. ArduSub eventually times out and... goes back to whatever
   safe state it pleases.

---

## 9. Things ArduSub will do *behind our back* (and that's fine)

* **Re-engage attitude hold** when Ch1/Ch2 are neutral or released.
* **Re-engage depth hold** when Ch3 is neutral and we're in
  ALT_HOLD. This is the cornerstone of why the cascade in §3 is
  "set it once, leave it alone".
* **Re-engage yaw hold** when Ch4 is neutral. This is the reason
  `HeadingLock` doesn't have to fight ArduSub -- they cooperate.
* **Apply EKF reset on a big jump** (e.g., compass yaw flip). We
  see the heading jump in `read_heading`; we don't have to do
  anything special.
* **Drop into SURFACE on emergency** if the GCS sends it. We don't
  initiate, but we should be ready for the AUV to swim to the top.

---

## 10. Things ArduSub will NOT do for us (and which we therefore implement)

* **Vision-driven control.** Detector + tracker + control loop are
  all on the companion. ArduSub never sees a pixel.
* **Multi-target heading from non-compass sources.** ArduSub holds
  heading from its own compass-fed EKF; if we want to lock to BNO085
  or to a vision bearing, we run `HeadingLock` and override Ch4.
* **Mission sequencing / Python DSL / parameter parsing.** The
  ArduSub mission system (AUTO mode + waypoint table) exists but is
  way too rigid for our use. We do everything in `duburi_planner`.
* **Heartbeat-as-failsafe-guard.** ArduSub treats absence-of-RC as
  an alarm; it doesn't help us *prevent* the alarm.
* **`set_servo_pwm` payload control.** Yes, ArduSub forwards
  `MAV_CMD_DO_SET_SERVO`, but the *what-to-actuate-when* logic
  (drop, fire, grab) is mission-side.

---

## 11. Cross-references

* MAVLink message bytes for everything in this doc:
  [`mavlink-reference.md`](./mavlink-reference.md)
* Per-verb implementation map:
  [`command-reference.md`](./command-reference.md)
* Per-channel RC ownership rules:
  [`axis-isolation.md`](./axis-isolation.md)
* Heading lock implementation detail:
  [`heading-lock.md`](./heading-lock.md)
* PID / gain theory used in our motion modules:
  [`pid-theory.md`](./pid-theory.md)
* ArduSub quirks quick-list:
  [`ardusub-reference.md`](./ardusub-reference.md)
* Vehicle-side parameters (PWM range, frame, mass):
  [`vehicle-spec.md`](./vehicle-spec.md)
