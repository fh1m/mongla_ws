# Control, against the world

> Dive 1 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`, never out of a document. Their side carries a URL and a number.

---

## 1. What we do today

### 1.1 The one-paragraph shape

Two actuation backends behind one HAL (`src/mongla_control/mongla_control/fc/base.py`). On
**ArduSub/Pixhawk** the host runs *every* outer loop in Python over `RC_CHANNELS_OVERRIDE`. On
**srot** the host sends *intent* (`SROT_MOVE`, cmd 31000) and the board runs the loops at 500 Hz;
the only streamed host loop left is the vision loop over `MANUAL_CONTROL`.

**Every controller in the package is P, P-with-floor, or PID on a scalar error.** There is no
state-space, no LQR, no MPC, no QP, no pseudo-inverse, no feedforward plant model, no added mass,
no drag model, and no acceleration or jerk limit anywhere in the stack.

### 1.2 Every loop that exists

| loop | file:line | rate | law |
|---|---|---|---|
| vision **lateral** | `motion_vision.py:1230-1245` | detection rate (floor 50 Hz srot / 20 Hz ArduSub, `motion_rates.py:15-32`) | **PI** on normalised pixel error, `p_lat = ctrl·kp_lat·rgain`, `lat_i` clamped ±15 % |
| vision **yaw** | `motion_vision.py:1247-1274` | same | **P + tapered stiction floor**, deadband, no I, no D |
| vision **depth** | `motion_vision.py:1288-1296, 1392-1426` | setpoint stepped at 5 Hz | **rate-limited P integrator into ArduSub's own PID**; refused on srot |
| vision **surge** (downward) | `motion_vision.py:1276-1287` | same | two-sided P reusing `kp_lat` |
| **standoff** (forward fill) | `motion_vision.py:1307-1361` | same | **one-sided** P on fill deficit, never reverses |
| `_YawPID` | `motion_yaw.py:134-189` | 10 Hz | **full PID**, dt-normalised. Kp 1.2 %/deg, Ki 0.03, Kd 0.5; integral reset on sign flip and inside tolerance |
| `HeadingLock` | `heading_lock.py:311-375` | 50 Hz | **pure P**, deadband 1.0°, tapered floor 5 %→0 over 1–6°. Ch4 only |
| `hold_depth` | `motion_depth.py:79-253` | 5 Hz | **no host PID** — a setpoint streamer with a 2.5 s ramp, hull-tracking clamp, 0.30 m brake zone |
| `thrust_loop` | `motion_writers.py:161-204` | 20 Hz | **open loop**; heading drift logged, not corrected |
| `drive_*_dist` | `motion_forward.py:180-285` | 20 Hz | **bang-bang** + reverse-kick brake; refuses without a position source |
| `style_roll` depth hold | `mongla.py:625-800` | 20 Hz | cos-modulated P inside ACRO, hard surface abort at −0.15 m |

Output conversion is one line: `percent_to_pwm` (`pixhawk.py:763-780`), `1500 + pct/100·400`
clamped to 1100–1900 — **the ±100 % limit is enforced at the wire, not in any controller.**

### 1.3 Allocation — the only place the mixer is reasoned about

`src/mongla_control/mongla_control/allocation.py`, 227 lines.

- `MIXER` (`:46-55`) is an 8×6 matrix of **exactly 0 or ±1**, a hand-mirror of the firmware's
  `mixer.cpp` for `SUB_FRAME_VECTORED_6DOF`. Block-diagonal: motors 1–4 carry yaw/forward/lateral,
  motors 5–8 carry roll/pitch/throttle.
- `allocate()` (`:90-117`) reproduces the board's own law: per-group **uniform scale-down**,
  `s = 1/max|u|` when `max|u| > 1`, computed separately for the horizontal and vertical groups.
- `headroom()` (`:120-140`), `largest_axis_within_budget()` (`:143-173`) — a **1-D feasibility
  bound**, solved per motor row, not an optimisation.
- `prioritise()` (`:193-227`) — a **greedy lexicographic fit**. Horizontal order
  (yaw, lateral, forward), vertical order (roll, pitch, throttle). Each axis takes
  `min(|want|, room)` against what is already granted.

Consumed from exactly two places, both in the vision path: `motion_vision.py:629-645`
(`_mixer_saturated` → lateral anti-windup) and `:648-692` (`_srot_drive` → priority fit before the
frame leaves). **Nothing on the ArduSub RC-override path consults it at all.**

⛔ Two facts the file states about itself: the board's scale-down **is never reported on any
message** (`:3-15`), so the host recomputes it forward; and the ±1 entries are a **demand mix, not
geometry** — used as a force sum they overestimate surge/sway by √2 = 41 %
(`estimator/thrust_model.py:45-51`).

### 1.4 The vehicle model we have, and the one we do not

| term | state |
|---|---|
| mixer | present, ±1, mirrored from firmware and asserted by `test_allocation.py:46` against `mixer.cpp` |
| thrust vs RPM | `thrust_model.py:57-107`, `T = k·n²` with `REVERSE_EFFICIENCY = 0.77` — and **`k` has no default on purpose**; nothing in the host or the firmware carries an absolute N-per-RPM². **Zero callers, asserted by a test.** |
| drag | **none.** The only drag reasoning is a comment arguing `T ∝ n²` against quadratic drag ⇒ steady speed ≈ linear in demand (`command_velocity.py:26-29`) |
| added mass | **none, anywhere.** Water inertia is handled empirically as reverse kicks (`REVERSE_KICK_PCT = 25`, `REVERSE_KICK_SEC = 0.20`) |
| battery compensation | **none on the host** — the firmware mixer scales by PM2 voltage; host compensation would double-count (`ROADMAP` L2) |
| the board's pilot shaping | **known and unmodelled**: `demand = ((1−EXPO)·u + EXPO·u³)·SPEED`, `EXPO = 0.30` (`srot_protocol.py:528-541`) — a **30 % small-signal droop exactly in the terminal-alignment regime**, plus a cubic that makes one `kp` wrong at both ends |
| learned velocity model | `command_velocity.py` — `v_ss = g·u + b` per axis by **RLS** (forgetting 0.995) + 1 s lag, gated on 100 excited samples and RMS ≤ 0.10 m/s. It feeds **localization**, and nothing in `mongla_control` reads it |

### 1.5 What the board owns vs what we own (srot)

| axis | host | board |
|---|---|---|
| roll / pitch | nothing — `MANUAL_CONTROL` has no such field | 500 Hz stabiliser |
| heading | nothing during vision (`release_yaw` → yaw 0); discrete turns are `MOVE_TURN` intent | 500 Hz hold + turn state machine |
| depth | **nothing** — `up = 0.0` always; no `set_target_depth`; depth-setpoint aligns refused | `DEPTH_HOLD` latch or `MOVE_DIVE` — ⛔ **never run closed** |
| surge / sway | the vision P loop, host-prioritised, streamed as `MANUAL_CONTROL` | mixer + the unreported group scale-down |
| timed legs, braking | send intent, relay ACKs | ramp, cruise, brake (the host brake was removed at fw rev 2) |

Refused before dispatch (`srot_fc.py:2823-2827`): `lock_heading`, `move_forward_dist`,
`move_back_dist`, `move_lateral_dist`, `arc`, `style_yaw`.

### 1.6 The setpoint shaping that does exist

No Ruckig (measured and rejected), no jerk limit, no online trajectory generation. What exists:
`smoothstep` / `smootherstep` (`motion_easing.py:11-29` — quintic chosen because ArduSub
differentiates the setpoint, so a C¹ corner appears as a rate spike), `trapezoid_ramp` (`:32-51`,
a thrust-scale envelope, not a velocity profile), the yaw sweep (`motion_yaw.py:336`), the depth
ramp (`motion_depth.py:198-212`), and the 5 Hz vision depth step (max slew 0.1 m/s).

### 1.7 Honest labels already in the code

`VISION_YAW_MIN_PCT = 5.0` — *"a hardware spin-up assumption, NOT a measured value"*
(`motion_vision.py:104-109`). `VISION_YAW_FLOOR_FILL`, `VISION_YAW_APPROACH_BAND_PX`, the four
brake constants, `MIN_ALIGN_ERR_PX`, `FWD_BAND`, `LOCK_HOLD_DEADBAND_DEG` — all labelled
**pool-tunable**, none measured. `heading_lock.py:96-99` states the pure-P **steady-state droop**
under a sustained disturbance as a known, accepted error. `STYLE_ROLL_DEPTH_KP = 150.0` and its
three siblings cite no measurement at all.

Shipped **off** by default: `ki_lat = 0.0` (lateral integral), `range_gain_floor = 1.0`
(range-adaptive gain scheduling), `settle_px = 0.0`.

---

## 2. What the best work does

*(Pending the research sweep for this dive.)*

---

## 3. The gap

---

## 4. Candidate moves

---

## 5. Rejected, with the reason
