# Ask M — what the control layer needs from the custom thruster

**Status:** a requirements ask to the **hardware / thruster design team**, not a firmware PR.
Nothing here asks anybody to change code.

**Why now.** The T200 is a placeholder. Custom thrusters are being designed, and the thruster
is the one component whose properties the control layer cannot work around. Every constant
below that we currently ship is fitted to a T200 datasheet, and every one of them is a
placeholder for the same reason.

**Provenance.** Everything numeric in this document is either (a) read in the firmware source
with file and line, or (b) measured on `tools/control_bench/`, which compiles the board's own
`mixer.cpp`, `attitude_control.cpp`, `feedforward.cpp`, `thrust_trim.cpp` and
`depth_control.cpp` natively and is validated against live-board captures to under one DShot
count of 999. Where a number comes from a vendor datasheet it says so, and that number is a
placeholder.

---

## 1. The measurement that prompted this

Host yaw stick, through `attitude::stabilize` into the mixer, into a motor, on the bench.
Single-shot command, loop otherwise quiet:

| host yaw stick | yaw torque at the mixer | motor output |
|---|---|---|
| 3.0 % | 0.002972 | **0.0 %** |
| 4.0 % | 0.003963 | **0.0 %** |
| 5.0 % | 0.004956 | **0.0 %** |
| 7.5 % | 0.007444 | 16.8 % |

`VISION_YAW_MIN_PCT = 5.0` is the floor our vision servo lifts small yaw demands to. It
produces **zero motor output** — it falls 0.9 % short of the mixer's `0.005` cliff. The first
demand that moves the vehicle at all produces **16.8 %** of full scale.

That cliff is not a mixer bug. It is `MOT_SPIN_MIN` doing exactly what it was written to do:
lift any non-zero demand up to a floor, because a brushless thruster below that floor does not
turn reliably — it cogs, stalls, or sits drawing current. **The floor exists because of the
thruster.** A thruster that turns smoothly from zero removes it.

**So the single most valuable property a custom thruster can have, for control, is not more
thrust. It is resolution near zero.**

---

## 2. What we ask for, ranked

### 2.1 ⭐ Turn smoothly from zero — so `MOT_SPIN_MIN` can go to 0

What makes a floor necessary today:

- **cogging torque** — the detent the rotor has to break out of;
- **static friction in the shaft seal and bearings** — the part that grows with depth rating;
- **ESC startup behaviour** — a sensorless six-step controller has no idea where the rotor is
  until it is already turning, so it has to kick.

Each has a design answer: a skewed or fractional-slot stator for cogging; a low-drag seal for
static friction; and **a sensored or FOC ESC** for startup, which knows rotor position at zero
speed and can produce rated torque there.

What it buys us, concretely:

- the 16.8 % quantum disappears and precision alignment becomes expressible. Today the
  smallest yaw correction the vehicle can make is a lurch;
- **INDI becomes possible.** An incremental controller works by commanding small increments.
  An actuator whose smallest non-zero increment is 16.8 % of full scale is not an incremental
  actuator, and INDI on this hull is blocked on exactly this, nothing else;
- the vision servo stops needing a minimum-percentage constant at all.

**This is the ask. If only one thing on this page is designed for, make it this one.**

### 2.2 Symmetric forward and reverse

We currently carry `REVERSE_EFFICIENCY = 0.77` — a T200 property: the propeller is optimised
for one direction and gives about three-quarters of the thrust the other way. Every allocation
we compute has to know the sign of each thruster's demand before it can know what that demand
is worth, and the asymmetry makes the effectiveness matrix demand-dependent.

A symmetric propeller section makes `B` constant, which is worth more to the controller than
the few percent of peak forward thrust it costs. On a vehicle that spends its competition run
station-keeping and correcting, **reverse is not the rare direction.**

### 2.3 Telemetry: RPM **and current**, and do not make shaft speed the primary mode

- **RPM.** `k_n_per_rpm2` is `null` in `hull_geometry.yaml` and stays null until a thruster
  exists to measure. Bidirectional DShot already carries rpm; we need it to be real. Measured
  2026-09-07: 958 CRC-valid `ESC_STATUS` frames with nothing attached, **every rpm exactly 0** —
  so a frame count can never prove a thruster is alive (see [`pr-f`](pr-f-esc-presence-on-the-wire.md)).
- **Current, alongside it.** Fault classification from rpm alone tops out around 78 % in the
  literature; current is what separates a fouled prop from a dead phase. The Pico already
  decodes current and discards it (see [`pr-c`](pr-c-pico-esc-health.md)), so the wire side is
  half-built — the ask here is that the ESC actually measures it.
- ⚠ **Do not make closed-loop shaft speed the primary control mode.** Sørensen's comparison of
  thruster control modes is unambiguous: under wave-induced inflow a shaft-speed controller
  delivers wildly less thrust than a torque controller (order 18 N against 65 N in the reported
  case), because holding rpm constant while inflow changes is precisely the wrong thing to do.
  Shaft speed as *telemetry*: yes, please. Shaft speed as the *setpoint*: no.

### 2.4 A measured bollard thrust curve, at our voltage

`MOT_THST_EXPO = 0.65` is fitted to the T200. It is the curve the firmware uses to linearise
demand into thrust, and it is currently describing a thruster we will not fly.

We need, from the real unit: **thrust against command, forward and reverse, at the battery
voltage we actually run** (the vendor's own reverse ratio drifts 0.787 → 0.754 across 12–20 V,
so a single-voltage curve is not enough), plus **thrust against rpm²** to give `k_n_per_rpm2`
its first real value. A load cell and an afternoon closes two open rows in `measured-bars.md`
and one in `BUGS.md`.

### 2.5 Thrusters as identifiable bodies in the CAD

Reading the Onshape document on 2026-09-23 for `hull_geometry.yaml`, the thruster positions
came out of the tunnel bores and one axial placeholder, because the fitted units in the model
are three A2212 placeholders. Once the real unit exists, **name the bodies** — a consistent
part name per thruster is what lets `tools/pull_hull_geometry.js` re-derive geometry every time
the design moves, instead of us identifying a thruster by where it happens to sit.

And while it is on the list: ⭐ **assign materials.** Onshape reports no material on any part,
so mass, centre of mass and the inertia tensor are not merely unmeasured but *uncomputable*.
That blocks every claim in newtons, the CoB restoring term the firmware already has written,
and INDI's 6×6 effectiveness matrix. It is a dropdown per part, once the hull material is
chosen.

---

## 3. What this does not ask for

**Not more thrust.** Authority is not our limit; resolution is. A thruster with twice the peak
and the same floor makes the problem worse, because the quantum scales with the peak.

**Not a faster thruster.** A T100-class unit's thrust dead time is around 0.59 s, and the
control loop runs at 2 ms. The actuator's lag already dominates the loop by two orders of
magnitude, and shaving it is not where the next win is. Respond *smoothly*, not *fast*.

**Not a redesign for fault tolerance.** The allocator handles a dead thruster as a limit
change (`u_min = u_max = 0`) and degrades rather than tumbles. We need to *know* the thruster
died — which is §2.3 — not for it to never die.

---

## 4. How we will verify each of these

| ask | how it gets checked |
|---|---|
| §2.1 smooth from zero | a demand ladder on the bench with `MOT_SPIN_MIN = 0`: the output must be monotone and continuous through the first non-zero step, with no 16.8 % jump |
| §2.2 symmetry | bollard thrust at ±command, matched pairs; `REVERSE_EFFICIENCY` retires when the ratio is within measurement error of 1.0 |
| §2.3 telemetry | rpm and current must move with a *known* mechanical change (a fouled prop, a held shaft) — a non-zero reading is not evidence, per the 958-frame result |
| §2.4 curve | the fitted curve predicts a held-out thrust point; the number lands in `measured-bars.md` with voltage and method |
| §2.5 CAD | `tools/pull_hull_geometry.js` re-runs and reports `materials_assigned: true`, and identifies every thruster by name rather than by position |

---

## 5. Related

- [`pr-i-spin-min-relay.md`](pr-i-spin-min-relay.md) — the firmware-side half of §2.1: the
  relay behaviour measured and filed. If the thruster removes the need for the floor, PR I's
  ask becomes a parameter change rather than a control redesign.
- [`pr-k-the-mixer-is-for-a-different-hull.md`](pr-k-the-mixer-is-for-a-different-hull.md) —
  the mixer is written for eight T200s at 45°; the hull being built has five orthogonal units.
- [`pr-f-esc-presence-on-the-wire.md`](pr-f-esc-presence-on-the-wire.md),
  [`pr-c-pico-esc-health.md`](pr-c-pico-esc-health.md) — the wire side of §2.3.
- `src/mongla_control/config/hull_geometry.yaml` — where `k_n_per_rpm2` waits.
