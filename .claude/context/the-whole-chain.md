# What owning the whole chain lets us do that nobody else can

> Hardware (SROT board) → firmware (Hengla) → **ESC firmware (Bluejay)** → thruster
> (custom) → companion stack (`mongla_ws`). Every layer is ours to change.
>
> Almost every other team buys T200s, Blue Robotics ESCs, a Pixhawk and ArduSub.
> They can tune gains. That is the whole of their control authority.

⛔ **This page is only for things that are IMPOSSIBLE without owning the layer**,
each one tied to a measurement we have actually taken. Anything achievable by
tuning belongs somewhere else.

---

## 1. ⭐⭐ Current sensing at the ESC → the achieved wrench, measured

**The single highest-value item on this page.**

Motor torque is `Q = Kt · I`, exactly — `Kt = 9.55/KV = 0.0104 N·m/A` for the DJI
2212. Propeller torque and thrust are linked through the propeller's own
characteristic. **So per-thruster current is a real-time thrust proxy, at the
ESC, at kHz rates.**

What that one signal delivers, all at once:

| it gives | which today is |
|---|---|
| the **achieved wrench** | [ask B](upstream/pr-b-telemetry-budget.md) — currently *modelled*, never measured. The allocator's scale-down reaches nobody |
| INDI's `τ_applied` | its stated precondition. [§12](measured-bars.md) shows INDI is sensitive to exactly this |
| fault classification | rpm alone tops out near 78 % accuracy; current separates a fouled prop from a dead phase |
| the thrust curve, continuously | `k_n_per_rpm2` is `null` and `MOT_THST_EXPO = 0.65` is a T200 figure — this calibrates *in situ*, every run |

⚠ **And it is a live procurement decision.** Bluejay runs on BLHeli_S-class
hardware, which **typically has no current shunt**. [Ask M §6.3](upstream/pr-m-thruster-requirements-from-control.md)
flags this. Choosing an ESC variant with a shunt costs nothing extra now and is
unobtainable later.

⭐ **This is the item to decide this week**, because it is the only one on this
page with a closing window.

---

## 2. ⭐ Delete the DShot quantum — it is a convention, not a law

[Measured](measured-bars.md): the smallest non-zero output is **16.22 % of full
scale**, and the usable yaw band is **4.06 : 1**. That comes from two things we
inherited rather than chose:

- **`MOT_SPIN_MIN`**, lifting every demand into `[0.15, 1]` — because a sensorless
  ESC cannot start a stopped prop reliably;
- **DShot 3D's 999-step band**, a protocol convention.

**We own both ends of that wire.** A team using off-the-shelf ESCs cannot touch
either. We can:

1. ⭐ **Close a speed or thrust loop inside the ESC.** Bluejay already computes
   commutation timing, so it holds the **best rpm estimate in the system, at the
   highest rate** — and throws it away down a 20 Hz telemetry pipe. A loop there
   makes the floor unnecessary: the ESC guarantees the prop turns, so the board
   need not lift the demand to force it.
2. **Replace the command encoding.** 999 steps is not a physical limit.

⚠ **Sequencing matters.** [§11](measured-bars.md) measured the floor's
*steady-state* cost at only ~0.1° of pointing jitter, so this is **not** urgent
for holding a target. It is urgent for **small corrections**, which are a lurch
today.

---

## 3. ⭐ The mixer computed from geometry at boot — and why it blocks us now

⛔ **The finding that reframes [ask K](upstream/pr-k-the-mixer-is-for-a-different-hull.md).**
`geometric_allocation.py` exists, is tested, proved the hull is **rank 5 of 6 with
roll unactuated** — and has **zero production callers.**

It cannot have any. `_srot_drive` sends `MANUAL_CONTROL` with four axes and **the
board does the mixing**. The host never addresses a thruster. So a geometric
allocator is not "not wired up yet" — it is **structurally unreachable while the
board owns the mixer.**

Ask K should therefore be argued as: *the mixer is what prevents geometry from
being used at all*, not merely *the mixer is for a different hull*. Either the
board computes `M` from geometry at boot, or a per-thruster command path exists.
Until one of those lands, **dead-thruster tolerance, weighted allocation and
redistribution are all unreachable**, however well tested they are here.

---

## 4. Co-designing the thruster with the controller

Normally the mechanical team fixes the geometry and control copes. We measured
that **thruster lag beats control-loop rate by 131×**, and that
`τ = L·√(ρA/T)` — so **duct length is a control parameter**.

⭐ Asking for a 150 mm tunnel to become 100 mm is a **CAD change worth more than
every gain on the vehicle**. No team that buys thrusters can make that request.
[Ask O §4.3](upstream/pr-o-thruster-step-response.md) proposes testing it directly
by printing two duct lengths.

---

## 5. One clock across the whole chain

ESC, Pico, ESP32, Pi — every clock is ours. Most teams cannot measure their own
actuator latency at all; we can stamp a command at the host and see it arrive at
the commutation. That is what makes a **latency budget** real rather than
estimated, and it feeds the localization retrodiction that already replays late
measurements at the instant they describe.

---

## 6. ⭐ The bench is itself a whole-chain artefact

`tools/control_bench/` compiles the **board's own control code** natively and
flies it on a Fossen plant built from our **own CAD geometry**. Neither half is
possible without owning both ends: you cannot compile firmware you do not have,
and you cannot build a plant for a hull you did not draw.

It has already refuted **five** of our own claims — `mass_kg: null`,
`k_roll = 0`, the `MOT_SPIN_MIN` precision argument, "not a faster thruster", and
INDI's prerequisite. ⭐ **That refutation rate is the actual competitive
advantage**, and it compounds: every wrong idea now costs minutes instead of a
pool day.

---

## 7. Ranked, by what today's measurements support

| # | item | why now |
|---|---|---|
| 1 | **ESC with a current shunt** | ⭐ the only closing window — it is a procurement choice being made now |
| 2 | **Shorten the ducts** | `τ ∝ L`, and lag costs 131× what loop rate does. A CAD change |
| 3 | **Ask K: geometry-derived mixer** | unblocks the entire allocator, which is written and unreachable |
| 4 | **ESC-side speed loop** | retires `MOT_SPIN_MIN` and the 16.22 % quantum |
| 5 | **One clock** | makes the latency budget measurable |

⛔ **What is NOT on this list, and deliberately:** a better attitude controller.
[§12](measured-bars.md) measured INDI against the tuned cascade and the cascade
won. The controller is not the limiting factor — **the actuator and the
allocation path are.**
