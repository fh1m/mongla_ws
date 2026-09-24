# SROT board control and on-board estimation, compared with state of the art (round 2)

> Builds on `sota/control.md`, `VERDICT.md`, `SOTA-GAPS.md`, and `sources/{control-laws-and-rate,sota-control-laws,rpm-thruster-control,system-id-no-basin}.md`. INDI, QP cost, Durham's theorem, REVERSE_EFFICIENCY, loop rate, the load cell, free decay and Saunders & Nahon are **not** repeated here.
> Firmware read at `srot-control-board@22f0ccb`. References were cloned and cited by line: PX4 `e370d98`, ArduPilot `ee0c343`, Bluejay `0368d11`, AM32 `8574381`.
> Legend: ✅ read in code or source · ⚠ from a search snippet or my own arithmetic, not measured · ❓ hypothesis.
> Everything below is **BUILT, NEVER FLOWN** unless it says otherwise.

---

## 0. Six defects found in the firmware during this read (new; not in any earlier dossier)

| # | defect | where | effect |
|---|---|---|---|
| D1 | **IMAX clamps the raw ∫e, not the I-term.** The gains are ArduSub's (`AC_AttitudeControl_Sub.h:15-23`: 0.135/0.090/0.0036, IMAX 0.444). But ArduPilot clamps the *ki-weighted* integrator (`AC_PID.cpp:345-346`). We clamp `_integ` and then multiply by `ki` (`pid.h:98,101`). | `pid.h:98`, `attitude_control.cpp:171-173`, `depth_control.cpp:41` | ✅ The largest possible I-term is **0.09·0.444 = 0.040** on roll and pitch (ArduSub: 0.444, 11× larger), **0.018·0.222 = 0.004** on yaw (55× smaller), and **0.1·1.0 = 0.10 heave** on depth. ❓ If net buoyancy or a CoB moment needs more than 10 % heave or 4 % torque, the loop holds a **permanent steady-state error**. On depth that error is (B − 0.10)/0.5 m. |
| D2 | **The CoB auto-trim mixes units.** It adds Δ to the trim in output units, then bleeds the same Δ from `_integ`, which is in ∫e units (`feedforward.cpp:210-219` → `pid.h:115-119`). | `feedforward.cpp:207-219` | ✅ Total effort is conserved only if ki = 1. At ki = 0.09 each transfer adds about 0.91Δ of net effort. That is a second integrator in the loop (❓ it probably limit-cycles). It is off by default (`TRIM_EN 0`, `config.h:562`). |
| D3 | **The "drag feedforward" is positive feedback.** `+K·gx·|gx|` is fed with the *measured* rate (`feedforward.cpp:187-189`), so it cancels damping. An overestimated K gives **negative net damping**. | `feedforward.cpp:186-192` | ✅ The defaults are 0 (`config.h:557-561`). The first person to tune it can destabilise the rate loop. |
| D4 | **RPM targets are linear in a thrust-linear demand.** `norm` is the mixer output, which `thstExpo` treats as thrust-linear (`mixer.cpp:15-21`). Yet the RPM target is `nrm·rpm_full` (`task_dshot_rmt.cpp:141-143`), so thrust goes as nrm² and the small-signal gain collapses near zero. `thrust_trim` predicts `rpm_full·|d|` the same way (`thrust_trim.cpp:70-72`). | as cited | ✅ The mapping is wrong by √. ❓ The "spin-up/stop" oscillation that `config.h:214-222` blames on RPM control *as an architecture* may be partly this mapping plus the 30 rpm target floor. With `THR_TRIM_EN`, the gains pin to the ±25 % clamp at part throttle. With T = k·n², the true expectation is n ∝ √d. |
| D5 | **Anti-windup ignores the real saturation.** The PID tests only its own ±1 (`pid.h:94-99`). The rate FF, the drag/trim terms and the mixer's group scale-down (`mixer.cpp:65-68`) all come later. | as cited | ✅ When a group saturates, the integrators keep charging. (C-2 covers the "not on the wire" half. This is the on-board half.) |
| D6 | **The host redistribution trades yaw away to keep sway.** Least squares in mixed units weights force error against yaw error at 1 : 0.175², about 33 : 1. | `geometric_allocation.py:208-215, 285-336` | ✅ Measured by running it: `sway=1.5, yaw=0.1` → sway 1.498 (−0.1 %) but **yaw 0.088 (−12 %)**. That is the opposite of `HORIZONTAL_PRIORITY` and of published DP convention (yaw first). |

---

## 1. Attitude estimation on the board

**What we do.**
- The BNO085's own fusion supplies attitude: `SH2_GAME_ROTATION_VECTOR` plus `GYROSCOPE_CALIBRATED`, both at 400 Hz (`bno085.cpp:39,97-98`).
- The magnetometer is read only once, at boot, to set the yaw reference (`config.h:160-167`). That is the right call next to thrusters.
- `LINEAR_ACCELERATION` and raw accel were **dropped** to lighten SHTP (`bno085.cpp:89-91`).
- The IMU shares I2C0 with the Bar30. The code records a collision that produced a 608→744 mbar excursion (`bar30.cpp:139-143`, `task_sensor_read.cpp:211-216`).

**The reference.**
- BNO08x datasheet (⚠ snippet): GRV dynamic error **2.5°**, heading drift **0.5°/min**.
- PX4 EKF2, ArduPilot EKF3 and Betaflight all fuse **raw** gyro and accel at 1–8 kHz, with known filter group delay and their own bias states.
- Gyro noise density (⚠ snippets): ICM-42688-P **2.8 mdps/√Hz** vs BMI088 **14 mdps/√Hz**. The BNO085's internal gyro is Bosch-class. ⚠ I did not verify the exact part.

**The gap.** Attitude is adequate for pool heading and level (0.5°/min is 7.5° over a 15-min run, which the host EKF corrects). What is missing:
- A **known-latency angular-acceleration signal**, which INDI needs. The BNO's internal filtering is opaque.
- **Bus isolation.** I2C SHTP desync accounts for a large share of the defensive code in `bno085.cpp`.

**Recommendation.**
- *Software, now:*
  - Re-enable `SH2_LINEAR_ACCELERATION` at 100–200 Hz. §2 needs it.
  - Log GRV yaw on a stationary vehicle for 30 min, and around a 4×90° jig.
- *Hardware, when INDI is pursued:*
  - Add an ICM-42688-P on its own SPI bus at 1–2 kHz and run Mahony, or a 6-state ESKF (quaternion error + gyro bias), on core 1.
  - Cost: about 150–300 flops per update, **under 10 µs at 240 MHz** (⚠ my estimate).
  - Keep the BNO085 as an independent cross-check. Two estimators disagreeing is a health signal (doctrine: redundancy over tuning).

**Falsifier.** If stationary GRV drift is below 0.1°/min and the 90° jig error is below 1°, the software case for a second IMU rests on INDI alone. Defer it until INDI is committed.

**Priority.** Medium. Software half: cheap. Hardware half: needs the hardware team.

---

## 2. Depth and vertical-velocity estimation

**What we do.**
- The Bar30 is read at 20 Hz (`task_sensor_read.cpp:39-41`), at **OSR 256** (`bar30.cpp:144`, `MS5837::read(bits=8)`), with a blocking read.
- The depth PID runs at 500 Hz on that zero-order-held signal, with D **on the measurement** (`depth_control.cpp:84`, `pid.h:72`).
- There is no vertical-velocity estimate anywhere.

**The numbers (⚠ datasheet values from a snippet; the arithmetic is mine).**
- MS5837-30BA RMS resolution: **1.57 mbar at OSR 256** (≈1.6 cm of water), 0.28 mbar at OSR 4096, 0.20 at OSR 8192. Conversion time at 4096 is 8.2 ms max.
- Differencing 20 Hz depth: σ_w = √2·0.016/0.05 ≈ **0.45 m/s RMS**.
- At 500 Hz the D term is a **20 Hz pulse train**: 24 ticks of zero, then one tick of Δz/0.002. Through the 20 Hz derivative LPF (α ≈ 0.2), the peak is about 5× the mean.

**The reference.**
- ArduPilot's classic `AP_InertialNav` ran a third-order complementary filter (z, w, accel bias) with TC_Z = 5–7 s (⚠ snippet of older source).
- The current ArduPilot vertical controller is a cascade: P on position → PI on velocity → PID on acceleration (`AC_PosControl.cpp:1104, 1115, 1136`). Each stage takes the motors' `limit.throttle_*` flags.
- Sabatini & Genovese (*Sensors* 2014) rotate specific force into the navigation frame, then complementary-filter it with pressure altitude.

**The gap, in numbers.** Steady-state Kalman for the double integrator gives P_ww = √2·q^¾·r^¼. With an assumed accel noise of 300 µg/√Hz (4× the BNO spec, deliberately pessimistic):
- **OSR 256: σ_w ≈ 3.7 mm/s**
- **OSR 4096: σ_w ≈ 2.4 mm/s**
- Raw differencing: 450 mm/s. That is **about 100×** better, with a crossover of about 0.15 Hz.
- In practice the floor is set by a tilt-error leak (1° ≈ 0.17 m/s²), which the bias state absorbs over a few seconds, and by **dynamic pressure**: ½ρv² at 0.5 m/s ≈ 1.25 mbar ≈ **1.3 cm** of apparent depth that depends on surge. ⚠ Both are computed, not measured.

**Recommendation (software, about 80 lines, under 100 flops per tick).**
1. Make the Bar30 non-blocking: start the conversion, come back 9 ms later. Use OSR 2048–4096, under an I2C0 mutex shared with the BNO. The earlier "higher OSR failed" was the bus collision (`bar30.cpp:139-143`), not the OSR.
2. Add a 3-state filter (z, w, b_az). Predict at 500 Hz from world-frame vertical accel (GRV quaternion × linear accel). Update at 20–50 Hz from baro.
3. Rebuild depth as a cascade:
   - `sqrt_controller(z_err)` → w_target, with the acceleration limit identified from a heave step.
   - PI on w, with the velocity-ramp feedforward from `movement.cpp:171`.
   - A buoyancy feedforward (§7).

**Falsifier.**
- *Bench:* vehicle still in air, log σ_w. If it exceeds 2 cm/s, the accel path or its rotation is wrong.
- *Pool:* depth-hold RMS and heave-effort RMS, filter vs raw D. If effort RMS does not drop by at least 3× at equal depth RMS, revert.

**Priority.** High. It sits under the depth loop that has **never run closed**.

---

## 3. Controller structure

**What we do.**
- ArduSub's cascade, transcribed: P on angle → PID on rate, with target filter FLTT and derivative filter FLTD (`attitude_control.cpp:205-240`).
- Conditional integration (`pid.h:94-99`).
- Battery compensation copied from ArduPilot's thrust linearisation (`mixer.cpp:114-126`), but **off by default** (`MOT_BAT_V_MAX 0`, `config.h:509`).
- The turn is **bang-bang**: a constant rate until |err| < 0.03 rad, then a snap to heading hold (`movement.cpp:198-206`).
- The depth ramp is linear, with no velocity feedforward (`movement.cpp:171`).

**The reference.**
- **Anti-windup.** PX4 publishes the allocator's `unallocated_torque`. The rate controller turns its sign into positive and negative saturation flags (`MulticopterRateControl.cpp:196-215`). The integral then only grows away from the saturated side (`rate_control.cpp:88-100`). PX4 also reduces I on large rate error (`:103-110`).
- ArduSub's `vectored_6dof` mixer is **the same per-group normalisation as ours**, and it clears the RPY limit flags (`AP_Motors6DOF.cpp:500-548`). So on anti-windup **we are level with ArduSub and behind PX4**.
- **Input shaping.** ArduPilot's `sqrt_controller` (`control.cpp:575-600`) gives a linear region near zero and a √(2·a·err) deceleration branch beyond it. It is used for every attitude axis (`AC_AttitudeControl.cpp:1352-1368`), with jerk-limited shaping in `shape_angle_vel_accel` (`control.cpp:433`).
- **Controller law.** INDI is covered in `control.md` §2.9. Pool-validated model-free HOSMC reports **75 % lower tracking error than PID** (`system-id-no-basin.md` §6, ⚠ snippet). Neither is worth adopting until D1, D3 and D5 are fixed. A new law on top of a broken integrator measures the integrator.

**Recommendation (software, cheap).**
1. **Fix D1.** Store `_integ` ki-weighted, as `AC_PID.cpp:345-346` does. Keep the parameter names; the meaning then matches ArduSub. Re-derive the autotune `LIMITS.ki`.
2. **Fix D5.** `mixer::mix()` returns per-axis `sat_pos` and `sat_neg`: when group g scales (maxabs > 1), every nonzero axis in g is flagged in its own sign. Pass the flags into `PID::update(..., lim_pos, lim_neg)`. About 20 lines, no measurable cost.
3. **Turns.** Replace bang-bang with `rate = sqrt_controller(err, ang_yaw_p, α_max)`, where α_max comes from a step.
4. **Turn battery compensation on**, with the thruster pack's V_max (the source is ≤2 Hz, which is fine at a 0.5 Hz LPF). ⚠ Not together with `THR_TRIM_EN` until D4 is fixed.

**Falsifiers.**
- D1 fix: on the bench, hold a constant error and read the I-term plateau. It should be 0.04 / 0.004 / 0.10 today and IMAX after the fix.
- D5 fix: a saturating yaw-plus-forward burst in water. Overshoot on release should fall. If it does not, the flags are not reaching the PID.
- Turn: overshoot on a 90° turn, bang-bang vs sqrt.
- Battery: a timed `move_forward` on a full pack vs one at 3.6 V/cell. Distance should differ by less than 10 %.

**Priority.** D1: highest. D5 and the turn: high. Battery: medium.

---

## 4. Thrust allocation on the MCU

**What we do.** The firmware mixes ±1 with a per-group uniform scale (`mixer.cpp:25-68`). The host has a geometric least-squares allocator with redistribution (`geometric_allocation.py`, and D6).

**The reference.** PX4 `ControlAllocationSequentialDesaturation`:
- pseudo-inverse, then desaturation along each axis's column in priority order;
- the gain is `k_min + k_max` over the violated actuators, applied once and then again at half (`:67-118`);
- actuators with effectiveness below 0.2 are skipped (`:96-99`);
- per-actuator `_actuator_min/max`, so limits can be asymmetric.

It is O(N·axes), with no QP.

**The key structural fact, which earlier dossiers missed.**
- **The 5-thruster hull has no redundancy.** Its actuated `B` is **5×5 and invertible**, and it splits into two near-independent blocks:
  - {lateral_a, lateral_b} → {sway, yaw}
  - {vertical_a, vertical_b, axial} → {heave, pitch, surge}. The axial unit couples into pitch only through its 8.1 mm z-offset.
- With no null space, "redistribution" can only choose **which axis loses**. A QP buys nothing that a closed-form priority clip does not.
- The **8×T200** frame has a **one-dimensional** null space per group in the ±1 idealisation:
  - horizontal: n = (1, 1, 1, 1)
  - vertical: n = (1, −1, −1, 1)

**Recommendation (both about 1 µs per tick, ⚠ estimated).**
- **5-thruster:** a closed-form lexicographic clip per 2×2 block. Take d = yaw/L and limits lo = −r·hi (r = 0.77 in thrust units).
  - Clip d so that |d|/2 ≤ min(hi, −lo).
  - Then clip sway into [2·lo + |d|, 2·hi − |d|].
  - Surge and heave get the same treatment in the vertical block.
  - This is **exactly** the lexicographic-QP optimum for a 2×2 block, in about 15 flops.
  - Output the unallocated wrench per axis. That feeds D5 and PR #20.
- **8×T200:** u = C·τ + λn. Each motor gives an interval for λ; intersect the four. If the intersection is non-empty, **allocation is exact**. This beats Durham's bound, which is about fixed inverses, and the chosen λ shifts load off the weak reverse direction (8 divides, 8 compares). If it is empty, scale the lowest-priority axis. That is a linear feasibility problem in (λ, α), solved by checking at most 8 vertices.
- **Deadband and minimum spin** belong **after** allocation, as the per-thruster output map (`oneToDshot`). The floor at `MOT_SPIN_MIN = 0.15` is already there. Allocation should see the true limit, `u_max(V) = lift_max(V)`, not ±1.
- **Host:** fix D6 by weighting residuals with **priority weights, not arms** (yaw ≫ sway), or reuse the same closed-form clip. Two copies of one law is the bug; generate both from one table.

**Falsifier.** Replay logged demands offline through all three allocators (current, clip, λ-interval) and compute the achieved wrench error. If the clip does not reduce the yaw error during saturating transits, keep the uniform scale.

**Priority.** High, software. The 5-thruster clip is 40 lines of C.

---

## 5. Thruster modelling and the ESC

**What we do.**
- `thstExpo` inverse-quadratic linearisation, e = 0.65, plus the `MOT_SPIN_MIN` floor (`mixer.cpp:110-127`).
- The same shaping in reverse. **No reverse-efficiency correction on the board** (the host has 0.77).
- An optional Pico closed-loop RPM PI at 2 kHz (`pico/main.cpp:241-270`), off by default (`config.h:224`).
- A slow RPM trim (`thrust_trim.cpp`), carrying D4.

**The reference.**
- **Bluejay has no closed-loop speed control.** It has only RPM-dependent PWM limits (`Bluejay.asm:246, 806`).
- **Its extended telemetry sends demag, status, debug1, debug2 and temperature, and no voltage or current** (`Modules/Scheduler.asm:66-80`). ✅
- AM32 has an in-ESC commutation-period speed PID (`main.c:1169-1171, 1464-1470`, gated by `drive_by_rpm`). Its extended telemetry sends **current every 40 frames at 1 A resolution**, and voltage in 0.25 V steps (`dshot.c:40-42, 256-261`).

**The gap.**
- ⛔ **G-13 and PR C ("detect a dead thruster on current") cannot work on Bluejay.** The Pico's `CURRENT` case (`pico/main.cpp:309-311`) is never fed. Even AM32's 1 A steps are coarse for low-thrust fouling detection (❓ a T200-class thruster draws only a few A at part throttle).
- Reverse thrust is about 23 % weaker for the same |n|, so every symmetric loop has an **asymmetric plant gain**.
- The RPM trim normalises RPM, not thrust, so it **cannot see** reverse efficiency.

**Recommendation.**
1. *Cheap, software:*
   - fix D4 with `rpm_tgt = sign·rpm_max·√|nrm|`, and use k_rev in reverse;
   - add a per-direction gain in `oneToDshot` (reverse ×1/0.77), with the allocator's limit lo = −0.77.
2. *Cheap, bench:* log eRPM at the Pico's 2 kHz for 0→50 % and 50→0 % steps, in a bucket, to get the **spin-up time constant**. The header claims 100–300 ms (`thrust_trim.h:18-20`), unmeasured. The literature's 0.59 s dead time was a T100 behind a ROS stack.
3. *Then:* re-test the RPM loop against open loop on those steps. If RPM-loop thrust settles faster with under 10 % overshoot, D4 was the oscillation.
4. *Hardware (custom thrusters):* per-thruster current sensing, as a shunt on the ESC or on a board channel, or an AM32-class ESC. The Pico should expose eRPM (it does), current at ≥10 Hz with ≤0.1 A resolution, temperature, and a `demand-vs-RPM residual` flag.
5. *Tunnel speed loss:* schedule the lateral column with surge speed, as b_lat(u) = b₀·max(0.1, 1 − c·u). Identify c from sway authority at three surge speeds. ⚠ c is unknown for ⌀84 mm tunnels.

**Priority.** D4 and the reverse gain: high and cheap. Current sensing: medium; it needs hardware.

---

## 6. System identification without a basin

**The existing autotune (`autotune.cpp`), critiqued.** It is a Ziegler–Nichols relay, done carefully: median period, consensus test, amplitude floor and ceiling, depth held during attitude phases. What remains:

1. **The relay has no hysteresis.** It switches on the raw sign (`:423, 437, 457`). Only the *measurement* has a Schmitt band. Gyro noise near zero therefore chatters the relay at up to 500 Hz; `MIN_HALF_MS = 120` hides this and does not fix it. The Åström–Hägglund fix is relay hysteresis ε with the corrected Ku = 4d/(π√(a² − ε²)). ⚠ Formula from memory, standard.
2. **Amplitude is the maximum |signal|** after settling (`:257`), not the mean half-cycle peak. A **static offset** biases it high and Ku low: buoyancy on depth, a CoB moment on pitch. Use a **biased relay** (auto-bias until the half-periods are equal) and a peak-to-peak/2 amplitude.
3. **The tuning rule is ZN "some overshoot"** (Kp = 0.33Ku, Ti = Tu/2, Td = Tu/3, `:185-187`). ZN is aggressive and poor for delay-dominated plants. SIMC with τc = θ gives **Ms ≈ 1.6–1.7 and a gain margin of about 3** (Skogestad). Thruster dead time makes these plants delay-dominated.
4. **It returns one Nyquist point, not a model.** INDI's effectiveness matrix and §7's feedforward need **gain and time constants**.

**Recommendation.**
- Keep the relay to find Tu. Add a step test per axis to fit a first-order-plus-dead-time model (k, τ, θ), and tune with SIMC.
- Add a **chirp-injection system-ID mode**, as ArduPilot does with `_actuator_sysid` added at the rate-controller output (`AC_AttitudeControl_Multi.cpp:472-478`). Log at 500 Hz to SD.
- In air, identify only the thrusters (§5). Air tells you nothing about added mass or damping.

**Falsifier.** Tune the same axis twice (ZN vs SIMC) and compare step overshoot and disturbance recovery. If SIMC is not at least as good on overshoot, keep ZN.

**Priority.** Medium. It needs water.

---

## 7. Hydrodynamic feedforward

**What we do.**
- ω|ω| "drag feedforward", on the measured rate (D3).
- A linear yaw→roll/pitch coupling term (`feedforward.cpp:190-191`).
- CoB trim (D2).
- All three are off by default.

**Critique against Fossen's model** (M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ):
- Feedforward must use the **reference**: τ_ff = (M_RB + M_A)·ω̇_ref + (D_l + D_q·|ω_ref|)·ω_ref. On measured signals it is feedback with an unanalysed sign (D3).
- The **linear damping term is missing**. At pool speeds it matters.
- **Restoring forces are missing.** Net buoyancy (W − B) is a constant heave term. The pitch moment is BG·W·sin θ. On depth, the buoyancy term is what D1 currently leaves uncompensated.
- The yaw-coupling term is physically a product (Coriolis and Munk terms, ∝ q·r and u·v), not linear in r.
- **On the 5-thruster hull roll is unactuated, so `XC_YAW2RLL` and `ATC_DRAG_RLL` must be refused on that frame, not tuned.**

**The minimal model worth identifying** (pool, no basin; methods in `system-id-no-basin.md`):

| term | how to get it |
|---|---|
| hover heave trim, as `DEPTH_FF` | one number, read from the settled depth integrator |
| BG | pitch free decay |
| yaw inertia + linear/quadratic damping (N_r, N_r\|r\|) | yaw free decay, or a chirp |
| surge damping | coast-down |

That is five numbers. **What it buys** (❓): less turn and ramp lag, and freed integrator headroom. No pool-scale number exists for "PID + feedforward vs PID" (`system-id-no-basin.md` §6 says so). This would be **our** measurement.

**Priority.** `DEPTH_FF`: high and cheap. The rest: medium, after §6.

---

## Ranked roadmap (top 10)

| # | move | kind | cost | falsifier |
|---|---|---|---|---|
| 1 | **Fix D1: ki-weighted integrator, IMAX in output units** | software, cheap | ~10 lines | bench I-term plateau 0.04 → 0.444 |
| 2 | **`DEPTH_FF` (buoyancy) + fix D5 (saturation flags into the PID)** | software, cheap | ~40 lines | depth steady-state error; overshoot after saturation |
| 3 | **Baro-inertial vertical filter + non-blocking OSR 4096 + depth cascade** (re-enable linear accel) | software | ~150 lines, <100 flops/tick | bench σ_w < 2 cm/s; pool heave-effort RMS ↓ ≥3× |
| 4 | **Fix D4 (√ RPM mapping, thrust_trim) + per-direction reverse gain** | software, cheap | ~20 lines | bucket step tests; RPM loop re-trial |
| 5 | **Closed-form prioritised allocator** (5-thruster 2×2 clip / 8×T200 λ-interval), asymmetric limits, unallocated wrench out; fix D6 on the host | software | ~150 lines, ~1 µs | offline replay of wrench error |
| 6 | **Fix D3** (feedforward from the reference) and **refuse roll terms** on the 5-thruster frame | software, cheap | ~15 lines | an analysis argument alone; no water needed |
| 7 | `sqrt_controller` turn shaping + enable battery compensation | software, cheap | ~30 lines + 1 param | 90° overshoot; full- vs low-pack distance |
| 8 | Autotune: relay hysteresis, biased relay, mean amplitude, SIMC from an FOPDT step, plus a chirp system-ID mode | software, **needs water** | ~200 lines | ZN vs SIMC A/B |
| 9 | Five-number Fossen feedforward from free decay and coast-down (fix D2 first) | software, **needs water** | pool session | turn and ramp lag, A/B |
| 10 | **Hardware:** per-thruster current (Bluejay cannot send it) and a separate ICM-42688-P on SPI for INDI | **hardware** | hardware team | fouled-prop detection; ω̇ noise |

Items 1, 2, 4 and 6 take one afternoon, no water, and no thrusters in the loop. They fix defects that would otherwise sit in the first closed depth hold and corrupt whatever it teaches us.

---

## Sources

**Local code**
- `srot-control-board` @ 22f0ccb: `src/control/{pid.h, attitude_control.cpp, depth_control.cpp, feedforward.cpp, mixer.cpp, movement.cpp, thrust_trim.{h,cpp}, autotune.cpp}`, `src/drivers/{bno085,bar30}.cpp`, `src/tasks/{task_sensor_read, task_dshot_rmt, task_control_loop}.cpp`, `src/pico/main.cpp`, `include/config.h`, `lib/MS5837`
- `mongla_ws`: `src/mongla_control/mongla_control/geometric_allocation.py` (D6 was measured by executing it)

**Reference implementations**
- PX4 @ e370d98: [ControlAllocationSequentialDesaturation.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp), [MulticopterRateControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_rate_control/MulticopterRateControl.cpp), [rate_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/rate_control/rate_control.cpp)
- ArduPilot @ ee0c343: [AC_PID.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PID/AC_PID.cpp), [AC_AttitudeControl_Sub.h](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h), [AP_Motors6DOF.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_Motors6DOF.cpp), [AP_Math/control.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Math/control.cpp), [AC_PosControl.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_AttitudeControl/AC_PosControl.cpp), [AC_AttitudeControl_Multi.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp)
- ArduPilot `AP_InertialNav` third-order complementary filter (older source, ⚠ snippet): [GaloisInc mirror](https://github.com/GaloisInc/ardupilot-mega/blob/master/libraries/AP_InertialNav/AP_InertialNav.h)
- Bluejay @ 0368d11: [bird-sanctuary/bluejay](https://github.com/bird-sanctuary/bluejay) (`src/Modules/Scheduler.asm`, `src/Bluejay.asm`)
- AM32 @ 8574381: [am32-firmware/AM32](https://github.com/am32-firmware/AM32) (`Src/main.c`, `Src/dshot.c`)

**Datasheets (⚠ search snippets; the direct PDF fetches were egress-blocked)**
- MS5837-30BA: [TE datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-30BA%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5837-30BA_B1.pdf%7FCAT-BLPS0017), [Mouser copy](https://www.mouser.com/datasheet/2/418/MS5837-30BA-736494.pdf)
- BNO08x: [CEVA BNO080/085 datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf), [Adafruit report types](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/report-types)
- ICM-42688-P: [TDK datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf)
- BMI088: [Bosch datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

**Papers**
- Sabatini & Genovese, *Sensors* 14(8):13324 (2014): [PMC4179067](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4179067/)
- Skogestad, SIMC rules: [README](https://skoge.folk.ntnu.no/publications/2003/tuningPID/README.html), [chapter](https://skoge.folk.ntnu.no/publications/2012/skogestad-improved-simc-pid/PIDbook-chapter5.pdf)
- Relay-feedback autotuning, tutorial review: [ResearchGate](https://www.researchgate.net/publication/222514888_Relay_feedback_auto-tuning_of_process_controllers_-_a_tutorial_review)
- Relay under large static disturbances: [ResearchGate](https://www.researchgate.net/publication/262271835_Technical_communique_Relay_feedback_method_under_large_static_disturbances)
- Asymmetric relay, multiple points: [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S000510982200615X)

**Carried from earlier dossiers, not re-fetched:** Johansen & Fossen 2013; Saunders & Nahon 2002; Sørensen/Smogeli 2009; arXiv 1807.04109 (0.59 s dead time); MDPI *Sensors* 22(2):488 (HOSMC); PMC6749442 (free decay).
