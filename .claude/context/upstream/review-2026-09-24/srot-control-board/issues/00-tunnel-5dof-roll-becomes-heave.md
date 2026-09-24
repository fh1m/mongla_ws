# tunnel_5dof: on the CAD hull, roll demand becomes heave and the roll trim-learner fights the depth loop. A frame change touches 8 sites beyond mixer.cpp

**Builds on PR #25**, which asks for a frame selector or a geometry-derived `M`. It is not a duplicate. It adds four things:
- proof that no motor wiring rescues the current matrix;
- the winding failure that roll causes on this hull;
- every code site a new frame has to touch, not just the matrix;
- the per-motor parameters the custom thruster needs.

Host counterpart: fh1m/mongla_ws#9.

## Summary
On the 5-thruster CAD hull, the fixed `vectored_6dof` matrix cannot separate surge, sway and yaw under **any** wiring. A roll demand becomes pure heave. The roll rate-PID integrator can never be satisfied, and the CoB trim learner then moves that integrator into `s_trim_roll`. That trim is a standing heave bias the depth loop has to fight, so two integrators end up winding against each other.

## Evidence (at `f1d3ba9`, measured from the Onshape GLB, 2026-09-24)
**Hull.** 176.1 × 172.1 × 702.0 mm. There is a lateral tunnel pair 350.0 mm apart, a vertical pair 519.2 mm apart, and one axial unit. All thrust lines are on the centreline. Only three motors are fitted in the CAD: the two vertical tunnel drives and the axial unit. The lateral tunnels are empty.

**B matrix.** Referenced at the bounding-box centre, because the CoM is unknown (no materials are assigned in Onshape):
- rank 5;
- condition number 5.7 raw, 1.49 with the moment rows scaled by the 0.2595 m arm.

Authority per unit thrust F of one thruster:

| Axis | Authority |
|---|---|
| surge | 1 F (the axial unit only) |
| sway | 2 F |
| heave | 2 F |
| pitch | 0.519 F·m |
| yaw | 0.35 F·m (weakest, so it saturates first) |

**Wiring search.** We tried all 9,216 assignments of the 5 units to the 8 outputs, times every `MOT_n_DIR` sign pattern. **None** separates surge, sway and yaw. This is structural: every horizontal row of `M` (`src/control/mixer.cpp:25-35`) carries yaw, forward and lateral at ±1 at once. With the natural wiring:
- a surge demand produces a full yaw couple;
- heading hold drives the axial unit continuously;
- a sway demand pushes the vehicle backwards;
- **a roll demand becomes 2 units of heave.**

**The integrator fight**, in `src/control/feedforward.cpp`:
```cpp
if (angle_hold && g_params.trim_en > 0.5f && learn) {
    ...
    attitude::bleedRateIntegral(0, shift(s_trim_roll,  attitude::rateIntegral(0)));
    ...
    if (depth_mode)
        depth::bleedIntegral(shift(s_trim_thr, depth::integral()));
}
roll     += s_trim_roll;      // on this hull the mixer turns this into heave
```
Two more roll contributions go the same way: `roll += atc_drag_rll * gx*|gx|` and `roll += xc_yaw2rll * gz`. The second injects roll, and therefore heave, in proportion to yaw rate.

## Failure scenario
1. On this hull, roll is only passively righted by BG.
2. A small roll error keeps the roll rate integrator charging (`attitude_control.cpp` ~106/122).
3. The trim learner moves the charge into `s_trim_roll`, up to `trim_max`.
4. The mixer turns that into heave, and the depth loop's integrator plus `s_trim_thr` counter it.
5. Result: a standing tug-of-war between two integrators. Heave authority is lost, and a depth offset follows whenever `trim_roll` saturates.
6. On top of that, every yaw rate command injects heave through `xc_yaw2rll`.

## Suggested fix: `FRAME_TYPE = tunnel_5dof`
Add this as a frame alongside `vectored_6dof`, or better, derive it from per-thruster position and axis parameters as PR #25 proposes. The sites it must touch:
1. **The matrix** at `mixer.cpp:25-35`. Drop the roll row and invert the 5×5 offline, about 25 multiply-adds per tick.
2. **Per-motor saturation group IDs** replacing the fixed `N_HORIZ` split at `mixer.cpp:41,53,66`. There are three groups: the lateral pair (yaw before sway), the vertical pair (pitch before heave), and the axial unit alone. Report each group's scale factor on the wire (PR #20).
3. **A reverse gain per motor**, applied in `oneToDshot` (`mixer.cpp:97-127`); the custom thruster's reverse ratio is unknown. **A minimum spin per motor**, instead of the single `MOT_SPIN_MIN` at `mixer.cpp:110` (relates to #13).
4. **`motorAngular`** (`mixer.cpp:142-145`) must report roll = 0 for this frame. Motor-detect uses it (`calibration.cpp:346`).
5. **The `MIX_VERT`/`MIX_VSGN` probe** at `mav_stream.cpp:957-964` hard-codes motors 5–8 as the verticals.
6. **Roll off:** force the roll output to 0 and freeze its integrator (`attitude_control.cpp` ~106/122). Skip the `s_trim_roll`, `atc_drag_rll` and `xc_yaw2rll` terms in `feedforward.cpp`.
7. **Refuse** the roll phases of autotune and `style_roll` / STUNT roll on this frame.
8. **Refuse to arm** when the frame type does not match the number of detected motors.

## How to verify
- A native build of `mixer.cpp` (the host already has this bench, see PR #25 §5): for each unit wrench demand, `B · mix(demand)` should equal the demand, for all five DOF, to within 1e-3.
- On the bench with the hull level, commanding roll produces zero output on every motor.
- With `trim_en = 1`, a held roll offset does not move `s_trim_thr`.

## Severity: High
This is not reachable on the 8-thruster competition vehicle. On the hull being built, it is the first thing to break once it is in the water.

## Also found (for the custom thruster and the hull design)
- The fitted prop in the CAD is **⌀37.2 mm in a ⌀78 mm bore**, a 20 mm tip gap. That is probably a placeholder, but the custom thruster should be sized to the bore.
- **Tunnel thrust falls with forward speed** (Saunders & Nahon, OCEANS'02; Palmer, Hearn & Stevenson 2008). A 5 N tunnel jet here is only about 1 m/s, so a 0.5 m/s transit is already in the loss region, and that affects the vertical pair's pitch and depth authority too. The allocator needs a measured, speed-dependent effectiveness factor, not a constant.
- **Design lever:** roll is tied to sway, with the roll row = −h × the sway row, where h is how far the CoM sits below the tunnel plane. Put the CoM on the tunnel plane and take BG from buoyancy placed high, not ballast placed low.
- **Trim the vehicle slightly positive**, so the vertical pair runs a steady 20–30 % push-down. That keeps both vertical motors above the minimum spin and makes pitch linear. The custom thruster then needs winding-temperature telemetry, which adds to what PR #26 asks for.

## Related
PR #25 (frame selector), PR #20 (scale factor), PR #26 (custom thruster needs), PR #28 (load cell), #13 (`MOT_SPIN_MIN`).

---
_Generated by [Claude Code](https://claude.ai/code)_
