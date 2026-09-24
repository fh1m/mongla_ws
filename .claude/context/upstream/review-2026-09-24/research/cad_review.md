# Thruster allocation vs the CAD hull: review (2026-09-24)

Read-only review. Nothing in any repo was changed. Hull = the **5-unit CAD hull** unless a line says otherwise. Every number below is **BUILT (geometry)**. None of it is measured in water.

## 1. What the GLB contains

- `tools/pack_hull.py` **cannot read this export.** It raises `KeyError: 'bufferView'` at `pack_hull.py:76`. The file declares `KHR_draco_mesh_compression` in `extensionsRequired`, and the loader has no Draco path. It only rejects quantised accessors. I decoded the file with DracoPy. There are 40 bodies, node transforms are identity, and the frame is glTF Y-up with Z as the long axis.
- **Overall size: 176.1 × 172.1 × 702.0 mm** (X −88…88, Y −97.1…75.0, Z −350…352). This matches CLAUDE.md. The bbox centre is (0, −11.05, +1.0) mm. The side skids (Parts 31/33, down to Y = −97.1) pull the box downward. The thrust axis is not at the bbox centre.

| unit | bodies | thrust line (glTF, mm) | fitted? |
|---|---|---|---|
| lateral fwd / aft | Part 1 / Part 11 tubes: ⌀84 OD, **⌀78.0 bore** (r = 39.0), 148.3 mm long (L/D 1.9) | axis X, (0, 0, ±175.0) | **Tunnel only. No motor, prop or spider.** |
| vertical fwd / aft | Part 2 / Part 12: ⌀84/⌀78, 124.8 mm long (L/D 1.6) | axis Y, (0, 0, ±259.5) | A2212 + prop (Part 1, ⌀37.2) + ⌀76.4 spider (Parts 10/17) |
| axial | A2212 (node 14) + prop ⌀37.2 at z = 330.9, ⌀88 grille (Part 3) at z = 350–352 | axis Z, prop hub at **(0, +0.12, 330.9)** | fitted |

- Pair separations are **350.0 / 519.2 mm**. They agree with the CLAUDE.md figures and with `hull_geometry.yaml`.
- **Missing: both lateral-tunnel thrusters.** Only three A2212s are modelled.
- **Finding, `hull_geometry.yaml` / `geometric_allocation.py:57`:** the axial unit is stored at (0, −0.3306, **0.0081**). That 8.1 mm offset is the A2212's bounding-box centre, which is biased by its mount bracket. The GLB puts the thrust line at +0.12 mm: prop hub centre, and motor centre of mass +0.5 mm. The stored offset creates a phantom pitch moment of 0.0081 N·m per N of surge. The least-squares solve then cancels it with a ±1.6 % vertical differential that the real hull does not need. Use the prop hub, not the bbox.
- **Finding, prop:** the fitted prop is **⌀37.2 mm in a ⌀78 bore**, leaving a 20.4 mm tip gap (D_prop/D_bore = 0.48). If that prop is flown, the tunnel adds nothing and thrust is that of an open ⌀37 prop. A ducted or tunnel prop wants a tip gap of about 1–2 % of D. This is probably a placeholder, but the custom thruster should be sized to the bore.
- **Which end is the bow is not settled by geometry.** A ⌀65 cap with a transparent-tinted ring (Parts 39–42) sits at −Z. The axial unit and a grille sit at +Z. CLAUDE.md calls +Z the nose. Nothing below depends on this except the sign of surge.

Uniform-density solid centroid is (0, −1.6, −3.2) mm; this is not the centre of mass (CoM). Convex-envelope centroid is (0, −9.4, 0). Side-view area is 797 cm², with its centroid **3.6 mm below** the tunnel axis. Frontal area is 225 cm², centroid 1.5 mm below.

## 2. B for the CAD hull

Body frame: x = +Z_glTF (toward the axial unit), y = −X, z = −Y (down). Reference = **bbox centre, used as a stand-in for the CoM, which is unknown.** In this frame the thrust plane is **11.05 mm above** the reference. Columns are [d; r × d], with moments in N·m per N.

```
            lat_fwd  lat_aft  vert_fwd vert_aft  axial
X surge      0.0000   0.0000   0.0000   0.0000   1.0000
Y sway       1.0000   1.0000   0.0000   0.0000   0.0000
Z heave      0.0000   0.0000   1.0000   1.0000   0.0000
K roll       0.0111   0.0111   0.0000   0.0000   0.0000
M pitch      0.0000   0.0000  -0.2586   0.2606  -0.0112
N yaw        0.1740  -0.1760   0.0000   0.0000   0.0000
```

- **Rank 5.** Singular values are 1.414, 1.414, 1.000, 0.367, 0.248. The 5×5 matrix with the roll row dropped has condition number 5.71 raw, and **1.49** with the moment rows divided by 0.2595 m. The matrix is well conditioned and exactly square, so the pseudo-inverse is simply an inverse.
- **Roll is not a free axis. It is tied to sway.** The roll row equals −h × the sway row, where h is how far the CoM sits below the tunnel axis. The rank stays 5, but **pure sway is unattainable**: every newton of sway carries h N·m of roll, and only the gravity–buoyancy righting moment resists it. That moment depends on BG, the vertical distance between the centre of buoyancy and the CoM. At the bbox reference h = 11 mm. A CoM placed low for roll stability makes h larger.
- The other couplings can be actuated and are small:
  - Surge → pitch, h N·m per N. At h = 11 mm the vertical pair needs a ±2.15 % differential, which is inside the 15 % MOT_SPIN_MIN floor (§3).
  - Sway → yaw appears only if the CoM station x_g ≠ 0. Pure sway needs u_f/u_a = (0.175 + x_g)/(0.175 − x_g), so 1 cm of CoM error gives 0.01 N·m per N of sway.
- **Authority per unit thrust F:**

  | axis | authority |
  |---|---|
  | surge | 1 F (one unit, and it is asymmetric in reverse) |
  | sway | 2 F |
  | heave | 2 F |
  | yaw | **0.35 F·m** |
  | pitch | 0.519 F·m |

  Yaw is the weakest rotational axis, even though the body is axisymmetric so yaw and pitch inertia are equal. `pinv` needs 2.86 units of lateral output per N·m of yaw.

**Roll tilt from sway.** φ = asin(F·h / (W·BG)), with W ≈ 100 N (10.2 kg estimate from `hydrodynamics.py`):

| sway force | h | BG | tilt |
|---|---|---|---|
| 5 N | 11 mm | 10 mm | 3.2° |
| 5 N | 11 mm | 5 mm | 6.3° |
| 10 N | 20 mm | 5 mm | 23.6° |

This applies while the vehicle is accelerating. At steady sway speed the couple is only F × 3.6 mm, the drag-centre offset. **Design lever:** put the CoM on the tunnel plane (h → 0). Get BG from buoyancy high up (foam on top), not from ballast low down.

## 3. The firmware mixer on this hull (builds on PR #25)

The mixer is `mixer.cpp:25-35`. I checked every injective assignment of the 5 units to the 8 outputs, with every MOT_n_DIRECTION sign pattern: 9,216 combinations. **None decouples surge, sway and yaw.** The reason is structural: every horizontal row has |yaw| = |fwd| = |lat| = 1, so whichever horizontal output drives the axial unit also receives yaw and sway, and both lateral tunnels receive surge.

Natural mapping: M1→lat_fwd, M3→lat_aft, M4→axial, M5→vert_fwd, M7→vert_aft.

| demand = 1 | wrench on the CAD hull [X Y Z K M N] |
|---|---|
| fwd | [1, 0, 0, 0, −0.011, **−0.35**]: full yaw couple on every surge |
| yaw | [**1**, 0, 0, 0, −0.011, 0.35]: heading hold drives the axial unit continuously |
| lat | [**−1**, 2, 0, 0.022, 0.011, 0]: sway backs the vehicle up |
| thr | [0, 0, −2, 0, 0, 0]: correct |
| pitch | [0, 0, 0, 0, 0.519, 0]: correct |
| **roll** | [0, 0, **2**, 0, 0, 0]: **roll demand becomes heave** |

The best mapping found only moves the damage around: roll becomes pitch and pitch becomes heave.

Roll is the dangerous one:
- The roll rate PID in `attitude_control.cpp:106/122` can never reduce its error on this hull, so its integrator winds up to `rat_rll_imax`.
- `feedforward.cpp:79,84` then transfers that into `s_trim_roll`.
- In the natural mapping that becomes a standing heave bias, which the depth integrator fights. Two integrators end up winding against each other.
- `xc_yaw2rll` and `atc_drag_rll` (`feedforward.cpp:50,53`) inject yaw-rate-driven heave the same way.

On the host side, **`allocation.py` `VERTICAL_PRIORITY = ('roll', 'pitch', 'throttle')` puts roll first.** On this hull that ranks an impossible axis above depth. `style_roll` and the autotune roll phases (`autotune.cpp:102-110`) must be refused on this frame.

## 4. Tunnel physics that matter for the custom thruster

- **Speed loss.** Side force drops as vehicle speed rises relative to jet speed (U/U_jet).
  - Saunders & Nahon (C-SCOUT, OCEANS'02) measured it with an internal 6-axis load cell. The thruster's dynamic response did not change with operating mode; steady-state thrust did.
  - Palmer, Hearn & Stevenson (Southampton / Delphin, 2008) modelled it, and a follow-up in *Ocean Systems Engineering* covers speed and drift angle.
  - The US patents already in `control.md §2.5` quote about 10 % of side force at 3 kn.
  - I could not reach the full texts from this network, so no percentages from them are quoted here.

  Our scale makes this worse. U_jet ≈ √(T/(ρA)); in the ⌀78 bore, **T = 5 N gives U_jet ≈ 1.0 m/s**. At 0.5 m/s transit, U/U_jet ≈ 0.5, well inside the loss region. **This also hits the vertical pair**, so pitch and depth authority fade in transit. That is exactly when the body's destabilising Munk moment grows. The allocator needs B_lat and B_vert scaled by an η(u) < 1 that is **measured, not assumed**.
- **Pitch arm 259.5 mm vs yaw arm 175 mm.** Pitch has 1.48× the yaw authority, plus BG stiffness. Yaw saturates first, so give yaw the higher priority.
- **Tunnel geometry.** L/D is 1.6–1.9 and both bores are symmetric, so a symmetric prop is natural here. A reverse-efficiency term is only needed on the axial unit, if it uses a forward-optimised prop.
- **Roll stays passive.** BG cannot be stated because the CAD has no materials; the float test proposed in `hull_geometry.yaml` settles it. The allocator must give the roll row **zero weight** (don't-care). It must never accept a roll demand into any saturation group, and any roll residual goes to telemetry only.

## 5. Recommended allocator

1. **Explicit B from geometry**, with the reference point at a CoM parameter (x_g, z_g), defaulting to the tunnel axis. Drop the roll row, invert the 5×5 offline, and fold in the η(u) and k± scalars at runtime. That costs 25 multiply-adds per tick, and an exact inverse needs no QP.
2. **Three saturation groups, not two:** {lat_fwd, lat_aft}, {vert_fwd, vert_aft} and {axial}. They share no axis once roll is gone.
   - Lateral priority: **yaw > sway**. Scale sway to the headroom left after yaw.
   - Vertical priority: **buoyancy trim > pitch > heave command**.
   - Axial: clip only.
   - Report each group's scale (PR #20).
3. **Reverse asymmetry:** after solving for force f_i, set u_i = f_i / k_i^sign(f_i). This is exact because each unit owns its column. Add a per-motor MOT_n_REV_GAIN, applied before shaping in `oneToDshot` (`mixer.cpp:97-127`).
4. **Deadband:** trim ballast slightly positive so the vertical pair runs at a steady 20–30 % push-down. Both motors then stay above MOT_SPIN_MIN, and pitch differential stays linear with no sign crossings. This absorbs the ±2 % surge→pitch compensation that would otherwise vanish into the floor. MOT_SPIN_MIN should become **per motor**; it is scalar today at `mixer.cpp:110`. The lateral pair cannot be biased, so it needs the smooth-from-zero thruster asked for in PR #26.
5. **Firmware changes for a `FRAME_TYPE` parameter, value `tunnel_5dof`.** NUM_THRUSTERS = 8 at `config.h:117` can stay as the output count.
   - `mixer.cpp:25-35`: add a second table, or build it from MOT_n_POS/AXIS parameters (PR #25 item 2). Unused outputs get zero rows, and the roll column is zero.
   - `mixer.cpp:41,53,66`: replace `N_HORIZ` with a per-motor group id, and implement the priority desaturation above instead of uniform scaling.
   - `mixer.cpp:142-145` `motorAngular`: expected roll = 0, so motor-detect (`calibration.cpp:346`) stops expecting roll.
   - `mav_stream.cpp:957-964`: the vertical probe assumes motors 4–7 are the vertical group.
   - `attitude_control.cpp:106,122`: force `out_roll = 0` and freeze the roll integrator when roll is unactuated.
   - `feedforward.cpp:50,53,79,84`: skip the roll terms.
   - Autotune and stunt: refuse the roll phases.
   - `arming`: refuse to arm if FRAME_TYPE does not match the connected motor count. PR-F gives the presence data for this.
6. **Host.** Remove roll from `VERTICAL_PRIORITY`. Fix the axial z-offset in `hull_geometry.yaml` and `geometric_allocation.py:57`. Make `pack_hull.py` decode Draco, or refuse such a file with a clear message.

**What the custom thruster should expose beyond PR #26:**
- **Per-unit η(u) data:** thrust at a set of transit speeds, so the lateral and vertical columns can be scheduled.
- **Bore-matched prop diameter and tip gap stated in the CAD.**
- **Direction-sign self-test:** a short burst with the IMU-response check, now that roll is absent.
- **Winding / ESC temperature**, which matters for the vertical pair if it runs a continuous bias duty.

## 6. What could not be determined

- **Unknown:** mass, CoM (x_g, z_g), CoB and therefore BG (no materials in the CAD); drag coefficients; thrust per unit (k); η(u); reverse ratio of the custom prop; which end is the bow; lateral-tunnel motor placement.
- **Everything above is geometry (BUILT).** The roll-tilt and jet-speed figures are parametric illustrations, not measurements.
- **Nothing on this hull has been in water.**
