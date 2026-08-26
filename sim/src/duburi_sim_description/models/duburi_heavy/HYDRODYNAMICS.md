# Hydrodynamic coefficients for `duburi_heavy`

Where the numbers in `configs.yaml` come from, what they mean, and why the
damping terms are negative while the added-mass terms are positive. Read this
before changing any of them.

## Summary

| | |
|---|---|
| Source | ROV-Simulator benchmark, identified BlueROV2 Heavy model |
| Paper | *An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot*, JMSE 10(12) 1898, 2022, Table A1 |
| Method | Eidsvik / DNV empirical estimation, not direct measurement |
| Vehicle | BlueROV2 **Heavy**, 13.5 kg, 8 thrusters |
| Convention in source | `M_A` and `D` positive-definite |
| Added mass in the SDF | `//inertial/fluid_added_mass`, **positive** |
| Damping in the SDF | `Hydrodynamics` plugin, **negated** |

## Two corrections to what the references claim

Both of these were assumed when this work started and both are wrong. They are
recorded here so nobody re-derives from the wrong place.

**Wu (2018) is not the source of these values.** The Wu thesis (*6-DoF Modelling
and Control of a Remotely Operated Vehicle*, Flinders University) is widely cited
for BlueROV2 parameters, but it models the **11.5 kg base BlueROV2**, with added
mass `5.5, 12.7, 14.57, 0.12, 0.12, 0.12` and linear damping
`4.03, 6.22, 5.18, …`. Those are different numbers for a different vehicle. The
values used here come from the JMSE 2022 paper's empirical estimation.

**`bluerov2_gz` credits ROV-Simulator but does not use its values.** Its
`BlueROV2Heavy.md` cites the benchmark, yet its generated SDF has zero added
mass, zero linear damping, and quadratic terms
(`-58.42 / -55.137 / -124.818 / -4 / -4 / -4`) that appear nowhere in the
benchmark — they are the generator's flat-plate fallback, derived from the
bounding box. The attribution is aspirational rather than literal.

## The coefficients

Published as positive magnitudes. `configs.yaml` stores the added mass exactly
as published and the damping negated, for the reasons in the two sections below.

| DoF | Added mass | Unit | Linear damping | Unit | Quadratic damping | Unit |
|---|---|---|---|---|---|---|
| Surge | X_u̇ = 6.356674 | kg | X_u = 13.7 | N·s/m | X_u\|u\| = 141.0 | N·s²/m² |
| Sway | Y_v̇ = 7.120600 | kg | Y_v = 0 | N·s/m | Y_v\|v\| = 217.0 | N·s²/m² |
| Heave | Z_ẇ = 18.686327 | kg | Z_w = 33.0 | N·s/m | Z_w\|w\| = 190.0 | N·s²/m² |
| Roll | K_ṗ = 0.185766 | kg·m² | K_p = 0 | N·m·s/rad | K_p\|p\| = 1.192 | N·m·s²/rad² |
| Pitch | M_q̇ = 0.134823 | kg·m² | M_q = 0.8 | N·m·s/rad | M_q\|q\| = 0.47 | N·m·s²/rad² |
| Yaw | N_ṙ = 0.221510 | kg·m² | N_r = 0 | N·m·s/rad | N_r\|r\| = 1.5 | N·m·s²/rad² |

Rigid body, from the same model:

```
m  = 13.5 kg        V = 0.0135 m^3   (exactly neutral in fresh water)
Ix = 0.26   Iy = 0.23   Iz = 0.37  kg*m^2
CoG = (0, 0, 0)     CoB = (0, 0, +0.01) m in Gazebo's z-up body frame
L = 0.457   W = 0.575   H = 0.378 m
```

### Caveats

- **`Y_v`, `K_p` and `N_r` are genuinely zero.** Sway, roll and yaw are modelled
  as purely quadratically damped. Do not "fix" them.
- **The rotational added-mass terms are soft.** The source states the estimation
  method carries 10–20% error translationally but **30–100% rotationally**, and
  warns that the Heavy violates one of the method's assumptions: its width
  exceeds its length by 20%. `K_ṗ`, `M_q̇` and `N_ṙ` are the first things to
  re-tune against real vehicle data.
- The paper's Table A1 prints `Ns` and `Ns²/m²` for the rotational rows. That is
  a typo in the publication; the units in the table above are the correct ones.

## Why the added mass is not in the Hydrodynamics plugin

The plugin has `<xDotU>`-style tags for added mass and they do not work for this
vehicle. It applies added mass **explicitly**, as `M_A·a` where `a` is a finite
difference of the previous step's velocity. That is a feedback loop with gain
`M_A / m` per axis, and it is unstable whenever the gain exceeds one. Heave adds
18.7 kg to a 13.5 kg vehicle, a gain of 1.38, so the simulation diverges within
about a second:

```
ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact
  && aabbBound < dMaxIntExact" failed in collide()
```

That message is ODE reporting a pose that has already gone to infinity, several
layers below the actual cause, and the process aborts rather than warning.

Added mass is therefore declared as `//inertial/fluid_added_mass` on `base_link`
instead. SDFormat ≥ 1.10 with the dartsim engine folds that matrix into the
link's spatial inertia, so it is solved implicitly and is unconditionally stable.
DART derives the added-mass Coriolis terms from the same inertia, so nothing is
lost. Three consequences:

- **The model SDF must declare version 1.10 or newer.** Ours says 1.11.
  `fluid_added_mass` in an older document is silently dropped by the version
  converter, taking the added mass with it and leaving no warning.
- **Those values are positive.** It is a real inertia matrix and has to be
  positive semi-definite. `generate_model.py` rejects negative entries.
- **Do not set both.** The plugin's tags are left at their zero default.
  Populating both double-counts.

## Why the damping values are negative

Gazebo's `Hydrodynamics` system reads the SDF tags into its coefficient matrices
**verbatim**, with no sign change:

```cpp
// gz-sim8 src/systems/hydrodynamics/Hydrodynamics.cc
this->dataPtr->Ma(i, j) = SdfParamDouble(_sdf, prefix, 0);
```

It then negates twice — once when forming the wrench, once when applying it:

```cpp
kTotalWrench += Dmat * state;
...
math::Vector3d totalForce(-kTotalWrench(0), -kTotalWrench(1), -kTotalWrench(2));
```

The two negations cancel, so the force actually applied is
`X = xU·u + xUabsU·|u|·u`, which only opposes motion if both tags are negative.

Gazebo states this itself, in `HydrodynamicsUtils.hh`:

> The Fossen sign convention is that diagonal elements of `_Ma` are negative
> (e.g. `X_u_dot < 0`).

**A positive value here is not a small error.** It flips the sign of the drag
force, and the vehicle accelerates itself without bound.

## Sanity check

Four vectored horizontal thrusters at 45°, roughly 50 N each, give about 141 N
of forward thrust. Terminal surge velocity solves `141 = X_u·u + X_u|u|·u²`:

| Coefficient set | Terminal surge |
|---|---|
| These values | **0.95 m/s** |
| `bluerov2_gz` flat-plate fallback | 1.56 m/s |
| Blue Robotics published spec | ~1.0 m/s |

This is an independent check: the published top speed was not used to derive
anything above, and the identified model reproduces it to within 5%.

## Verifying it in the simulator

Two scripts in `duburi_sim_worlds/scripts` check the model against the
simulator rather than against itself. Both need the empty pool with ArduSub out
of the way, so nothing else is commanding the thrusters:

```bash
ros2 launch duburi_sim_bringup sim.launch.py \
    course:=pool_empty ardusub:=false bridge:=false gui:=false
```

`step_response.py` drives 25 N through the forward mix and compares the terminal
surge with the analytic solution of `F = |X_u|·u + |X_u|u||·u²` taken from
`configs.yaml`. Last measured **0.650 m/s against 0.661 m/s predicted, 1.6%**.
A sign error in the damping shows up here as unbounded acceleration, which the
script names explicitly rather than reporting as a large error.

`thruster_survey.py` pulses each thruster alone and checks the sign of every
entry in the resulting allocation matrix against ArduSub's `vectored_6dof`
table. This catches the failure mode where the vehicle still flies but a forward
command yaws, which otherwise surfaces much later as an autonomy bug.

Prefer `<xUabsU>` over the older `<xUU>` spelling; Gazebo emits an
instability warning for the latter
([gz-sim#1888](https://github.com/gazebosim/gz-sim/pull/1888)).
