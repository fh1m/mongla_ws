# Dubomini 2.0 in the simulator — the real hull

The sim flew a BlueROV2 Heavy for its whole life. Recolouring it could never fix
that, and two rounds spent trying is what made the point. This is the actual
vehicle, from the team's own CAD.

## Provenance, and the check that it is the right vehicle

`hullv3.stl` — 2,149,667 triangles, 3,276 components, **Y-up, millimetres**.
Cross-checked against the published spec at `bracuduburi.com/auv/dubomini`
**before** anything was built on it:

| | published | STL |
|---|---|---|
| length | 545.9 mm | **545.9** — 0.0 mm |
| width | 464.3 mm | 470.2 |
| height | 166.8 mm | 169.3 |

A length agreeing to **0.0 mm** is what says this mesh is that vehicle.

## What is measured, derived, and assumed

Read this before trusting any number in `configs.yaml`.

- **MEASURED (published):** mass **14.6 kg** in air, overall dimensions,
  Aluminium 5083 welded frame, 8× T200 in a 6-DOF vectored arrangement.
- **DERIVED FROM THE CAD:** every thruster position, and the axis *line* of each.
- **SOLVED:** the added-mass matrix, by BEM on the hull (below).
- **ASSUMED:** buoyancy, drag, inertia. Each is marked in the config.

## Buoyancy cannot come from the mesh

The STL's signed volume is **6.58 L**, which would displace 6.58 kg against a
14.6 kg vehicle — **8.0 kg negative, a rock**. The enclosures are modelled as
*shells*, so the sealed air that actually floats this vehicle is not in the mesh
volume at all. The convex hull is **31.8 L**, so the truth lies between those
bounds and ~14.6 L (neutral) is 46 % of the envelope — reasonable for an open
frame, but reasonable is not measured.

`buoyancy_adjustment: 0.2` is the usual AUV trim and is **ASSUMED**. It is the
single most valuable number missing from this model; replace it when somebody
weighs the vehicle in water.

## Thrusters — position from CAD, direction from the frame

The published spec says only "vectored arrangement with empirically distributed
placement" and gives **no coordinates**. They are recovered from the geometry:
the eight identical ~43 k-face components with a ~97 mm envelope are the T200
ducts, and each thrust axis is that cylinder's *short* principal axis.

The check that this reads real features rather than fitting noise: the four
horizontals come out at **exactly ±45.0°**, the vectored-X layout, with nothing
rounded to get there. Against Duburi 4.5 the horizontals sit much further out in
x — **0.228 m against 0.14 m** — which is what "agile" means for this hull.

### The distinction that cost a debugging pass

**A mesh gives an axis LINE. It does not give a push DIRECTION.** A duct is a
cylinder; which way it pushes along its own axis is set by prop handedness and
ESC wiring, and neither is in the geometry. −45° and +135° are the *same line*.

Taking the CAD reading literally gave yaws of +135/+45/+45/+135, and ArduSub's
measured forward mix — `[−33, −27, +33, +27]` across thrusters 1–4 — sums
against those to **(0.0, −8.5) N: zero forward thrust and a sideways nudge**.
The vehicle drifted diagonally at 0.045 m/s and *nothing logged a fault*,
because nothing was faulty — every thruster did exactly as it was told.

With the ArduSub frame's own convention (−45/−135/+45/+135) the same mix sums to
**(84.9, 0.0) N**. Measured before and after:

```
CAD signs taken literally   0.268 m in 6 s   dx +0.205  dy +0.172   diagonal
frame signs                 0.958 m in 6 s   dx +0.958  dy +0.010   straight
```

Positions stay from the CAD and are real; only the four **signs** come from the
frame definition, which is a wiring fact rather than a geometric one.

## Added mass — solved on the hull, and it is an upper bound

`scripts/added_mass.py` runs Capytaine's BEM at infinite frequency with the free
surface removed — the standard treatment for a deeply submerged ROV.

| dof | added mass |
|---|---|
| surge | 8.57 kg |
| sway | 10.81 kg |
| **heave** | **64.51 kg** |
| roll | 0.294 kg·m² |
| pitch | 0.619 kg·m² |
| yaw | 0.072 kg·m² |

**Heave is 4.4× the vehicle's own mass**, where Duburi's is 1.4×. That is not an
error: Dubomini is a flat plate, and a flat body accelerating perpendicular to
its face drags an enormous amount of water. A disc of this radius gives 42 kg
from the closed form `8/3·ρ·a³`, so 64.5 for a squarer planform is the right
order.

**It runs on the CONVEX HULL, which is forced rather than chosen** — a BEM panel
method needs a closed surface and the assembly is 3,276 disconnected components,
not water-tight. A convex envelope displaces more water than an open frame, so
**these coefficients are an UPPER BOUND** and the modelled vehicle accelerates
more sluggishly than the real one.

**Two traps in getting this out of Capytaine 3.0:** `assemble_dataset` *silently
drops* every case without a free surface (their issue #88) — which is exactly
the submerged case — and returns a dataset with no `added_mass` variable at all,
reading like a failed solve when the solve succeeded. Read the result objects
directly. And SDF's `//inertial/fluid_added_mass` takes **positive** magnitudes
while the Hydrodynamics plugin's `<xDotU>` family is negative.

## What flies, and what does not yet

```
move_forward  8 s @ gain 60  ->  1.284 m   0.160 m/s   dx +1.262  dy +0.238
move_forward 16 s @ gain 90  ->  0.287 m               dz -0.674  DOVE
```

Forward motion is correct and straight. **Sustained high-gain runs lose depth**:
ALT_HOLD is fighting 64.5 kg of heave added mass, and ArduSub's depth controller
is tuned for a vehicle with far less. The upper-bound hull is the prime suspect.

**Speed is not calibrated.** 0.16 m/s against Duburi's ~0.65 m/s, and the drag
coefficients are still the BlueROV2's — inherited and marked ASSUMED. Nothing
here has been fitted to a measured Dubomini speed because no measured speed is
published.

## Still to do

1. **Measured buoyancy**, which unblocks the trim.
2. **Drag identified for this hull**, which unblocks speed.
3. **Depth-hold behaviour** against the real heave added mass.
4. **Inertia from a water-tight CAD export** — gz-sim 8 computes it from the
   mesh, and `hullv3.stl` is not water-tight, so the values are scaled from
   Duburi and marked ASSUMED.
