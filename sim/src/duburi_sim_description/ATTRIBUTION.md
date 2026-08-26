# Attribution

## bluerov2_gz

`scripts/generate_model.py`, `models/duburi_heavy/model.sdf.in` and the
`configs.yaml` schema derive from
[bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) by Clyde McQueen,
licensed MIT.

Changes made here:

- Renamed the model to `duburi_heavy` and parameterised the Thruster
  `<namespace>` and mesh URIs, which upstream hardcodes per model.
- Added front and bottom camera links, joints and sensors, sized to the
  `sim_front` / `sim_bottom` profiles that `duburi_ws` expects. Upstream ships
  no cameras at all.
- Added `gz-sim-odometry-publisher-system` for ground truth.
- Made the ArduPilot FDM endpoint configurable rather than hardcoded.
- Populated the added-mass and linear-damping Fossen coefficients, which
  upstream leaves at zero. See `models/duburi_heavy/HYDRODYNAMICS.md`.
- Set `fluid_density` to match the world's buoyancy `default_density`; upstream's
  base model is inconsistent between the two.
- Removed the ping-sonar code path, which double-counts the sensor mass upstream
  (bluerov2_gz issue #23) and is not used here.
- Generator now fails loudly on unknown template tokens instead of raising a
  bare `KeyError` from a dictionary lookup, and rounds to 6 decimal places
  rather than 3.

## Meshes

`meshes/duburi_heavy.dae`, `meshes/t200_ccw_prop.dae` and
`meshes/t200_cw_prop.dae` are from [Blue Robotics](https://bluerobotics.com/),
via bluerov2_gz:

- [BlueROV2 mesh](https://grabcad.com/library/bluerov2-1)
- [T200 propeller mesh](https://grabcad.com/library/bluerobotics-t200-thruster-1)

## Hydrodynamic coefficients

See `models/duburi_heavy/HYDRODYNAMICS.md` for per-coefficient sourcing.
