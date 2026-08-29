# Hydrodynamic validation results

Measured by `duburi_sim_worlds/scripts/drag_survey.py` against
`/model/duburi/odometry` ground truth. A coefficient change should be diffed
against these rather than re-derived.

```bash
ros2 launch duburi_sim_bringup sim.launch.py course:=pool_empty \
    ardusub:=false bridge:=false gui:=false headless:=true t200:=false
python3 drag_survey.py --axes surge,sway --settle 14
python3 drag_survey.py --axes heave --settle 1.4 --thrust 12
```

**Two things will silently ruin a run and both are now guarded:**

- **`t200:=false` is required.** The T200 curve node publishes to the same
  `cmd_thrust` topic the test rig commands, so with it running the rig is
  fighting a second publisher and reads roughly half the thrust it asked for.
- **The pool has 12.5 m of runway.** A 25 s run at 0.66 m/s covers 16.5 m: the
  vehicle hits the far wall, stops dead, and the tail average returns a number
  that is neither terminal velocity nor an obvious error. `step_response.py`'s
  own 25 s default does exactly this, which is why its recorded 1.6 % result
  could not be reproduced until the settle time was cut. `drag_survey.py`
  refuses a run that would outrun the pool and reports distance travelled.

## 2026-08-29, 25 N per thruster (12 N for heave)

| Axis | Force | Predicted | Measured | Error | τ |
|---|---|---|---|---|---|
| surge | 70.7 N | 0.661 m/s | **0.658 m/s** | 0.5 % | 1.0 s |
| sway | 70.7 N | 0.571 m/s | **0.563 m/s** | 1.3 % | 1.0 s |
| heave | 48.0 N | 0.423 m/s | **0.395 m/s** | 6.6 % | 1.0 s |

**Sway and heave had never been measured before this.** The only prior
validation of the entire hydrodynamic model was a single surge number.

Heave's 6.6 % is the largest error and the least surprising: it is the axis with
the most added mass (18.7 kg against a 13.5 kg hull, a ratio of 1.38), it runs
against the vehicle's deliberate 100 g of positive buoyancy, and it has the
least runway — 1.6 m of pool depth against 25 m of length, so the measurement
window is short enough that it may not be fully converged.

## Still unmeasured

- **Rotational axes.** `HYDRODYNAMICS.md` states the rotational added-mass terms
  carry **30–100 % estimation error** because the vehicle violates the
  estimating method's assumption that length exceeds width — the Heavy is 20 %
  wider than long. These are the numbers most likely to be wrong and the hardest
  to measure: a free vehicle fired on one thruster rotates far more than it
  translates, and body-axis readings decouple within a second.
- **Settling time as an added-mass check.** τ is reported but is currently
  sample-rate limited at 1.0 s across every axis, which is the sampling
  interval rather than a measurement. Sampling faster than the physics would be
  needed to make it meaningful.

## The fidelity limit

The sim vehicle is a **BlueROV2 Heavy, not the Duburi 4.5 hull**
(`model.sdf.in` line 12). These numbers validate that the simulation is
self-consistent — the drag it applies matches the drag it was configured with.
They do **not** validate that either matches the real Duburi.
