# DVL and sonar in the simulator

**Status: DVL is BUILT and working (2026-08-28). Sonar is not, and cannot be
without a custom plugin.**

## Why this exists

`move_forward_dist` and its siblings close a position loop on DVL bottom-track.
The sim had no DVL, so they silently fell back to dead reckoning at a hardcoded
0.3 m/s. Measured against `/duburi/sim/ground_truth`, a 1.0 m command drove
**2.361 m** and reported `OK … completed`. Adding a DVL is what makes those
verbs testable at all.

---

## DVL — native, shipped with Gazebo Harmonic

`gz-sim 8.15` includes a real DVL sensor. **No Project DAVE, no
`uuv_simulator`, no custom physics plugin.**

| piece | where |
|---|---|
| sensor implementation | `libgz-sensors8-dvl.so` |
| world system plugin | `libgz-sim8-dvl-system.so` (`gz::sim::systems::DopplerVelocityLogSystem`) |
| messages | `gz.msgs.DVLVelocityTracking` (+ beam / target / range / kinematic sub-messages) |
| reference world | `/usr/share/gz/gz-sim8/worlds/dvl_world.sdf` |

### What we built

| # | Work | File |
|---|---|---|
| 1 | 4-beam janus array, 8 Hz, ±0.002 m/s noise | `duburi_sim_description/models/duburi_heavy/model.sdf.in` |
| 2 | `DopplerVelocityLogSystem` in every world | `duburi_sim_worlds/templates/world.sdf.template` |
| 3 | gz→ROS republisher | `duburi_sim_bridge/duburi_sim_bridge/dvl_bridge.py` |
| 4 | `yaw_source=sim_dvl` | `duburi_sensors/sources/sim_dvl.py` + `factory.py` |

`model.sdf` and the `.world` files are **generated** — edit the `.in` /
`.template` and run `generate_model.py` / `gen_world.py --all`.

### Four traps, each of which looked like a broken sensor

Every one of these produced a plausible-looking DVL that was quietly wrong.
They are written down because none of them raises anything.

1. **The sensor must sit on `base_link`.** On its own child link it reports
   bottom lock, a plausible range, and a velocity of ~0.007 m/s while Gazebo's
   own odometry reads 0.698. gz-sim only populates velocity components for links
   something enabled them on. The reference world's `element_id="base_link"` is
   *placement*, not just include-override syntax.
2. **The DVL frame is not the body frame.** `<reference_frame>` carries a −90°
   z rotation, so a naive `.x` read is ~zero. Measured at ~0.68 m/s:

   | motion | DVL reads |
   |---|---|
   | forward (+x body) | `y = −0.687` |
   | back (−x body) | `y = +0.685` |
   | starboard | `x = −0.678` |

   Hence **`forward = −dvl.y`**, **`starboard = −dvl.x`**. Change
   `<reference_frame>` and this must be re-measured; no runtime check can catch
   it, only ground truth.
3. **Integrate on SIM time, not wall clock.** Gazebo does not run at real time.
   `time.monotonic()` deltas inflate distance by roughly 1/RTF, and because RTF
   drifts with scene load the error *grows with the move*: 0.89 m of real travel
   reported as 0.93, then 1.58 as 2.04, then 2.18 as 3.00. Use the message's own
   `header.stamp`. (`NucleusDVLSource` keeps wall clock, correctly — on the
   vehicle real time *is* the clock.)
4. **`ros_gz_bridge` cannot carry this message.** There is no
   `DVLVelocityTracking` conversion and no matching `ros_gz_interfaces` `.msg`,
   so the usual `parameter_bridge` route does not exist. `dvl_bridge.py` speaks
   gz-transport directly. It is **observability only** — `SimDvlSource`
   subscribes to gz itself, so a debugging node being down can never break
   vehicle control.

### Using it

```bash
ros2 run duburi_sim_bringup duburi_sim stack            # yaw_source=sim_dvl by default
ros2 run duburi_sim_bringup duburi_sim stack yaw_source:=mavlink_ahrs   # no DVL
```

`sim_dvl` is a **composite**: heading still comes from MAVLink AHRS, only
position comes from the DVL. A DVL registered as a bare yaw source would
displace the heading source and break every yaw verb —
`sources/composite_bno_dvl.py` is the pattern.

Topics: `/duburi/sim/dvl/velocity` (`TwistWithCovarianceStamped`, body frame)
and `/duburi/sim/dvl/altitude` (`Range`, bottom-track).

### Accuracy today

| commanded | DVL reports | ground truth |
|---|---|---|
| 1.0 m | 1.10 m | 1.26 m |
| 2.0 m | 2.14 m | 2.19 m |
| 3.0 m | 3.05 m | 4.20 m |

The DVL's own figure is within ~10%. The residual gap to ground truth is
**water-inertia coast past the arrival brake**, i.e. control tuning, not
measurement. Tighten `REVERSE_KICK_PCT` / kick duration for the distance path if
it matters.

---

## Sonar — not available, and don't be fooled by the header

**Gazebo Harmonic ships no sonar sensor.** 19 sensor implementations are
installed and none of them is sonar. `gz/msgs10/gz/msgs/sonar.pb.h` exists but
is a legacy *message* with no sensor behind it — its presence is not evidence
that sonar works.

Substitutes, in order of effort:

| want | use | notes |
|---|---|---|
| bottom altitude | **already have it** — `/duburi/sim/dvl/altitude` | free, comes with the DVL |
| forward obstacle range | `gpu_lidar` | native, well supported; a single-beam config is a plausible echosounder stand-in |
| depth below surface | `altimeter` | native |
| imaging / multibeam sonar | custom plugin, or port from Project DAVE | real work; DAVE's sonar is the usual reference |

**Recommendation: don't build imaging sonar.** The competition tasks Mongla
targets are vision-driven, the DVL now covers the navigation gap that actually
blocked the `*_dist` verbs, and a `gpu_lidar` stand-in covers obstacle ranging
if a task ever needs it. Revisit only if a task genuinely requires acoustic
imaging.
