# mongla_control — the verbs, and the line between us and the hardware

**no ROS nodes · [`src/mongla_control`](../../../../src/mongla_control)**

This package answers two questions:

1. **What does "move forward" actually mean?** Every verb's implementation lives here.
2. **How do we talk to the board?** One driver, `SrotFC`, over one USB-C link.

It is a plain Python library: it starts no nodes and subscribes to nothing — `mongla_manager`
imports it. That separation is what lets the tests exercise the real control logic with no ROS
graph and no vehicle.

---

## The board boundary

```
            Mongla facade  (one method per verb)
                   │
                SrotFC   (arm, mode, manual, move, fire, read)
                   │
          MAVLink 2 over USB-C, compid 191
                   │
        SROT board — Hengla firmware, 500 Hz
```

`fc/base.py` defines the typed outcomes every board call returns. `fc/srot_protocol.py` is the
**one copy** of the wire constants shared with the firmware, and a test reads the firmware's
own headers to prove it has not drifted. `fc/port_guard.py` enforces that one process owns the
serial port — a second one gets a clear error instead of a corrupted link. There is one board,
so there is no backend interface and no selector; the earlier stack lives at the tag
`last-with-pixhawk-and-sim`.

## The verbs

Twenty-three of them, in one registry (`commands.py`). Adding one touches two places: a row in
that registry and a method of the same name on the facade. The CLI, the action server and the
Python client pick it up with no further edits.

| Group | Verbs | What happens |
|---|---|---|
| **On the board** | `move_forward` · `move_back` · `move_left` · `move_right` · `yaw_left` · `yaw_right` · `turn` · `set_depth` · `stop` · `pause` · `style_roll` | one `SROT_MOVE`, run and braked on the board at 500 Hz |
| **Here** | `arm` · `disarm` · `set_mode` · `head` · `surface` · `mission_reset` · `calibrate_depth` · `calc_distance` · `fire` · `motor_test` · `vision_align` · `vision_move` | a host loop, or a single message |

A name that is not in the registry fails at the node with `Unknown command` and puts nothing on
the wire. There is no third "refused" group: a verb the board cannot run does not exist.

## The two vision verbs

Only two, and they are pixel-native: no metric model of the world is needed to use them.

- **`vision_align`** — hold the target at a chosen offset on the axes you name (sideways, yaw,
  and a forward standoff or, on the downward camera, surge). It can hold station for a few
  seconds and fire the payload mid-hold, on a *fresh* detection. It never commands depth:
  `set_depth` first, then align.
- **`vision_move`** — drive toward the target until it fills a chosen fraction of the frame,
  or straight through it.

Neither raises. Both return an outcome — aligned, lost, timed out, no camera, aborted,
drifted — plus where the target was when the verb ended, so a mission can recover instead of
guessing. They actuate through `MANUAL_CONTROL` in STABILIZE at 50 Hz.

## The map

| File | Role |
|---|---|
| `commands.py` | the registry: one row per verb, the single source of truth |
| `mongla.py` | the facade: locking, dispatch, and the safety verbs |
| `fc/srot_fc.py` | the board driver `SrotFC`: the link, arming, modes, the `SROT_MOVE` verbs, `manual()` |
| `fc/srot_telemetry.py` · `fc/srot_health.py` | what the board reports: attitude, IMU, batteries; STATUSTEXT, the boot burst, reboots, ESC and leak health |
| `fc/srot_params.py` · `fc/srot_bench.py` · `fc/srot_uplink.py` | parameters and the preflight checks; bench-only motor test / detect / tune; what the host sends the board it sees |
| `fc/srot_payload.py` · `fc/_srot_base.py` | `SrotPayload` (fire(N) on board channel N); the shared names and the one MAVLink 2 import |
| `fc/srot_protocol.py` · `fc/base.py` · `fc/port_guard.py` | wire constants, typed outcomes, one owner per port |
| `motion_vision.py` · `motion_writers.py` · `motion_rates.py` | the two vision loops, what they write, and their tick rate |
| `vision_verbs.py` | the verbs on top of those loops: mode gating, firing, evidence |
| `allocation.py` · `actuation_model.py` | what the mixer will *actually* deliver, and what the board does with a demand |
| `geometric_allocation.py` · `hydrodynamics.py` | the hull's real thruster geometry and what its shape implies |
| `bearing.py` · `nav_filter.py` | pixel → bearing, and the small filters the loops share |
| `errors.py` | the typed failures every verb raises |

The heartbeat and the heading source live in `mongla_manager` (`_HeartbeatThread`,
`yaw_source.BoardYaw`), next to the link they serve.

## Why `allocation.py` exists

Ask for full forward *and* full yaw and the mixer cannot give you both. The board scales the
whole set down uniformly, which quietly steals authority from the axis you cared about. So the
host fits the demand first — yaw first, because that is what the tool points with — and sends
something the mixer can honour. Below saturation it changes nothing.

## Testing

```bash
python3 -m pytest -q src/mongla_control/test
```

The style matters more than the count (that lives in [the packages page](../README.md)): the board is faked at the wire, so a test
fails when a *real* defect is injected. Several of these tests exist precisely because an
earlier version of them passed while the vehicle was broken.

---

Related: [`mongla_manager`](../mongla_manager/README.md) (who calls this) ·
[Capability Map](../../capability-map.md) · [The Shift](../../the-shift.md)
