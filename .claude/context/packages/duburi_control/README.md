# duburi_control — the verbs, and the line between us and the hardware

**13.2k lines · 865 tests · no ROS nodes · [`src/duburi_control`](../../../../src/duburi_control)**

This package answers two questions:

1. **What does "move forward" actually mean?** Every verb's implementation lives here.
2. **Who are we talking to?** One class per flight controller, behind one interface, so the
   rest of the codebase never has to ask which hardware is fitted.

It is a plain Python library: it starts no nodes and subscribes to nothing — `duburi_manager`
imports it. That separation is what lets 865 tests exercise the real control logic with no ROS
graph and no vehicle.

---

## The flight-controller boundary

```
            Duburi facade  (one method per verb)
                   │
        FlightController  (the interface: arm, mode, manual, move, fire, read)
                   │
        ┌──────────┴───────────┐
     SrotFC                 PixhawkFC
  the SROT board          ArduSub — how the simulator flies
```

`fc/base.py` defines the boundary and the typed outcomes. `fc/srot_protocol.py` is the **one
copy** of the wire constants shared with the firmware, and a test reads the firmware's own
headers to prove it has not drifted. `fc/port_guard.py` enforces that one process owns the
serial port — a second one gets a clear error instead of a corrupted link.

## The verbs

Thirty of them, in one registry (`commands.py`). Adding one touches two places: a row in that
registry and a method of the same name on the facade. The CLI, the action server and the
Python client pick it up with no further edits.

On the srot board they fall into three groups:

| Group | Verbs | What happens |
|---|---|---|
| **On the board** | `move_forward` · `move_back` · `move_left` · `move_right` · `yaw_left` · `yaw_right` · `turn` · `set_depth` · `stop` · `pause` · `style_roll` | one command, run and braked on the board at 500 Hz |
| **Here, as a loop** | `vision_align` · `vision_move` · `arm` · `disarm` · `set_mode` · `fire` · `surface` · `mission_reset` and the rest | a host loop, or a single message |
| **Refused** | `lock_heading` · `move_forward_dist` · `move_back_dist` · `move_lateral_dist` · `arc` · `style_yaw` | not supported on this backend; refused before dispatch, with the reason |

Refusing loudly is the point. The failure it prevents — a verb that reports success while the
vehicle does nothing — is the one that ends a competition run with nobody knowing why.

## The two vision verbs

Only two, and they are pixel-native: no metric model of the world is needed to use them.

- **`vision_align`** — hold the target at a chosen offset on whichever axes you name
  (sideways, yaw, depth, and a forward standoff). It can also hold station for a few seconds
  and fire the payload mid-hold, on a *fresh* detection.
- **`vision_move`** — drive toward the target until it fills a chosen fraction of the frame,
  or straight through it.

Neither raises. Both return an outcome — aligned, lost, timed out, no camera, aborted — plus
where the target was when the verb ended, so a mission can recover instead of guessing.

## The map

| File | Role |
|---|---|
| `commands.py` | the registry: one row per verb, the single source of truth |
| `duburi.py` | the facade: locking, dispatch, and the safety verbs |
| `fc/` | the backends, the wire constants, the port guard |
| `motion_vision.py` | the two vision loops |
| `vision_verbs.py` | the verbs on top of those loops: mode gating, firing, evidence |
| `allocation.py` | what the mixer will *actually* deliver, and fitting a demand yaw-first |
| `motion_forward.py` · `motion_lateral.py` · `motion_yaw.py` · `motion_depth.py` | per-axis motion, used on the ArduSub path |
| `heading_lock.py` · `heartbeat.py` | ArduSub-only: a background heading hold, and the keepalive that stops it disarming |
| `payload.py` | the older USB payload driver (on srot the board's own outputs are used) |
| `bearing.py` · `nav_filter.py` | pixel → bearing, and the small filters the loops share |
| `errors.py` | the typed failures every verb raises |

## Why `allocation.py` exists

Ask for full forward *and* full yaw and the mixer cannot give you both. The board scales the
whole set down uniformly, which quietly steals authority from the axis you cared about. So the
host fits the demand first — yaw first, because that is what the tool points with — and sends
something the mixer can honour. Below saturation it changes nothing.

## Testing

```bash
python3 -m pytest -q src/duburi_control/test
```

865 tests. The style matters more than the count: the board is faked at the wire, so a test
fails when a *real* defect is injected. Several of these tests exist precisely because an
earlier version of them passed while the vehicle was broken.

---

Related: [`duburi_manager`](../duburi_manager/README.md) (who calls this) ·
[Capability Map](../../capability-map.md) · [The Shift](../../the-shift.md)
