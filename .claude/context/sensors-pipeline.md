# Sensors pipeline — `duburi_sensors`

This file is the design rationale + future-extension guide for the
`duburi_sensors` package. Read this before adding sensors, swapping
fusion strategy, or "improving" the architecture — the constraints
below were chosen deliberately.

---

## Why a separate package

Control code (`duburi_control`) should not know whether yaw came from
ArduSub's AHRS, an external BNO085, a DVL, or a stub. It should ask a
`YawSource` for `read_yaw()` and trust the answer (or hold last value
if `None`).

Putting the source plumbing in its own package gives us:

1. **Replaceable backend, fixed contract.** Swap chips by changing one
   ROS parameter; control code never recompiles.
2. **Sensor-only depends on `duburi_control`** (for the `Pixhawk`
   wrapper used by `MavlinkAhrsSource`), but `duburi_control` does NOT
   depend on `duburi_sensors`. The dependency arrow points one way.
3. **Standalone diagnostic** (`sensors_node`) can talk to any source
   without booting the action server.

---

## Design rules (non-negotiable)

These came out of explicit user direction during the architecture
discussion. Don't relax them without a new conversation.

| Rule                                          | Why                                                |
|-----------------------------------------------|----------------------------------------------------|
| **Single source per launch**                  | Mid-run switching needs a fusion layer + state     |
|                                               | machine; that's 10x complexity for marginal gain.  |
| **No fallback to another source**             | "Silent fallback" hides hardware failures. If the  |
|                                               | operator picked BNO085, they want to KNOW when it  |
|                                               | dies. Loud failure > quiet wrong-answer.           |
| **No fusion**                                 | BNO085 is already 9DoF-fused on-chip. Re-fusing on |
|                                               | the Jetson side adds latency without new info.     |
|                                               | Fusion belongs in a future `robot_localization`    |
|                                               | layer, not here.                                   |
| **Yaw-only firmware (BNO085)**                | Cheaper to flash, easier to debug, smaller wire    |
|                                               | format. Nothing in our control loop needs roll /   |
|                                               | pitch from the external chip — ArduSub gives us    |
|                                               | those just fine.                                   |
| **Read-only — no actuation, no MAVLink writes** | Sensor source must NEVER send to the autopilot.  |
|                                               | If something armed via this path, it'd be          |
|                                               | catastrophic on the desk.                          |
| **No vision in this package**                 | Vision is large enough to deserve its own package  |
|                                               | (`duburi_vision`). Don't pollute the sensor-only   |
|                                               | scope.                                             |

---

## Contract — `YawSource`

```python
class YawSource:
    name: str = 'base'

    def read_yaw(self) -> float | None:
        """Latest yaw in degrees [0, 360), magnetic north, +CW.
        Returns None when no fresh sample is available."""

    def is_healthy(self) -> bool:
        """True iff read_yaw() is currently producing usable values.
        Diagnostic only — never used for runtime fallback."""

    def close(self) -> None:
        """Release threads / sockets. Idempotent."""
```

Frame: north-east-down. 0° = magnetic north, +CW from above. This is
the same convention ArduSub uses for AHRS2 yaw, so any new source must
align with it. For sources whose hardware doesn't natively output
Earth-frame yaw (e.g. BNO085 in `SH2_GAME_ROTATION_VECTOR` mode), the
source itself owns the calibration that maps sensor frame → Earth
frame; downstream code must always see Earth-frame.

---

## Calibration model — BNO085 specific (and template for similar chips)

The BNO085 ships in `SH2_GAME_ROTATION_VECTOR` mode: gyro + accelerometer
fusion, magnetometer **disabled**. We deliberately don't use the BNO's
mag because the inside of the AUV is magnetically hostile (8 thrusters,
battery currents, aluminum (Marine 5083) hull). Without the mag the chip has no
absolute heading — its yaw=0 is whatever direction it was pointing at
boot.

To get an Earth-referenced heading anyway, `BNO085Source.__init__`
performs a **one-shot calibration** before returning:

```
offset_deg = pixhawk_yaw  -  bno_raw_yaw    # captured at boot, locked
read_yaw() = (bno_raw + offset_deg) mod 360 # applied forever after
```

Properties of this design that match the rules above:

| Property                                         | Why it stays consistent with the rules           |
|--------------------------------------------------|--------------------------------------------------|
| Calibration is **synchronous**, blocking up to 5 s in `__init__`. | Fail-loud at startup, not silently mid-mission.  |
| The Pixhawk magnetometer is read **once**, then never again.       | Avoids "two sources of truth" + magnetic drift.  |
| The offset is **immutable** for the lifetime of the source.        | No mid-run mutation = no race conditions.        |
| If `reference_yaw_provider` is `None`, the source returns RAW yaw. | Diag tool can run with no MAVLink. Mission code  |
|                                                                    | always passes a provider, so it always gets cal. |
| Failure mode: `RuntimeError`, port released, exception propagates. | Same loud-failure policy as a missing serial port. |

The calibration helper lives **inside** `BNO085Source` rather than in
the factory or in the manager. Reasons:

1. The factory's job is dispatch, not policy. A calibration loop is
   policy.
2. The manager doesn't care **how** a source becomes Earth-referenced;
   it only knows the contract: `read_yaw()` returns Earth yaw.
3. Future sources that need their own calibration (witmotion, dvl)
   can implement the same pattern without coordinating with the
   factory or the manager.

If you ever add a second sensor that also needs an Earth-reference
offset, copy the `_calibrate` / `reference_yaw_provider` pattern; do
**not** factor it out into a base-class hook unless three or more
sources need it. Two-of-something is not yet a pattern.

---

## Stale handling

Every source MUST implement a stale check inside `read_yaw()`. The
control loop polls at 10 Hz; the standard threshold is 250 ms. If the
last sample is older than that, return `None` and let the caller hold
its previous value.

Consequences:
- Reader threads keep running even when callers are idle.
- A source that crashed silently shows up as `is_healthy() == False`
  in the next telemetry tick.
- The control loop never blocks on `read_yaw()` — worst case it's a
  dictionary lookup + timestamp compare.

---

## Adding a new source

The mechanical steps:

1. `src/duburi_sensors/duburi_sensors/sources/<name>.py` — subclass
   `YawSource`. Mirror the BNO085 background-thread pattern.
2. `src/duburi_sensors/duburi_sensors/factory.py` — add a `_build_<name>`
   function and an entry in `_BUILDERS`.
3. `src/duburi_sensors/firmware/<name>.md` — wire format spec + smoke
   test instructions. (For pure-software sources like a network DVL,
   document the network protocol instead.)
4. README §10A — add a paragraph and update the architecture diagram.

The control side does not change. If you find yourself touching
`duburi_control` to add a sensor, stop — you're probably adding fusion
or fallback logic, both forbidden by the rules above.

---

## When to break the rules

- **Add fusion** → only after `robot_localization` lands as a separate
  package (`duburi_estimation` / similar). The fused estimate becomes
  a NEW `YawSource`; existing sources stay as they are.
- **Add mid-run switching** → only with explicit operator-in-the-loop
  control (e.g. a service call from QGC that changes source). Even
  then, audit the control-side state-machine: a smooth handoff needs
  yaw-rate continuity.
- **Add a fallback chain** → only with telemetry that surfaces "we
  fell back from X to Y" prominently (LED, audible alarm, or at least
  a `[FALLBACK]` log line every second). The reason for the rule is
  visibility, not the chain itself.

---

## Cross-references

- `[../../src/duburi_sensors/](../../src/duburi_sensors/)` — package source
- `[../../src/duburi_sensors/firmware/esp32c3_bno085.md](../../src/duburi_sensors/firmware/esp32c3_bno085.md)` — BNO085 wire contract
- `[./yaw-stability-and-fusion.md](./yaw-stability-and-fusion.md)` — research notes on yaw drift sources
- `[./proven-patterns.md](./proven-patterns.md)` — 2023/2025 codebase patterns we draw from
