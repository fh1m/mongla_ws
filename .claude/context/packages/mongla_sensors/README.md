# mongla_sensors — one question, several possible answers

**1.9k lines · 31 tests · [`src/mongla_sensors`](../../../../src/mongla_sensors)**

"Which way is the vehicle pointing?" has had four different answers over this project's life,
and a fifth is plausible. This package exists so the rest of the codebase never has to care
which one is fitted.

---

## The interface

One abstract class with a small surface: the current heading, how old that reading is, and
optionally a position. Everything else — an autopilot, a separate IMU board, a Doppler
velocity log — implements it.

| Source | Heading from | Status today |
|---|---|---|
| `mavlink_ahrs` | whichever flight controller is connected | **the live path.** On srot this is the board's own fused attitude |
| `bno085` | a separate IMU on a USB board | not fitted — the IMU moved onto the SROT board |
| `nucleus_dvl` | a Nortek DVL | never fitted, never validated in water |
| `composite_bno_dvl` | IMU heading + DVL position | the same, combined |
| `sim_dvl` | the simulator's DVL | simulator only |

On the current vehicle `mavlink_ahrs` reads the SROT board, which measured, at rest, drift
below **0.01 °/min** and under **0.08°** peak-to-peak across eight minutes. That heading is
relative to where the board booted, not to magnetic north — which matters when a mission talks
about absolute bearings. Absolute heading comes from
[`mongla_localization`](../mongla_localization/README.md) instead: from a prop of known
bearing, or from pool lane lines.

## Why keep the abstraction when only one source is live?

Because the question keeps coming back. A magnetometer inside an aluminium hull cannot be
trusted; a DVL is expensive and heavy; a camera pointed at the floor turns out to be a decent
velocity sensor. Each is the same question with different hardware, and switching is one
launch argument (`yaw_source:=`) rather than a refactor.

The package also ships a standalone diagnostic node that reads a sensor and prints it. It
never touches thrusters or arming, so it is safe to run beside anything else.

```bash
ros2 run mongla_sensors sensors_node
```

## The map

| File | Role |
|---|---|
| `factory.py` | name → source instance |
| `sources/base.py` | the interface everything implements |
| `sources/mavlink_ahrs.py` | the live path: the flight controller's own attitude |
| `sources/bno085.py` | the USB IMU board (not fitted) |
| `sources/nucleus_dvl.py` · `nucleus_parser.py` | the DVL protocol |
| `sources/composite_bno_dvl.py` | heading from one, position from another |
| `sensors_node.py` | the read-only diagnostic node |
| `config/sensors.yaml` | ports, hosts and defaults |

---

Related: [`mongla_localization`](../mongla_localization/README.md) (absolute heading and
position) · [Capability Map](../../capability-map.md)
