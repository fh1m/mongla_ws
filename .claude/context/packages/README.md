# The packages

Seven ROS 2 packages under [`src/`](../../../src). One page each, written so someone who has
never seen this codebase can find where a thing lives and how to run it.

![Package map](../../../docs/assets/diagrams/package-map.svg)

| Package | Lines | Tests | What it is |
|---|---|---|---|
| [`mongla_control`](mongla_control/README.md) | 13.2k | 865 | Every verb's implementation, and the boundary between us and a flight controller. A library — it owns no ROS nodes. |
| [`mongla_manager`](mongla_manager/README.md) | 8.5k | 388 | The one node that talks to the board: the link, the action server, the state topic, the operator tools. |
| [`mongla_vision`](mongla_vision/README.md) | 24k | 824 | Cameras, the Hailo detector, tracking and the visual lock, optical flow, optics and calibration, the mission console. |
| [`mongla_localization`](mongla_localization/README.md) | 3.6k | 244 | Where the vehicle is: the invariant filter, replay of late measurements, course priors, pose fusion. |
| [`mongla_planner`](mongla_planner/README.md) | 11k | 405 | The mission language, the CLI, the missions and the state machines. |
| [`mongla_sensors`](mongla_sensors/README.md) | 1.9k | 31 | One interface for "which way is north", whatever answers it. |
| [`mongla_interfaces`](mongla_interfaces/README.md) | — | — | The messages. One action and one state topic are the entire cross-package surface. |

Counts are non-test Python lines and `def test_` functions, measured 2026-09-17.

## How to read these

Start with **`mongla_manager`** if you want to know what happens when you type a command, or
**`mongla_vision`** if you want to know what happens to a camera frame. `mongla_control` is
where the two meet.

Two documents give the context these pages assume:

- [The Shift](../the-shift.md) — why the system is split between a board and a Pi.
- [Capability Map](../capability-map.md) — what the whole thing can do, with evidence.
