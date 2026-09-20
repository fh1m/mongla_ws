# mongla_manager — the node that owns the vehicle

**8.5k lines · 388 tests · [`src/mongla_manager`](../../../../src/mongla_manager)**

Exactly one process in the running system holds the link to the flight-control board. This is
it. Everything else — the CLI, a mission, a state machine, the web console — asks *this* node
over one ROS action, and it decides what actually reaches the hull.

That is a deliberate constraint, not an accident of history. Two processes writing motion
commands to the same vehicle is the kind of bug that looks like a broken sensor.

---

## What happens when you send a command

```
mongla arm ─────► /mongla/move (action) ─────► auv_manager_node
                                                    │
                                    ┌───────────────┼────────────────┐
                                    ▼               ▼                ▼
                          refused on this    run ON THE BOARD   run HERE as a
                          backend, clearly   as one primitive   host-side loop
```

The node answers three questions for every goal:

1. **Is this verb supported on this backend?** Six are refused on the srot board —
   `lock_heading`, the three distance moves, `arc`, `style_yaw`. They are refused *before
   dispatch*, with a message saying why: a refusal is safer than a verb that silently does
   nothing.
2. **Can the board do it better than we can?** Eleven verbs — the moves, turns, `set_depth`,
   `stop`, `pause`, `style_roll` — collapse into a single on-board primitive. The board runs
   and brakes the motion itself at 500 Hz and reports when it is done.
3. **Otherwise, run it here.** Vision alignment is the important case: a loop on this node
   streams axis demands while the board stabilises underneath.

## What it publishes

| Topic | What | Notes |
|---|---|---|
| `/mongla/state` | armed, mode, heading, depth, battery | on change plus a heartbeat; a missing number is `NaN`, never `0.0` |
| `/mongla/imu` · `/mongla/imu_rates` | the board's inertial data | stamped with the board's own clock where available |
| `/mongla/demand` | what we just asked for | the estimator uses it to predict motion |
| `/mongla/esc_rpm` | per-thruster RPM | srot only — it did not exist before |
| `/mongla/flare_order` | the competition order received over the radio | srot only, latched |

## Entry points

| Command | What it does |
|---|---|
| `ros2 run mongla_manager start` | the node itself (also `auv_manager`, `auv_manager_node`) |
| `ros2 run mongla_manager connect` | open the board's link and print everything it sends — no ROS graph, no built workspace needed. `--watch` for live, `--json` for machines |
| `ros2 run mongla_manager bringup_check` | preflight that grades each subsystem and exits non-zero on a real fault; `--srot` adds the board-specific checks |
| `ros2 run mongla_manager autotune` | run the board's own gain tuning, in water, behind a typed confirmation |
| `ros2 run mongla_manager flare_order` | send the competition's flare order to the vehicle over the radio |

`connect` **reports**; `bringup_check` **grades and gates**. Use the first to look, the second
to decide.

## The map

| File | Role |
|---|---|
| `auv_manager_node.py` | the node: link ownership, the action server, dispatch, telemetry |
| `bringup_check.py` | the preflight, section by section |
| `srot_connect.py` · `srot_format.py` · `srot_changes.py` | reading the board, formatting it, showing what changed |
| `srot_autotune.py` | the operator's front end to the board's tuning routine |
| `srot_recorder.py` | raw MAVLink capture for later analysis |
| `vision_state.py` | per-camera detection cache; turns a detection into a pixel error |
| `distance_state.py` · `flow_position.py` | distance travelled, from the downward camera |
| `vision_tunables.py` + `config/vision_tunables.yaml` | the live-tunable vision gains, in one place |
| `dispatch_policy.py` | which verbs go where, on a given backend |
| `health.py` · `health_reporters.py` | subsystem health, reported rather than assumed |
| `connection_config.py` | connection profiles and network constants |
| `launch/bringup.launch.py` | the whole vehicle: control, optional vision, optional localization |

## Running it

```bash
# the vehicle, with vision and the localization filter
ros2 launch mongla_manager bringup.launch.py vision:=true

# control only — the smallest useful system
ros2 run mongla_manager start

# is the board healthy enough to arm?
ros2 run mongla_manager bringup_check --srot
```

Every capability in the launch file is a switch with a default we can defend. Bring the
vehicle up bare, then add one at a time.

## Testing

```bash
python3 -m pytest -q src/mongla_manager/test
```

388 tests, no hardware required: the board, the cameras and the ROS graph are all faked at
their real boundaries. Several tests read the documentation in `.claude/context/` and fail
when a documented constant no longer matches the code.

---

Related: [`mongla_control`](../mongla_control/README.md) (what the verbs actually do) ·
[`mongla_interfaces`](../mongla_interfaces/README.md) (the action) ·
[The Shift](../../the-shift.md)
