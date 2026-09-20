# mongla_planner — how you tell the vehicle what to do

**11k lines · 405 tests · [`src/mongla_planner`](../../../../src/mongla_planner)**

Three ways in, one road out: a command line for a single verb, a Python mission language for a
run, and state machines for missions that want explicit states. All of them send the same goal
to the same action.

---

## The command line

```bash
ros2 run mongla_planner mongla arm
ros2 run mongla_planner mongla set_depth --target -0.5
ros2 run mongla_planner mongla move_forward --duration 5 --gain 40
ros2 run mongla_planner mongla disarm
```

The CLI is *generated* from the command registry in `mongla_control`. Add a verb there and it
appears here, with its arguments, automatically — there is no command list to keep in sync,
because there is no second list.

## The mission language

A mission is a Python file with a `run(mongla)` function. The object it is handed is the whole
vehicle:

```python
def run(mongla, log=None):
    mongla.mission_reset()
    mongla.arm()
    mongla.set_depth(-0.8)

    if mongla.detected('gate'):
        mongla.vision.align('gate', yaw=0, lat=0, gain=30)
        mongla.vision.move('gate', fwd=None)      # drive through it

    mongla.surface()
    mongla.disarm()
```

Grouped by what you are doing:

| Intent | Methods |
|---|---|
| **Move** | `arm` `disarm` `set_depth` `move_forward/back/left/right` `yaw_left/right` `turn` `surface` `stop` `pause` `fire` |
| **See** | `vision.align` `vision.move` · `detected` `wait_for` `where` `can_see` `side_on` `outline` |
| **Measure** | `range_to` `bearing_to` `floor_range` `floor_height` `pose` `motion` `fix_from_prop` `standoff_for_prop` |
| **Navigate to something known** | `use_course` `acquire` `goto_prop` `anchor_on` `absolute_heading` |
| **Manage the run** | `task` `use_budget` `worth_attempting` `note` `countdown` `log_scoreboard` |
| **Configure vision live** | `use_camera` `set_model` `set_classes` `set_conf` `set_vision_param` `pause_detector` `resume_detector` |
| **Know the vehicle** | `backend` — `'srot'` or `'pixhawk'` |

### Three ideas worth knowing before writing a mission

**`detected()` is a window, not a frame.** It asks "was this seen in the last second?", so a
class that flickers out of individual frames still counts as present. An `if detected()` runs
once — a moving search needs a loop.

**A run is a budget.** Competition scoring rewards finishing. `use_budget()` starts a clock on
arm, and `worth_attempting()` then answers, per task, whether there is time for the full
attempt, only a fallback, or neither — recording the verdict either way.

```python
v = mongla.worth_attempting('torpedo', points=300, worst_case_s=120,
                            fallback_s=25, fallback_points=100)
```

**A task can be abandoned cleanly.** `with mongla.task('gate', deadline_s=90):` cancels the
goal in flight when the deadline passes and raises, so the mission takes its fallback instead
of overrunning into the next task.

## The map

| File | Role |
|---|---|
| `mongla_dsl.py` | the mission language |
| `vision_dsl.py` | `mongla.vision.align` and `.move`, and their results |
| `client.py` | the blocking action client: deadlines, cancellation, goal ids |
| `cli.py` | the `mongla` command, generated from the registry |
| `mission.py` | the `mission` runner and its catalogue |
| `run_budget.py` | the budget and its verdicts |
| `resilience.py` | retries and fallbacks shared between missions |
| `model_context.py` | naming a detection model and a class together |
| `missions/` | the missions themselves, plus the pool-day constants |
| `state_machines/` | the state-machine layer, wrapping the same verbs as states |

## Missions

```bash
ros2 run mongla_planner mission --list
ros2 run mongla_planner mission sauvc_full
```

`sauvc_full` is the current shape of a full run: the gate, then the drum, then — optionally —
the flares. One dive, budgeted, each task bounded by a deadline, every outcome recorded.
**No mission has been flown on this platform yet**; their guards check names, depths, budgets,
and which verbs a backend will accept.

## Testing

```bash
python3 -m pytest -q src/mongla_planner/test
```

405 tests. Mission guards read a mission's own *call arguments* out of its syntax tree rather
than its prose — an earlier guard passed because the string it matched happened to sit in an
import line.

---

Related: [`mongla_control`](../mongla_control/README.md) (the verbs) ·
[`mongla_manager`](../mongla_manager/README.md) (what serves them) ·
[Capability Map](../../capability-map.md)
