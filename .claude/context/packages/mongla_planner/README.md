# mongla_planner — how you tell the vehicle what to do

**[`src/mongla_planner`](../../../../src/mongla_planner)**

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
| **Measure** | `range_to` `bearing_to` `floor_range` `floor_height` `pose` `motion` `fix_from_prop` `place_prop` `standoff_for_prop` |
| **Navigate to something known** | `use_course` `acquire` `goto_prop` `anchor_on` `absolute_heading` |
| **Manage the run** | `task` `use_budget` `worth_attempting` `note` `countdown` `log_scoreboard` |
| **Configure vision live** | `use_camera` `set_model` `set_classes` `set_conf` `set_vision_param` `pause_detector` `resume_detector` |

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
| `mongla_dsl.py` | the mission language: the verbs, the session, the run report |
| `dsl_detections.py` | what the cameras see: `detected()`, `wait_for()`, `where()`, `outline()` |
| `dsl_position.py` | where the vehicle is: bearings, ranges, position fixes, the fused pose, the anchored heading |
| `dsl_cameras.py` | which camera is live (`use_camera`, `camera_usable`), and `calc_distance` |
| `dsl_detector.py` | tuning the detectors mid-mission: `set_model`, `set_classes`, `set_conf`, `lock_class`, pause/resume |
| `dsl_support.py` | parsing detections; the helpers the parts share |
| `vision_dsl.py` | `mongla.vision.align` and `.move`, and their results |
| `client.py` | the blocking action client: deadlines, cancellation, goal ids |
| `cli.py` | the `mongla` command, generated from the registry |
| `mission.py` | the `mission` runner and its catalogue |
| `run_budget.py` | the budget and its verdicts |
| `resilience.py` | retries and fallbacks shared between missions |
| `model_context.py` | naming a detection model and a class together |
| `missions/` | the missions themselves, plus the pool-day constants |

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

Mission guards read a mission's own *call arguments* out of its syntax tree rather
than its prose — an earlier guard passed because the string it matched happened to sit in an
import line.

---

Related: [`mongla_control`](../mongla_control/README.md) (the verbs) ·
[`mongla_manager`](../mongla_manager/README.md) (what serves them) ·
[Capability Map](../../capability-map.md)

## Every module

<!-- modules:begin -->
Generated from each module's docstring by `tools/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `cli.py` | mongla CLI -- argparse wrapper auto-built from `COMMANDS`. |
| `client.py` | blocking Python API over the /mongla/move ActionServer. |
| `dsl_cameras.py` | Which camera is live: one detector at a time, because the Hailo holds one model. |
| `dsl_detections.py` | What the cameras see: detected(), wait_for(), where(), outline(). |
| `dsl_detector.py` | Tuning the detectors and the manager mid-mission: models, classes, confidence, parameters. |
| `dsl_position.py` | Where the vehicle is: bearings, ranges, position fixes, the fused pose, the anchored heading. |
| `dsl_support.py` | What the mission DSL's parts share: parsing detection messages, and small helpers. |
| `mission.py` | argparse runner that dispatches into missions/<name>.run. |
| `model_context.py` | typed model/class handles for the mission DSL. |
| `mongla_dsl.py` | the mission DSL. |
| `resilience.py` | Make a task survive its own failures. The idiom, without the tree library. |
| `run_budget.py` | What is still worth attempting, given the clock. Effort is not points. |
| `vision_dsl.py` | mongla.vision.* closed-loop sub-namespace (two verbs). |
| `missions/competition_config.py` | Competition pool-day constants for RoboSub 2026. |
| `missions/demo_dual_camera.py` | gate → bin → torpedo across TWO cameras, ONE detector at a time. |
| `missions/demo_find_person.py` | canonical vision-driven mission for desk/thruster testing. |
| `missions/demo_move_see.py` | minimal open-loop + vision demo (two-verb). |
| `missions/demo_pursue.py` | torpedo-style approach using align() then move(). |
| `missions/demo_square.py` | the classic shakedown mission. |
| `missions/pool_day_practice.py` | Pool day practice run: Gate → Slalom → Torpedo → Bin. |
| `missions/pool_day_torpedo.py` | Pool day torpedo practice: align → approach → fine-lock → fire. |
| `missions/sauvc_full.py` | SAUVC 2026 full run: Navigation, Target Acquisition, then (opt-in) the flares. |
| `missions/sauvc_navigation.py` | SAUVC 2026 Navigation — swim through the red/green gate. THE mandatory task. |
| `missions/sauvc_qualification.py` | SAUVC Qualification — through the hanging gate, U-turn, back through it. |
| `missions/sauvc_target_acquisition.py` | SAUVC 2026 Target Acquisition — drop the ball into the BLUE drum. 30 points. |
| `missions/survey_course.py` | put the props where the vehicle actually sees them. |
| `missions/task_bin.py` | Bin drop task — DOWNWARD camera, hover over the bin, drop a marker. |
| `missions/task_full_2026.py` | Full RoboSub 2026 competition run: Gate → Slalom → Bin → Torpedo → Return+Roll. |
| `missions/task_gate.py` | Gate task — find the gate and drive through it. |
| `missions/task_return.py` | Return task — find gate again, pass back through, style roll. |
| `missions/task_slalom.py` | Slalom task — weave through red pipes. |
| `missions/task_torpedo.py` | Torpedo task — align on board, approach via blood, standoff-lock hole, fire. |
| `missions/test.py` | a scratch bench mission, retargeted 2026-09-28. |
<!-- modules:end -->
