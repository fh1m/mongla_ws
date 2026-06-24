# Mongla FSM Guide — YASMIN State Machine Planning Layer

> **Status:** BUILT & TESTED (phase-2, commit 4a94231, 2026-06-02).
> Runs alongside `detected()` scripted missions — both use the same
> `run(duburi, log)` drop-in interface and auto-discovery.
>
> **Home:** `src/duburi_planner/duburi_planner/state_machines/`
> **Entry missions:** `missions/gate_flare_fsm.py`, `missions/prequal_fsm.py`
>
> **See also:** [`fsm-vision-missions.md`](fsm-vision-missions.md) — vision as primary
> guidance: search patterns, avoid branches, pick/drop sequences, gain tuning, DVL/timed
> decision table, full worked example.

---

## 1. Why YASMIN? (vs py_trees, vs raw scripts)

### The problem with raw detected() scripts

`detected()`-paradigm scripts work for simple linear sequences, but they hit a wall when:
- Mission logic branches on perception ("if gate not found in 60s, try yawing right")
- Recovery is needed after a state fails ("lost target → re-scan → retry home")
- Multiple operators need to read / modify the plan without tracing imperative code
- Pool-day bug report is "it got stuck" — you can't tell *where*

### Why not py_trees / behavior trees?

| Property | py_trees (BT) | YASMIN (FSM) |
|---|---|---|
| Execution model | Tick-based (every frame, nodes return SUCCESS/FAILURE/RUNNING) | Event-driven (state executes until it returns an outcome) |
| State visibility | "Which leaf ticked SUCCESS?" — indirect | "Current state: FIND_GATE" — explicit |
| Transition model | Tree structure (Sequence, Fallback, Parallel) — implicit | Explicit `{outcome: next_state}` dict — readable |
| Parallel tasks | Native (Parallel composite) | Need Concurrence node (rarely needed for AUV) |
| Complexity | Higher — subtree nesting, memory selectors, blackboard types | Lower — named states + outcome strings |
| ROS2 integration | `py_trees_ros` (separate package, extra deps) | `yasmin_ros` (official ROS2 Humble apt pkg) |
| Competition debugging | Hard to answer "what was it doing?" | Easy — just read the last logged state name |
| Adding a new task | New subtree, must understand parent tree | New plan builder file, copy pattern |

**Behavior trees shine for reactive behaviors** (a robot that reacts to continuous sensor streams, runs parallel behaviors, needs preemption). **AUV competition missions are fundamentally sequential**: arm → dive → search → align → pass → next task. Recovery is "retry this phase" or "surface". YASMIN maps directly to this mental model.

The 2025 BRACU Duburi RoboSub codebase used YASMIN and placed 8th world. This build inherits that design, upgraded to YASMIN 5.0.0 and unified with Mongla's DSL verbs.

### Why not just script everything?

Scripts (detected() paradigm) are kept and remain the **prototyping / unit-test / fallback** layer. FSMs pay off when:
- You need explicit timeouts per phase (each state has `TIMEOUT_S`)
- You want retry loops (`HOME_GATE → TIMEOUT → FIND_GATE` — automatic re-search)
- Multiple people maintain the mission plan and need to read it as a graph
- You want to hot-swap the gate heading or depth without re-reading Python

---

## 2. YASMIN 5.0.0 Fundamentals

### Installation (already done in this repo)

```bash
sudo apt install ros-humble-yasmin ros-humble-yasmin-ros
```

`package.xml` in `duburi_planner` already declares `<depend>yasmin</depend>`.

### Core concepts

```
┌─────────────────────────────────────────┐
│  StateMachine                           │
│  ┌─────────┐  outcome   ┌───────────┐  │
│  │  State A │ ─────────▶│  State B  │  │
│  │ execute()│           │ execute() │  │
│  └─────────┘            └───────────┘  │
│       │  terminal outcome              │
│       ▼                                │
│  "succeeded" / "aborted"  (exits SM)   │
└─────────────────────────────────────────┘

Blackboard: shared dict between all states in one SM execution
```

**State** — unit of work. Implements `execute(blackboard) → str`:
```python
class MyState(State):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'timeout'])

    def execute(self, blackboard):
        # do work
        if done:
            return 'succeeded'
        return 'timeout'
```

**StateMachine** — routes outcomes to next states:
```python
sm = StateMachine(outcomes=['succeeded', 'aborted'])
sm.add_state('A', MyState(), transitions={
    'succeeded': 'B',    # go to B on success
    'timeout': 'SURFACE' # go to SURFACE on timeout
})
sm.add_state('B', OtherState(), transitions={'succeeded': 'succeeded'})
sm.set_start_state('A')
outcome = sm(Blackboard())  # ← __call__, not .execute()!
```

**Blackboard** — dict-like shared memory. Supports `bb['key']` and `bb.get('key')`:
```python
bb = Blackboard()
bb['start_heading'] = 63.0
heading = bb['start_heading']
```

### YASMIN 5.0.0 API quirks to know

| Item | Behaviour |
|---|---|
| `sm(bb)` | Executes SM (uses `__call__`, NOT `sm.execute()`) |
| `sm.validate()` | Checks all transitions reference existing states — run before first use |
| `add_transition_cb(fn)` | Registered but **Python callbacks don't fire** in 5.0.0 C++ binding |
| Transition logging | YASMIN's C++ core logs `[INFO] state_machine.cpp: transitioning X→Y` automatically |
| ROS logger routing | `from yasmin_ros import set_ros_loggers; set_ros_loggers()` → routes to `/rosout` |
| Outcomes module | `from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT` |

---

## 3. Mongla FSM Architecture

### File layout

```
state_machines/
├── __init__.py                  ← public API (VehicleProfile, build_* fns, outcomes)
├── core/
│   ├── outcomes.py              ← SUCCEED / FAILED / TIMEOUT / ABORT constants
│   ├── blackboard.py            ← BK.* typed key registry (prevents typos)
│   ├── vehicle_profile.py       ← VehicleProfile dataclass
│   └── base_state.py            ← DuburiState: timeout + ABORT-on-exception
├── states/
│   ├── navigation.py            ← motion + safety states
│   ├── vision.py                ← perception states
│   └── utility.py               ← countdown, pause, score
└── plans/
    ├── gate_flare.py            ← build_gate_flare_fsm()
    ├── prequal.py               ← build_prequal_fsm()
    ├── gate_then_bin.py         ← build_gate_then_bin_fsm()
    ├── slalom.py                ← build_slalom_fsm()
    ├── bin_drop.py              ← build_bin_drop_fsm()
    ├── torpedo_fire.py          ← build_torpedo_fire_fsm()
    ├── return_gate.py           ← build_return_gate_fsm()
    └── full_competition.py      ← build_full_competition_fsm()
```

### VehicleProfile — the plug-and-play key

```
                ┌─────────────────────────────────┐
                │  VehicleProfile.auto(node)        │
                │  probes /duburi_manager at runtime│
                └──────────────┬──────────────────┘
                               │
              ┌────────────────┴────────────────┐
        yaw_source=dvl                    yaw_source=mavlink_ahrs
              │                                  │
   ┌──────────▼──────────┐           ┌───────────▼──────────┐
   │  VehicleProfile      │           │  VehicleProfile       │
   │  name='duburi45'     │           │  name='dubomini'      │
   │  has_dvl=True        │           │  has_dvl=False        │
   │  has_manipulators=T  │           │  has_manipulators=F   │
   │  mission_depth=-0.8m │           │  mission_depth=-0.6m  │
   └─────────────────────┘           └──────────────────────┘
```

States query `profile.has_dvl`, `profile.has_manipulators` to pick the right verb.

**Named constructors for pool-day without a running manager:**
```python
profile = VehicleProfile.duburi45()   # force Duburi 4.5
profile = VehicleProfile.dubomini()   # force Dubomini 2.0
profile = VehicleProfile.auto(node)   # auto-detect (recommended in missions)
```

### DuburiState base class

Every state inherits from `DuburiState` which wraps YASMIN's `State`:

```python
class DuburiState(State):
    TIMEOUT_S = 60.0  # override per state

    def execute(self, blackboard):
        self._start = time.monotonic()
        try:
            return self._run(blackboard)     # subclass logic
        except Exception as exc:
            self.duburi.stop()               # always stop thrusters on crash
            blackboard[BK.LAST_ERROR] = str(exc)
            return ABORT

    def timed_out(self): ...
    def elapsed(self): ...
```

Rules for subclasses:
- Implement `_run(bb)` only
- Check `self.timed_out()` in any polling loop
- Never call Pixhawk/MAVLink directly — use `self.duburi.*` DSL verbs
- Return one of: `SUCCEED`, `FAILED`, `TIMEOUT`, `ABORT`

### BK — typed blackboard keys

```python
from duburi_planner.state_machines.core.blackboard import BK

bb[BK.START_HEADING]   # float deg — written by ArmState
bb[BK.DVL_CONNECTED]   # bool — written by ArmState
bb[BK.MISSION_START_T] # float monotonic — written by CountdownState
bb[BK.LAST_ERROR]      # str — written by base class on exception
bb[BK.GATE_HEADING]    # float deg — passed via params dict
```

Always use `BK.*` constants, never raw strings — grep-able and typo-safe.

---

## 4. State Library Reference

### Navigation states (`states/navigation.py`)

| State | Constructor | Outcomes | Notes |
|---|---|---|---|
| `ArmState` | `(duburi, profile)` | SUCCEED, ABORT | Arms + DVL connect if `has_dvl`; sets `BK.DVL_CONNECTED` |
| `DisarmState` | `(duburi, profile)` | SUCCEED | Calls `release_heading()` before `disarm()` (mirrors `SurfaceState`) |
| `SetDepthState` | `(duburi, profile, depth_m, timeout_s=45)` | SUCCEED, TIMEOUT, ABORT | Calls `duburi.set_depth()` |
| `LockHeadingState` | `(duburi, profile, heading=0.0, lock_timeout=600.0)` | SUCCEED, TIMEOUT, ABORT | Calls `duburi.lock_heading(heading, timeout=lock_timeout)`; stores `BK.START_HEADING`. `lock_timeout` is how long the lock HOLDS heading across later states (mission/hold duration), **not** this state's `TIMEOUT_S` |
| `MoveForwardState` | `(duburi, profile, distance_m=None, duration=None, gain=60)` | SUCCEED, ABORT | **DVL/timed auto-select** |
| `MoveBackState` | same | SUCCEED, ABORT | **DVL/timed auto-select** |
| `MoveLateralState` | same | SUCCEED, ABORT | **DVL/timed auto-select** |
| `SurfaceState` | `(duburi, profile)` | SUCCEED, ABORT | release_heading + stop + set_depth(0) + disarm |

**DVL/timed auto-selection rule in MoveForwardState:**
```
if distance_m is not None AND profile.has_dvl:
    duburi.move_forward_dist(distance_m, gain)   ← DVL closed-loop
else if duration is not None:
    duburi.move_forward(duration, gain)           ← timed open-loop
```
Always pass **both** `distance_m` and `duration` in plan builders so each vehicle gets the right path.

### Vision states (`states/vision.py`)

Three states map 1:1 onto the two-verb vision API plus an open-loop search.
(Every state also inherits `TIMEOUT` + `ABORT` from `DuburiState`; ABORT fires
on an unexpected exception.)

| State | Constructor | Outcomes | Wraps DSL |
|---|---|---|---|
| `VisionSearchState` | `(duburi, profile, target, camera=None, pattern='forward', gain=35, step_s=0.6, yaw_step=20.0, timeout=45.0)` | SUCCEED, TIMEOUT, ABORT | open-loop creep / yaw-sweep; polls `duburi.detected()` (replaces find/scan) |
| `VisionAlignState` | `(duburi, profile, target, camera=None, yaw=None, lat=None, depth=None, err=40, gain=30, duration=20, fallback=None)` | SUCCEED, FAILED, ABORT | `duburi.vision.align()` |
| `VisionMoveState` | `(duburi, profile, target, camera=None, fwd=95, mode='area', maintain=None, hold=None, gain=30, duration=20, fallback=None)` | SUCCEED, FAILED, ABORT | `duburi.vision.move()` |

**Axis flags on `VisionAlignState`** (`yaw` / `lat` / `depth`): `True` = centre
(offset 0), a number = signed **pixel offset** from centre, `None`/`False` =
axis off. At least one axis must be on.

**VisionSearchState outcome semantics:**
- `SUCCEED` — target detected within the search window
- `TIMEOUT` — search window elapsed without a detection (`pattern='forward'`
  creeps ahead; `pattern='yaw'` sweeps in `yaw_step` increments)

**VisionAlignState outcome semantics:**
- `SUCCEED` — every active axis held within `err` px for `align_stable_frames`
  ticks (`VisionResult.ok`)
- `FAILED` — any miss (LOST / TIMEOUT / NO_CAMERA); the verb never raises
- `fallback` — mission-authored `fn(duburi[, should_stop])` search run on
  target loss; the verb re-enters within the same `duration`

Use `FAILED → FIND_*` in your plan to auto-retry after target loss.

**VisionMoveState outcome semantics:**
- `SUCCEED` — bbox reached `fwd`% fill (`mode` = area/width/height; `maintain`
  holds a lateral px offset while driving; `hold` station-keeps once reached)
- `FAILED` — fill not reached before `duration` elapsed, or target lost.
  Never re-centres yaw/depth (ArduSub holds depth; heading lock holds yaw)

### Navigation states — additional (`states/navigation.py`)

| State | Constructor | Outcomes | Notes |
|---|---|---|---|
| `TurnState` | `(duburi, profile, heading_deg: float)` | SUCCEED | Absolute compass heading snap via `duburi.turn()`. Use to orient toward a task zone. |

### Utility states (`states/utility.py`)

| State | Constructor | Outcomes | Notes |
|---|---|---|---|
| `CountdownState` | `(duburi, profile, seconds=10)` | SUCCEED | Tether-removal window; sets `BK.MISSION_START_T` |
| `PauseState` | `(duburi, profile, seconds=3.0)` | SUCCEED | `duburi.pause()` dwell |
| `LogScoreState` | `(duburi, profile)` | SUCCEED | Exports mission scoreboard JSON |
| `FireState` | `(duburi, profile, channel: int, confirm_pause_s=2.0)` | SUCCEED | `duburi.fire(channel)` + settle pause. Channels: 1/2=torpedo, 3/4=dropper. Always explicit. |
| `StyleRollState` | `(duburi, profile, flips=1, headroom=0.4, gain=60)` | SUCCEED | ACRO roll manoeuvre; use as final style points after Return gate pass. |

---

## 5. Plan Builders

Plan builders are functions that return a `StateMachine`. They take:
- `duburi` — DuburiMission instance (from the mission runner)
- `profile` — VehicleProfile (usually from `VehicleProfile.auto(node)`)
- `params` — optional dict to override defaults

### `build_gate_flare_fsm` — state graph

Vision is the two-verb API: **search** (open-loop) → **align** (centre) →
**move** (drive in). Each align/move SUCCEEDs only on a real outcome; a miss
routes back to search.

```
COUNTDOWN → ARM → DIVE → LOCK_HEADING
                              │
                         FIND_GATE  (VisionSearchState) ←────────────┐
                              │ SUCCEED        TIMEOUT → SURFACE      │
                         HOME_GATE  (VisionAlignState yaw+lat)        │
                              │ SUCCEED        FAILED ────────────────┘
                         MOVE_GATE  (VisionMoveState area)
                              │ SUCCEED / FAILED (commit pass)
                         PASS_GATE  (MoveForwardState DVL/timed) ── ABORT → SURFACE
                              │ SUCCEED
                         FIND_FLARE (VisionSearchState) ── TIMEOUT → SURFACE
                              │ SUCCEED
                         HOME_FLARE (VisionAlignState yaw+depth)
                              │ SUCCEED        FAILED → FIND_FLARE (retry)
                         MOVE_FLARE (VisionMoveState height)
                              │ SUCCEED / FAILED
                         SCAN_GATE  (VisionSearchState yaw) ── TIMEOUT → SURFACE
                              │ SUCCEED
                        RETURN_HOME (VisionAlignState yaw+lat)
                              │ SUCCEED        FAILED → SCAN_GATE (re-scan)
                        RETURN_PASS → LOG_SCORE → SURFACE → succeeded
```

### `build_prequal_fsm` — state graph

```
COUNTDOWN → ARM → DIVE → LOCK_HEADING → FIND_GATE → HOME_GATE
                                                         │ SUCCEED
                                                    PASS_GATE → LOG_SCORE → SURFACE → succeeded
                                   TIMEOUT/FAILED at any state → SURFACE
```

### Default params

```python
from duburi_planner.state_machines import GATE_FLARE_DEFAULTS, PREQUAL_DEFAULTS

GATE_FLARE_DEFAULTS = {
    'countdown_s':    10,
    'depth_m':       -0.8,
    'gate_heading':   0.0,    # ← SET THIS at pool day
    'pass_dist_m':    3.5,    # DVL distance through gate
    'pass_duration':  5.0,    # timed fallback for Dubomini
    'pass_gain':      80,
    'return_dist_m':  1.5,
    'return_duration': 3.0,
    'find_timeout':   45.0,
    'align_duration': 20.0,   # vision.align budget (HOME_* states)
    'move_duration':  20.0,   # vision.move budget (MOVE_* states)
    'align_err_px':   40,     # px tolerance for "aligned"
    'align_gain':     30,     # max-speed cap while centring
    'approach_gain':  45,     # max-speed cap while driving in
    'gate_fwd_fill':  42,     # gate area % of frame at standoff
    'flare_fwd_fill': 38,     # flare height % of frame at standoff
}
```

Override any key at runtime without code edits:
```python
sm = build_gate_flare_fsm(duburi, profile, params={
    'gate_heading': 63.0,   # compass heading to gate at this pool
    'depth_m':     -1.0,    # deeper competition pool
    'pass_dist_m':  4.0,    # wider gate
})
```

### Competition task plan builders (RoboSub 2026)

Five new builders cover the full competition run. Each is standalone-runnable or chained via `full_competition`.

| Builder | Launcher mission | Key fill-at-pool params |
|---|---|---|
| `build_slalom_fsm` | `fsm_slalom` | `slalom_heading` (None=skip turn), `pipe_offset_px=80` |
| `build_bin_drop_fsm` | `fsm_bin` | `bin_heading`, `bin_depth_m=-1.0`, `fire_channel=3` |
| `build_torpedo_fire_fsm` | `fsm_torpedo` | `torpedo_heading`, **`torpedo_depth_m`** (assert != None) |
| `build_return_gate_fsm` | `fsm_return` | `return_heading`, `pass_depth_m`, `pass_bbox_frac` |
| `build_full_competition_fsm` | `fsm_full_2026` | all above; torpedo section skipped if `torpedo_depth_m=None` |

Heading params default to `None` — when None, the turn state is omitted and the AUV proceeds straight.
`torpedo_depth_m=None` (default) causes the torpedo task to be skipped entirely in `full_competition`.

```bash
# Standalone task FSMs:
ros2 run duburi_planner mission fsm_slalom
ros2 run duburi_planner mission fsm_bin
ros2 run duburi_planner mission fsm_torpedo    # requires TORPEDO_DEPTH_M != None
ros2 run duburi_planner mission fsm_return
ros2 run duburi_planner mission fsm_full_2026  # ★ recommended full competition run
```

`fsm_full_2026` is a flat 5-task state machine. Each task section's failure transitions to the
**next task's entry state** (skip pattern) rather than surfacing — the AUV completes as much as possible.

---

## 6. Running Prequal on Both AUVs

### Quickstart — same command, auto-detects vehicle

```bash
# Gate + Flare FSM (Duburi 4.5 OR Dubomini 2.0 — auto-detected)
ros2 run duburi_planner mission gate_flare_fsm

# Gate-only prequal
ros2 run duburi_planner mission prequal_fsm
```

The mission prints at startup:
```
[FSM] body=duburi45  dvl=True  manip=True  depth=-0.8m
```
or:
```
[FSM] body=dubomini  dvl=False  manip=False  depth=-0.6m
```

You see exactly what profile was detected before any movement.

### Gate + Flare prequal — with custom params

Create a one-off wrapper mission (or pass params inline):

```python
# missions/competition_day.py
from duburi_planner.state_machines import build_gate_flare_fsm, VehicleProfile
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

def run(duburi, log):
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera = 'forward'
    duburi.set_classes('gate,flare')

    profile = VehicleProfile.auto(duburi.client.node)

    # Pool-day tunable block — edit here, not in the plan builder
    params = {
        'gate_heading': 63.0,   # today's pool compass heading
        'depth_m':     -0.9,    # competition pool depth
        'pass_dist_m':  3.5,
        'align_duration': 15.0, # faster vision.align convergence once tuned
    }

    set_ros_loggers()
    sm = build_gate_flare_fsm(duburi, profile, params=params)
    outcome = sm(Blackboard())
    log(f'[FSM] result: {outcome}')
```

```bash
ros2 run duburi_planner mission competition_day
```

### Force a specific vehicle profile

When manager is not running (bench test):

```python
profile = VehicleProfile.duburi45()   # forces DVL path
profile = VehicleProfile.dubomini()   # forces timed path
```

### Verify detector is loaded before running

```bash
# Check detector sees gate and flare
ros2 run duburi_vision vision_check --camera forward --require-class gate
ros2 run duburi_vision vision_check --camera forward --require-class flare

# See detection → RC echo (safe, sub disarmed)
ros2 run duburi_vision vision_thrust_check --camera forward --duration 4
```

---

## 7. Vision-DVL Smooth Autonomous Missions — Design Pattern

The FSM shines when combining closed-loop vision + DVL distance. The vision
half is the two-verb search → align → move chain:

```
FIND_GATE (VisionSearchState, pattern='forward')   → detects gate
HOME_GATE (VisionAlignState, yaw=True, lat=True)    → centre yaw + lateral on the gate
MOVE_GATE (VisionMoveState, fwd=42, mode='area')    → drive in until gate fills 42% of frame
PASS_GATE (MoveForwardState, distance_m=3.5, duration=5.0)
  → Duburi 4.5: DVL-measured 3.5m forward pass (precise, heading-locked)
  → Dubomini: 5.0s timed thrust at gain=80
```

Key settings for smooth vision+DVL:

| Param | Purpose | Tune toward |
|---|---|---|
| `align_duration` | Max seconds `vision.align` may converge | 15-25s after first pool test |
| `gate_fwd_fill` | Gate bbox % of frame that commits the pass | 38-45 (higher = closer = more risk of clipping) |
| `approach_gain` | Max-speed cap while `vision.move` drives in | 35-50 |
| `align_gain` | Max-speed cap while `vision.align` centres | 25-35 |
| `kp_yaw / kp_lat` | Vision loop P-gains | Tune via `ros2 param set /duburi_manager vision.kp_yaw 80` |
| `fallback=` | Mission search fn run on target loss; the verb re-enters within `duration` | Pass a creep/sweep fn; omit to coast through brief losses |

A real miss (LOST / TIMEOUT) routes the align/move state to `FAILED`, so wire
`FAILED → FIND_*` in the plan to auto-re-search. There is no `gate_guard`,
`pass_at`, or `on_lost` knob any more — the verbs are pixel-native and the
plan owns recovery.

### Heading lock + DVL forward = smooth straight pass

`LockHeadingState` engages `lock_heading` — the heading_lock background thread continues running through `MoveForwardState`. So during DVL forward pass: DVL controls distance on Ch5, heading_lock controls Ch4 yaw simultaneously. The AUV stays dead straight through the gate.

---

## 8. Adding a New Task (e.g. Slalom)

### Step 1 — new states if needed

If slalom needs a new behavior not covered by existing states, add to `states/navigation.py` or a new `states/slalom.py`:

```python
class YawStepState(DuburiState):
    """Yaw by a fixed increment. Returns SUCCEED always."""
    def __init__(self, duburi, profile, degrees: float, timeout_s=15.0):
        super().__init__(duburi, profile, [SUCCEED])
        self._deg = degrees
        self.TIMEOUT_S = timeout_s

    def _run(self, bb):
        self.duburi.yaw_right(self._deg) if self._deg > 0 else self.duburi.yaw_left(-self._deg)
        return SUCCEED
```

### Step 2 — new plan builder

Create `plans/slalom.py`:

```python
from yasmin import StateMachine
from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..states.navigation import ArmState, SetDepthState, LockHeadingState, MoveForwardState, SurfaceState
from ..states.vision import VisionSearchState, VisionAlignState, VisionMoveState

SLALOM_DEFAULTS = {
    'depth_m': -0.8,
    'gate_heading': 0.0,
    'pass_dist_m': 2.5,
    'pass_duration': 3.5,
}

def build_slalom_fsm(duburi, profile, params=None):
    p = {**SLALOM_DEFAULTS, **(params or {})}
    sm = StateMachine(outcomes=[SUCCEED, ABORT])
    # ... add states ...
    sm.set_start_state('COUNTDOWN')
    return sm
```

### Step 3 — drop-in mission

Create `missions/slalom_fsm.py`:

```python
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers
from ..state_machines.plans.slalom import build_slalom_fsm
from ..state_machines import VehicleProfile

def run(duburi, log):
    duburi.set_model('slalom_red_pipe')
    duburi.camera = 'forward'
    duburi.set_classes('red_pipe')
    profile = VehicleProfile.auto(duburi.client.node)
    set_ros_loggers()
    outcome = build_slalom_fsm(duburi, profile)(Blackboard())
    log(f'[FSM] {outcome}')
```

> The slalom task already ships as `plans/slalom.py` + `missions/fsm_slalom.py`
> (it switches model/classes inside a `SetDetectorState`). The skeleton above is
> the generic pattern for adding *any* new task.

### Step 4 — export

Add to `plans/__init__.py`:
```python
from .slalom import build_slalom_fsm, SLALOM_DEFAULTS
```

That's the complete pattern. No changes to any existing file except the plans `__init__.py`.

---

## 9. Testing FSM States

All states are unit-testable without hardware:

```bash
# Run FSM unit tests only
PYTHONPATH=src/duburi_planner python3 -m pytest -q -p no:anyio \
  src/duburi_planner/test/test_fsm_states.py

# Full planner suite
PYTHONPATH=src/duburi_planner python3 -m pytest -q -p no:anyio \
  src/duburi_planner/test/
```

**Test pattern:**

```python
from unittest.mock import MagicMock
from yasmin import Blackboard
from duburi_planner.state_machines.states.navigation import MoveForwardState
from duburi_planner.state_machines import SUCCEED, VehicleProfile

def test_dvl_path():
    d = MagicMock()
    state = MoveForwardState(d, VehicleProfile.duburi45(), distance_m=2.0, duration=4.0)
    assert state.execute(Blackboard()) == SUCCEED
    d.move_forward_dist.assert_called_once_with(2.0, gain=60)
```

**Testing exception → ABORT:**
```python
def test_abort_on_thruster_fault():
    d = MagicMock()
    d.move_forward.side_effect = RuntimeError('thruster fault')
    state = MoveForwardState(d, VehicleProfile.dubomini(), duration=3.0)
    assert state.execute(Blackboard()) == 'aborted'
    d.stop.assert_called_once()
```

---

## 10. Pool-Day Workflow

```bash
# 1. Bringup check
ros2 run duburi_manager bringup_check

# 2. Start control stack + detector (gate+flare model)
ros2 launch duburi_manager bringup.launch.py vision:=true

# 3. Verify detector
ros2 run duburi_vision vision_check --camera forward --require-class gate
ros2 run duburi_vision vision_check --camera forward --require-class flare

# 4. Disarmed detection → RC echo test
ros2 run duburi_vision vision_thrust_check --camera forward --duration 4

# 5. Run FSM prequal (will countdown 10s, then arm and execute)
ros2 run duburi_planner mission prequal_fsm

# 6. Run FSM full gate+flare
ros2 run duburi_planner mission gate_flare_fsm

# 7. Watch state transitions in ROS logs
ros2 topic echo /rosout   # shows [INFO] state_machine.cpp: transitioning X→Y

# 8. Tune gains live without restart
ros2 param set /duburi_manager vision.kp_yaw 75.0
ros2 param set /duburi_manager vision.kp_lat 65.0

# 9. Override pool-day params without code edit
#    Create a thin mission wrapper (see §6) with a params dict
```

---

## 11. Architecture Diagram

```
┌────────────────────────────────────────────────────────────┐
│  mission runner (ros2 run duburi_planner mission X)         │
│  ├── DuburiMission DSL (duburi.*)                          │
│  └── VehicleProfile.auto() ─── /duburi_manager ROS params  │
└────────────────────────────────┬───────────────────────────┘
                                 │
                    build_gate_flare_fsm(duburi, profile)
                                 │
         ┌───────────────────────▼────────────────────────┐
         │  YASMIN StateMachine  sm(Blackboard())          │
         │                                                 │
         │  COUNTDOWN → ARM → DIVE → LOCK_HEADING          │
         │      ↓                                          │
         │  FIND_GATE ←── (on FAILED/TIMEOUT retry) ──┐   │
         │      ↓                                      │   │
         │  HOME_GATE ─── FAILED ─────────────────────┘   │
         │      ↓ SUCCEED                                  │
         │  PASS_GATE  (DVL dist OR timed, auto)           │
         │      ↓                                          │
         │  FIND_FLARE → HOME_FLARE → SCAN_GATE            │
         │      ↓                                          │
         │  RETURN_HOME → RETURN_PASS → LOG_SCORE          │
         │      ↓                                          │
         │  SURFACE → "succeeded"                          │
         └─────────────────────────────────────────────────┘
                                 │
              ┌──────────────────▼──────────────────┐
              │  DuburiState.execute()               │
              │  calls: duburi.vision.align(...)     │
              │         duburi.vision.move(...)       │
              │         duburi.move_forward_dist(...) │
              │         duburi.set_depth(...)         │
              └──────────────────┬──────────────────┘
                                 │
              ┌──────────────────▼──────────────────┐
              │  /duburi/move ActionServer            │
              │  (auv_manager_node)                  │
              │  MAVLink → Pixhawk → ArduSub          │
              └─────────────────────────────────────┘
```

States never touch MAVLink. The FSM layer is entirely above the existing
Pixhawk/Duburi facade. If you swap the control layer, the FSM is untouched.
