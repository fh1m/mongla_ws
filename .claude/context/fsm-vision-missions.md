# Vision-Guided FSM Mission Design — First Principles

> **Companion to:** [`fsm-guide.md`](fsm-guide.md) (state library reference, pool-day workflow)
> **Prerequisite:** [`fsm-guide.md`](fsm-guide.md) §1–4 (VehicleProfile, DuburiState, outcomes)

This document covers **how to design missions where vision drives every decision**.
It starts from first principles, builds a vocabulary of patterns, and ends with
a fully-worked multi-task example: search → avoid → deep dive → pick → bin drop.

---

## 1. Two Modes of Vision

Every vision interaction fits one of two modes. Confusing them is the #1 design error.

| Mode | What it does | Mongla primitive | Blocks? |
|---|---|---|---|
| **Vision-as-trigger** | Detects that an object IS present; changes state | `duburi.detected('gate')` | No — cache poll |
| **Vision-as-control** | Continuously centres the AUV on a target via closed-loop | `duburi.vision.home(...)` | Yes — runs until settle/timeout |

```
Vision-as-TRIGGER (inside a state's _run loop):

  while not detected('gate'):
      move_forward(0.5)        ← open-loop step
  return SUCCEED               ← gate seen → next state


Vision-as-CONTROL (hand off to DSL):

  result = duburi.vision.home(target='gate', yaw=True, lat=True)
  return SUCCEED if result.success else FAILED
  ← DSL runs a 20 Hz P-loop until error < deadband × N frames
```

**Rule:** use `detected()` to decide *when* to change state; use `vision.home/find/scan` to do the actual alignment work inside a dedicated state.

---

## 2. The Detection Model

### 2.1 How `detected()` works inside a state

`duburi.detected(class_name, camera=None, stale_after=1.0)` is a **non-blocking cache read**:

```
Detector node → /duburi/vision/<cam>/detections (Detection2DArray, ~15-25 Hz)
                         │
                DuburiMission._on_detections()  ← lazy-subscribed on first call
                         │
                _det_cache[camera] = (monotonic_stamp, {class_names})
                         │
duburi.detected('gate') ←── reads cache; returns True if:
                              1. class_name in cached set
                              2. stamp is fresh (< stale_after seconds)
```

Safe in any tight loop. Adds ~50 ms latency (one `spin_once` per call). No side-effects.

### 2.2 Detection confidence and staleness

| `stale_after` | Effect | Use when |
|---|---|---|
| 0.1s | Only very recent detections count | Object is fast-moving or camera is erratic |
| 0.5s | Single missed frame doesn't trigger false-negative | Default for most search loops |
| 1.0s (default) | Tolerates up to 1s outage | Stable scene, slow AUV |
| 2.0s+ | Sticky detection — hard to lose | Vision-as-trigger only, never for control |

### 2.3 Multi-class polling pattern

```python
# Inside a state's _run() method — poll priority order matters
for _ in range(MAX_STEPS):
    if self.timed_out():
        return TIMEOUT

    # Priority 1: safety/avoid checks first
    if self.duburi.detected('obstacle', stale_after=0.3):
        return 'avoid'           # custom outcome → routes to AVOID state

    # Priority 2: mission target
    if self.duburi.detected('gate', stale_after=0.5):
        return SUCCEED

    # No detection: take one search step
    self.duburi.move_forward(STEP_S, gain=SEARCH_GAIN)

return TIMEOUT
```

Custom outcomes (not in `SUCCEED/FAILED/TIMEOUT/ABORT`) are fully supported — just list them in the state's `__init__`:

```python
class SearchState(DuburiState):
    def __init__(self, duburi, profile):
        super().__init__(duburi, profile, [SUCCEED, 'avoid'])
        # TIMEOUT and ABORT added automatically by DuburiState base
```

---

## 3. DVL vs Timed — Decision Table

Every movement in a mission falls into one of these categories. Choose before coding.

| Movement type | Duburi 4.5 (`has_dvl=True`) | Dubomini 2.0 (`has_dvl=False`) | Notes |
|---|---|---|---|
| **Pass through gate** | `move_forward_dist(3.5m, gain=80)` | `move_forward(5.0s, gain=80)` | `MoveForwardState` handles automatically |
| **Approach to standoff** | `vision.approach(dist=0.4, metric='area')` | same | Vision-measured — DVL irrelevant |
| **Lateral clear of obstacle** | `move_lateral_dist(0.8m, gain=50)` | `move_right(1.5s, gain=50)` | Lateral precision matters less |
| **Return through gate** | `move_forward_dist(1.5m, gain=80)` | `move_forward(3.0s, gain=80)` | |
| **Depth change** | `set_depth(-5.0m)` | same | ArduSub ALT_HOLD always; DVL irrelevant |
| **Yaw search sweep** | `yaw_right(20°)` × N | same | Heading-based; DVL irrelevant |
| **Hold position** | DVL auto-helps via EKF | POSHOLD not available | Duburi can hold; Dubomini drifts |
| **Forward search steps** | `move_forward(0.5s, gain=30)` | same | Short steps — precision irrelevant |
| **Orbit around target** | `vision.scan(step=20, dwell=1.5)` | same | Vision-guided; DVL irrelevant |
| **Pick approach** | `vision.approach(dist=X, metric='height')` | same | Vision-measured |
| **Drop manoeuvre** | `move_lateral_dist` or stay in place | timed lateral | DVL for precision |

**Core rule: any open-loop distance move should use DVL when available.
Vision-closed-loop moves (home, approach, scan) are DVL-agnostic.**

---

## 4. Search Pattern Catalog

These are the six reusable search primitives. Combine them as FSM states.

### 4.1 Forward march with trigger

```
AUV moves forward in short steps; polls detected() each step.
Exits immediately on detection.

    ──→──→──→──→──→──→──→──[DETECTED]──▶ next state
              no detect

Best for: gate (known direction), long straight searches.
```

```python
class MarchSearchState(DuburiState):
    TIMEOUT_S = 120.0
    def __init__(self, duburi, profile, target: str, step_s=0.5, gain=30, max_steps=80):
        super().__init__(duburi, profile, [SUCCEED])
        self._target, self._step_s, self._gain, self._max = target, step_s, gain, max_steps

    def _run(self, bb):
        for _ in range(self._max):
            if self.timed_out(): return TIMEOUT
            if self.duburi.detected(self._target, stale_after=0.5): return SUCCEED
            self.duburi.move_forward(self._step_s, gain=self._gain)
        return TIMEOUT
```

### 4.2 Yaw sweep with trigger (look_around)

```
AUV rotates in place (POSHOLD), stops every N° to observe.
Exits on first detection.

       20°   20°   20°   20°
    ──▶rot──▶rot──▶rot──▶[DETECTED]──▶ next state

Best for: post-pass searching for next task object, orbit around marker.
```

Use `VisionScanState(duburi, profile, target='flare', step=20, dwell=1.5, duration=90)`.

### 4.3 Yaw sweep + forward march (helix)

```
Sweep N°, check, step forward, sweep again — helix pattern.

    ──→──yaw──→──yaw──→──yaw──[DETECTED]──▶

Best for: large unknown area, no prior heading to target.
```

```python
class HelixSearchState(DuburiState):
    TIMEOUT_S = 180.0
    def __init__(self, duburi, profile, target, yaw_step=30, fwd_step=1.0, gain=30):
        super().__init__(duburi, profile, [SUCCEED])
        self._t, self._ys, self._fs, self._g = target, yaw_step, fwd_step, gain

    def _run(self, bb):
        for direction in ([1] * (360 // self._ys)):   # full circle
            if self.timed_out(): return TIMEOUT
            if self.duburi.detected(self._t, stale_after=0.5): return SUCCEED
            self.duburi.yaw_right(self._ys)
            if self.duburi.detected(self._t, stale_after=0.5): return SUCCEED
            self.duburi.move_forward(self._fs, gain=self._g)
        return TIMEOUT
```

### 4.4 Depth-layer scan (search at new depth)

```
Dive to target depth → stabilise → scan.
Used when target is at a known depth (bin, path marker).

    dive(-5m) → stabilise → [scan pattern] → DETECTED
```

Two states: `SetDepthState(-5.0)` → `VisionScanState` or `MarchSearchState`.

### 4.5 Standoff approach (vision-measured distance)

```
Home on target until within threshold. DVL-agnostic.

    ────────────────────────▶ [bbox grows] ──▶ SUCCEED when area ≥ threshold
    vision.home(forward=True, dist=0.4, metric='area')
```

Use `VisionHomeState(forward=True, dist=0.4, metric='area')`.
For bins seen from above: `metric='area'`, `depth=True` (nudge depth down).

### 4.6 Multi-priority search (search A while watching for B)

```
Primary: search for object A.
Secondary: if object B detected, yield to AVOID state, then resume.

    step → poll_A → poll_B → step → poll_A → [B!] → 'avoid'
              ↑_______________________________↑ (after avoid, returns here)
```

Implemented with custom outcomes + FSM loop-back transition (see §5 full example).

---

## 5. Vision-Guided Mission Patterns

### 5.1 Find → Centre → Pass

The fundamental gate-passing pattern:

```
FIND (march forward)
  │ SUCCEED
HOME (yaw + lat, gate_guard, pass_at=0.38)
  │ SUCCEED (pass_at triggered)
PASS (DVL 3.5m or timed 5s)
```

State graph:
```
FIND_GATE ──SUCCEED──▶ HOME_GATE ──SUCCEED──▶ PASS_GATE
    ▲                      │FAILED
    └──────────────────────┘ (lost target → re-find)
```

### 5.2 Orbit + Confirm

Fly around a marker to confirm its nature or score points:

```
FIND → HOME (yaw + lat) → SCAN (orbit 360°) → SCORE
```

Each step in `VisionScanState` pauses `dwell` seconds to re-detect.

### 5.3 Depth Dive → Search Below

Target is at known depth (bin, path marker):

```
SET_DEPTH(-5.0m)
  │ SUCCEED
SCAN_AT_DEPTH (VisionScanState)
  │ SUCCEED
HOME_ON_TARGET (yaw + lat + forward from above)
```

### 5.4 Pick Sequence

```
APPROACH (vision.home, dist=0.15m from camera)
  │ SUCCEED
ACTUATE_GRAB (set_servo or ESP32 grab cmd)
  │
CONFIRM_PICK (poll current or bbox disappear)
  │ SUCCEED | TIMEOUT (assume picked)
ASCEND (set_depth to drop depth)
```

### 5.5 Drop Sequence

```
SEARCH_BIN (yaw sweep looking down-camera)
  │ SUCCEED
LOCK_BIN (vision.home yaw+lat, depth=True nudge down)
  │ SUCCEED
ACTUATE_DROP (dropper command)
  │
CONFIRM_DROP (bbox reappears, or timeout)
  │ SUCCEED
```

---

## 6. Full Worked Example — Search → Avoid → Deep Pick → Bin Drop

### 6.1 Mission brief

> Move forward searching; yaw-sweep if needed. When **object_a** detected, approach it.
> If **object_b** appears at any time, lateral-clear and avoid, then resume.
> Once **object_a** secured, dive to −5 m. Scan for **object_c**. When found,
> lock on, approach, confirm grabber pick by sensing. Ascend to −1 m.
> Scan for **bin_a** (downward camera). Lock above bin. Drop. Confirm. Surface.

### 6.2 State graph

```
COUNTDOWN
    │
ARM (+ DVL connect if Duburi 4.5)
    │
DIVE_SEARCH (-1.2m mission depth)
    │
SEARCH_A ◀──────────────────────────────────────────────┐
    │ SUCCEED: object_a detected                         │
    │ 'avoid':  object_b detected mid-search             │
    │                 │                                  │
    │            AVOID_B ─────────────────────────────────┘
    │            (lateral clear + pause)
    │
HOME_A (vision.home yaw+lat+forward, dist=0.3, metric='height')
    │ SUCCEED
    │ FAILED → SEARCH_A (lost object_a → re-search)
    │
DEEP_DIVE (-5.0m)
    │ SUCCEED
    │ TIMEOUT → SURFACE
    │
SCAN_C (yaw sweep, down-cam or forward-cam)
    │ SUCCEED
    │ TIMEOUT → SURFACE
    │
HOME_C (vision.home yaw+lat, approach to 0.2m)
    │ SUCCEED
    │ FAILED → SCAN_C
    │
PICK (actuate grabber)
    │
CONFIRM_PICK (poll for grasp signal; timeout = assume picked)
    │ SUCCEED | TIMEOUT
    │
ASCEND_DROP (-1.0m drop depth)
    │ SUCCEED
    │
SEARCH_BIN (VisionScanState, down-camera, target='bin_a')
    │ SUCCEED
    │ TIMEOUT → SURFACE_WITH_OBJ (ascend and keep object)
    │
LOCK_BIN (vision.home yaw+lat+depth from above, metric='area')
    │ SUCCEED
    │ FAILED → SEARCH_BIN
    │
DROP (actuate dropper)
    │
CONFIRM_DROP (bbox in bin, or timeout)
    │ SUCCEED | TIMEOUT
    │
LOG_SCORE → SURFACE → "succeeded"
```

### 6.3 Custom state implementations

#### SearchAWhileAvoidingState

```python
from duburi_planner.state_machines.core.base_state import DuburiState
from duburi_planner.state_machines.core.outcomes import SUCCEED, TIMEOUT

SEARCH_STEP_S   = 0.5    # forward step duration
SEARCH_GAIN     = 30     # conservative gain while scanning
MAX_SEARCH_STEPS = 120   # ~60s at 0.5s steps

class SearchAWhileAvoidingState(DuburiState):
    """March forward; yaw-sweep every 8 steps. Exits on object_a or object_b."""
    TIMEOUT_S = 70.0

    def __init__(self, duburi, profile, target_a='object_a', avoid_trigger='object_b'):
        # 'avoid' is a custom outcome alongside the standard ones
        super().__init__(duburi, profile, [SUCCEED, 'avoid'])
        self._target_a = target_a
        self._avoid    = avoid_trigger

    def _run(self, bb):
        for step in range(MAX_SEARCH_STEPS):
            if self.timed_out():
                return TIMEOUT

            # Priority 1: avoid hazard (check first, every step)
            if self.duburi.detected(self._avoid, stale_after=0.3):
                bb['avoid_direction'] = 'right'   # hint for AvoidBState
                return 'avoid'

            # Priority 2: mission target
            if self.duburi.detected(self._target_a, stale_after=0.5):
                return SUCCEED

            # Search step: small forward advance
            self.duburi.move_forward(SEARCH_STEP_S, gain=SEARCH_GAIN)

            # Yaw sweep every 8 steps to widen search arc
            if step > 0 and step % 8 == 0:
                self.duburi.yaw_right(15.0)
                if self.duburi.detected(self._target_a, stale_after=0.5):
                    return SUCCEED
                self.duburi.yaw_left(15.0)   # return to heading

        return TIMEOUT
```

#### AvoidBState

```python
AVOID_LATERAL_M  = 0.8    # Duburi 4.5 DVL distance
AVOID_LATERAL_S  = 1.8    # Dubomini timed equivalent
AVOID_PAUSE_S    = 2.0

class AvoidBState(DuburiState):
    """Lateral-clear object_b; pause; return SUCCEED to re-enter search."""
    TIMEOUT_S = 20.0

    def __init__(self, duburi, profile, avoid_trigger='object_b'):
        super().__init__(duburi, profile, [SUCCEED])
        self._trigger = avoid_trigger

    def _run(self, bb):
        # Check which side to dodge (hint from search state, default right)
        direction = bb.get('avoid_direction', 'right')

        if self.profile.has_dvl:
            if direction == 'right':
                self.duburi.move_lateral_dist(AVOID_LATERAL_M, gain=50)
            else:
                self.duburi.move_lateral_dist(-AVOID_LATERAL_M, gain=50)
        else:
            if direction == 'right':
                self.duburi.move_right(AVOID_LATERAL_S, gain=50)
            else:
                self.duburi.move_left(AVOID_LATERAL_S, gain=50)

        # Pause and let object_b pass / clear
        self.duburi.pause(AVOID_PAUSE_S)

        # Wait until object_b not visible before resuming
        deadline = self.elapsed() + 8.0
        while self.elapsed() < deadline:
            if not self.duburi.detected(self._trigger, stale_after=0.3):
                break
            self.duburi.pause(0.3)

        return SUCCEED
```

#### ConfirmPickState

```python
PICK_CONFIRM_POLL_S = 0.2
PICK_CONFIRM_TIMEOUT = 5.0

class ConfirmPickState(DuburiState):
    """Poll for grasp confirmation. Tries three signals in priority order:
    1. ESP32 actuator ACK (when payload serial is live)
    2. Target bbox disappears from camera (occluded by gripper)
    3. Timeout — assume picked and continue
    """
    TIMEOUT_S = 8.0

    def __init__(self, duburi, profile, target='object_c', camera='forward'):
        super().__init__(duburi, profile, [SUCCEED])
        self._target = target
        self._camera = camera

    def _run(self, bb):
        deadline = self.elapsed() + PICK_CONFIRM_TIMEOUT
        bbox_was_visible = True

        while self.elapsed() < deadline:
            # Signal 2: bbox disappears (object now in gripper, occluded)
            if bbox_was_visible and not self.duburi.detected(
                    self._target, camera=self._camera, stale_after=0.3):
                bb['pick_confirmed'] = 'bbox_occluded'
                return SUCCEED

            # Track whether it was previously visible
            if self.duburi.detected(self._target, stale_after=0.5):
                bbox_was_visible = True

            self.duburi.pause(PICK_CONFIRM_POLL_S)

        # Timeout — assume picked (conservative: continue mission)
        bb['pick_confirmed'] = 'timeout_assumed'
        return SUCCEED   # TIMEOUT outcome would abort to surface; not desired here
```

#### LockBinState (downward camera alignment)

```python
class LockBinState(DuburiState):
    """Align above bin using downward camera.
    yaw + lat centre; depth nudge to get target area fraction.
    """
    TIMEOUT_S = 30.0

    def __init__(self, duburi, profile, target='bin_a',
                 camera='downward', duration=20.0):
        super().__init__(duburi, profile, [SUCCEED, FAILED])
        self._target   = target
        self._camera   = camera
        self._duration = duration

    def _run(self, bb):
        # Home: yaw+lat to centre bin in down-cam, depth nudge to get area ~0.25
        result = self.duburi.vision.home(
            target=self._target,
            camera=self._camera,
            yaw=True,
            lat=True,
            depth=True,            # nudge depth down to enlarge bbox
            target_bbox_h_frac=0.5,# target: bin fills ~50% of frame height
            metric='area',
            duration=self._duration,
            on_lost='fail',
        )
        return SUCCEED if result.success else FAILED
```

#### ConfirmDropState

```python
CONFIRM_DROP_TIMEOUT = 4.0

class ConfirmDropState(DuburiState):
    """Confirm object landed in bin.
    Looks for target bbox reappearing in downward camera (fell into bin, now visible from above).
    Falls back to timeout (assume dropped).
    """
    TIMEOUT_S = 6.0

    def __init__(self, duburi, profile, target='object_c', camera='downward'):
        super().__init__(duburi, profile, [SUCCEED])
        self._target = target
        self._camera = camera

    def _run(self, bb):
        deadline = self.elapsed() + CONFIRM_DROP_TIMEOUT
        while self.elapsed() < deadline:
            # Object visible in down-cam below AUV → dropped successfully
            if self.duburi.detected(self._target, camera=self._camera, stale_after=0.5):
                bb['drop_confirmed'] = 'bbox_visible'
                return SUCCEED
            self.duburi.pause(0.3)
        bb['drop_confirmed'] = 'timeout_assumed'
        return SUCCEED
```

### 6.4 Plan builder

```python
# plans/pick_and_drop.py
from yasmin import StateMachine
from ..core.outcomes import SUCCEED, FAILED, TIMEOUT, ABORT
from ..core.vehicle_profile import VehicleProfile
from ..states.navigation import (
    ArmState, SetDepthState, LockHeadingState,
    MoveForwardState, SurfaceState,
)
from ..states.vision import VisionHomeState, VisionScanState
from ..states.utility import CountdownState, LogScoreState

# Import custom states defined above (same file or imported)
# from ..states.custom import (SearchAWhileAvoidingState, AvoidBState,
#     ConfirmPickState, LockBinState, ConfirmDropState, PickActuateState, DropActuateState)

PICK_DROP_DEFAULTS = {
    'countdown_s':     10,
    'search_depth_m': -1.2,    # depth while searching for object_a
    'deep_depth_m':   -5.0,    # depth for object_c scan
    'drop_depth_m':   -1.0,    # depth for bin approach
    'approach_dist':   0.3,    # vision.home standoff to object_a
    'pick_dist':       0.15,   # vision.home standoff for grabber
    'gate_heading':    0.0,
}

def build_pick_drop_fsm(duburi, profile: VehicleProfile,
                         params: dict | None = None) -> StateMachine:
    p = {**PICK_DROP_DEFAULTS, **(params or {})}
    sm = StateMachine(outcomes=[SUCCEED, ABORT])

    sm.add_state('COUNTDOWN',
                 CountdownState(duburi, profile, seconds=p['countdown_s']),
                 transitions={SUCCEED: 'ARM', ABORT: ABORT})

    sm.add_state('ARM',
                 ArmState(duburi, profile),
                 transitions={SUCCEED: 'DIVE_SEARCH', ABORT: 'SURFACE'})

    sm.add_state('DIVE_SEARCH',
                 SetDepthState(duburi, profile, depth_m=p['search_depth_m']),
                 transitions={SUCCEED: 'LOCK_HEADING', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('LOCK_HEADING',
                 LockHeadingState(duburi, profile, heading=p['gate_heading']),
                 transitions={SUCCEED: 'SEARCH_A', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── search with avoid ─────────────────────────────────────────────
    sm.add_state('SEARCH_A',
                 SearchAWhileAvoidingState(duburi, profile),
                 transitions={SUCCEED: 'HOME_A',
                               'avoid': 'AVOID_B',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('AVOID_B',
                 AvoidBState(duburi, profile),
                 transitions={SUCCEED: 'SEARCH_A',   # ← loops back!
                               ABORT: 'SURFACE'})

    # ── approach object_a ─────────────────────────────────────────────
    sm.add_state('HOME_A',
                 VisionHomeState(duburi, profile, target='object_a',
                                 yaw=True, lat=True, forward=True,
                                 dist=p['approach_dist'], metric='height',
                                 duration=20.0, on_lost='fail'),
                 transitions={SUCCEED: 'DEEP_DIVE',
                               FAILED: 'SEARCH_A',    # lost → re-find
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── deep dive + scan for object_c ─────────────────────────────────
    sm.add_state('DEEP_DIVE',
                 SetDepthState(duburi, profile, depth_m=p['deep_depth_m']),
                 transitions={SUCCEED: 'SCAN_C', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SCAN_C',
                 VisionScanState(duburi, profile, target='object_c',
                                 step=20.0, dwell=2.0, duration=120.0),
                 transitions={SUCCEED: 'HOME_C', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── lock on object_c for pick ─────────────────────────────────────
    sm.add_state('HOME_C',
                 VisionHomeState(duburi, profile, target='object_c',
                                 yaw=True, lat=True, forward=True, depth=True,
                                 dist=p['pick_dist'], metric='height',
                                 duration=20.0, on_lost='fail'),
                 transitions={SUCCEED: 'PICK',
                               FAILED: 'SCAN_C',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('PICK',
                 PickActuateState(duburi, profile),   # ESP32 serial grab cmd
                 transitions={SUCCEED: 'CONFIRM_PICK', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_PICK',
                 ConfirmPickState(duburi, profile, target='object_c'),
                 transitions={SUCCEED: 'ASCEND_DROP', ABORT: 'SURFACE'})

    # ── ascend + bin search ───────────────────────────────────────────
    sm.add_state('ASCEND_DROP',
                 SetDepthState(duburi, profile, depth_m=p['drop_depth_m']),
                 transitions={SUCCEED: 'SEARCH_BIN', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SEARCH_BIN',
                 VisionScanState(duburi, profile, target='bin_a',
                                 step=20.0, dwell=1.5, duration=90.0),
                 transitions={SUCCEED: 'LOCK_BIN',
                               TIMEOUT: 'SURFACE_WITH_OBJ',   # no bin found: give up
                               ABORT: 'SURFACE'})

    sm.add_state('LOCK_BIN',
                 LockBinState(duburi, profile, target='bin_a', camera='downward'),
                 transitions={SUCCEED: 'DROP',
                               FAILED: 'SEARCH_BIN',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('DROP',
                 DropActuateState(duburi, profile),   # ESP32 serial drop cmd
                 transitions={SUCCEED: 'CONFIRM_DROP', ABORT: 'SURFACE'})

    sm.add_state('CONFIRM_DROP',
                 ConfirmDropState(duburi, profile, target='object_c', camera='downward'),
                 transitions={SUCCEED: 'LOG_SCORE', ABORT: 'SURFACE'})

    sm.add_state('LOG_SCORE',
                 LogScoreState(duburi, profile),
                 transitions={SUCCEED: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SURFACE_WITH_OBJ',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.add_state('SURFACE',
                 SurfaceState(duburi, profile),
                 transitions={SUCCEED: SUCCEED, ABORT: ABORT})

    sm.set_start_state('COUNTDOWN')
    return sm
```

### 6.5 Drop-in mission entry

```python
# missions/pick_drop_fsm.py
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers
from ..state_machines.plans.pick_and_drop import build_pick_drop_fsm
from ..state_machines import VehicleProfile

def run(duburi, log):
    # ── detector setup ──────────────────────────────────────────────
    duburi.models(
        objects='pick_drop_combined_100ep',   # model with object_a, object_b, object_c, bin_a
    )
    duburi.camera  = 'forward'
    duburi.set_classes('object_a,object_b,object_c,bin_a')

    # ── vehicle detection ───────────────────────────────────────────
    profile = VehicleProfile.auto(duburi.client.node)
    log(f'[FSM] body={profile.name}  dvl={profile.has_dvl}')

    # ── pool-day params ─────────────────────────────────────────────
    params = {
        'gate_heading':  63.0,   # compass heading for this pool
        'search_depth_m': -0.9,
        'deep_depth_m':  -4.5,
        'drop_depth_m':  -1.0,
        'approach_dist':  0.35,
    }

    set_ros_loggers()
    sm = build_pick_drop_fsm(duburi, profile, params=params)
    outcome = sm(Blackboard())
    log(f'[FSM] result: {outcome}')
```

---

## 7. Vision Parameter Tuning Reference

### 7.1 `vision.home` axis selection

| Situation | `yaw` | `lat` | `depth` | `forward` | Notes |
|---|---|---|---|---|---|
| Centre gate in frame | ✓ | ✓ | — | — | Depth hold by ArduSub; no vertical correction |
| Gate pass (committed) | ✓ | ✓ | — | ✓ | `gate_guard=True`, `pass_at=0.38` |
| Flare approach | ✓ | ✓ | ✓ | ✓ | Full 4-axis; `metric='height'` |
| Object approach for pick | ✓ | ✓ | ✓ | ✓ | `metric='height'`, tight `dist` |
| Lock above bin | ✓ | ✓ | ✓ | — | Down-cam; `metric='area'`, nudge depth |
| Track moving target | ✓ | ✓ | — | — | `lock_mode='follow'`; never exits on settle |

### 7.2 `dist` and `metric` per object

| Object | Camera | `metric` | `dist` | Rationale |
|---|---|---|---|---|
| Gate | forward | `area` | 0.38 | Pass when fills 38% frame area |
| Flare / buoy | forward | `height` | 0.38–0.42 | Height more stable for tall targets |
| Pick object | forward | `height` | 0.15–0.20 | Close approach; height = real size proxy |
| Bin (from above) | downward | `area` | n/a (depth nudge) | Depth nudged until area ~0.25 |
| Torpedo target | forward | `width` | 0.30–0.35 | Width = board face width |
| Path marker | downward | `area` | n/a | Large flat target |

### 7.3 `on_lost` behaviour

| `on_lost` | State outcome | Use for |
|---|---|---|
| `'fail'` (default) | `FAILED` immediately | Production missions — force re-search on loss |
| `'hold'` | Holds last setpoint; waits for reappearance | Debugging, slow-moving targets |

### 7.4 `kp_*` gain tuning guide

Gains are live-tunable via `ros2 param set /duburi_manager vision.kp_yaw 80.0`.

| Gain | Too low symptom | Too high symptom | Starting point |
|---|---|---|---|
| `kp_yaw` | Slow, drifts off-axis during forward motion | Oscillates yaw, never settles | 60–70 |
| `kp_lat` | Misses lateral centre; passes offset | Jerky strafe, oscillates | 55–65 |
| `kp_depth` | Misses vertical centre | AUV bobs up/down | 40–50 |
| `kp_forward` | Stays far; doesn't approach | Overshoots, crashes | 30–45 |

Tune order: `kp_yaw` first (most critical), then `kp_lat`, then `kp_depth`, then `kp_forward`.

### 7.5 `deadband` tuning

Deadband = minimum error fraction before correction fires. Too tight = chattery, too loose = sloppy.

| Object | `deadband` | Why |
|---|---|---|
| Gate | 0.05 | Large target; coarse centering fine |
| Buoy / flare | 0.04 | Medium target |
| Pick object | 0.02 | Tight precision required for grabber |
| Bin (from above) | 0.04 | Precision needed but not extreme |

---

## 8. State Outcome Vocabulary (complete reference)

| Outcome | When returned | Typical plan transition |
|---|---|---|
| `SUCCEED` | Task completed successfully | Next task state |
| `FAILED` | Target lost during vision-control (`on_lost='fail'`) | Re-search state (avoid surfacing for recoverable error) |
| `TIMEOUT` | `TIMEOUT_S` elapsed | Safety surface OR re-search if budget allows |
| `ABORT` | Unhandled exception; base class caught it; `stop()` called | Always → SURFACE |
| Custom (e.g. `'avoid'`) | Specific condition detected mid-state | Side-branch state, then loop back |

**Decision rule for FAILED vs TIMEOUT transitions:**

```
FAILED → re-search:   target was in sight, we lost it — likely re-findable nearby
TIMEOUT → surface:    we spent our budget without progress — unsafe to continue
ABORT → surface:      hardware/software fault — always surface immediately
```

---

## 9. FSM Design Checklist

Before writing a new plan:

- [ ] Draw the state graph on paper first — every arrow is an explicit decision
- [ ] Every state has a `TIMEOUT` path to `SURFACE` (no infinite loops)
- [ ] Every state has `ABORT → SURFACE` (exceptions always surface)
- [ ] `detected()` priority order: safety checks first, mission target second
- [ ] `vision.home` uses `on_lost='fail'` → plan routes `FAILED` to re-search
- [ ] Both `distance_m` AND `duration` passed to `MoveForwardState` (DVL/timed)
- [ ] Detector loaded + classes filtered before first vision state
- [ ] `VehicleProfile.auto()` called before SM construction (or hardcoded for test)
- [ ] `CONFIRM_*` states use `SUCCEED` on timeout (conservative: continue) not `TIMEOUT` (abort)
- [ ] Camera set per-task if switching between forward/downward (set before each scan state)
- [ ] `set_classes()` updated when switching target (e.g. `'gate,flare'` → `'object_c,bin_a'`)
- [ ] `log_scoreboard()` in `LOG_SCORE` state before final SURFACE

---

## 10. Camera Switching in Multi-Task Missions

When a mission uses both forward and downward cameras:

```python
class SetCameraState(DuburiState):
    """Switch sticky camera context + update detector classes."""
    def __init__(self, duburi, profile, camera: str, classes: str):
        super().__init__(duburi, profile, [SUCCEED])
        self._camera  = camera
        self._classes = classes

    def _run(self, bb):
        self.duburi.camera = self._camera
        self.duburi.set_classes(self._classes)
        return SUCCEED
```

Insert between phases:

```
... PICK ──▶ SET_CAMERA_DOWN ──▶ ASCEND_DROP ──▶ SEARCH_BIN ...
            (camera='downward', classes='bin_a')
```

---

## 11. Testing Vision-Guided FSMs Without Hardware

### Mock detection sequence

Simulate detections by overriding `detected()` on the mock:

```python
from unittest.mock import MagicMock, patch
from yasmin import Blackboard

detection_seq = ['', '', '', 'object_a', 'object_a', 'object_a']

def mock_detected(target, camera=None, stale_after=1.0):
    if not detection_seq:
        return False
    val = detection_seq.pop(0)
    return val == target

duburi = MagicMock()
duburi.detected.side_effect = mock_detected
duburi.vision.home.return_value = MagicMock(success=True)
```

### Test avoid branch fires

```python
def test_avoid_branch_taken_when_object_b_detected():
    # First call returns False (no avoid), second returns True (avoid fires)
    duburi = MagicMock()
    duburi.detected.side_effect = [False, False, True]  # 3rd poll: object_b seen
    profile = VehicleProfile.dubomini()
    state = SearchAWhileAvoidingState(duburi, profile)
    outcome = state.execute(Blackboard())
    assert outcome == 'avoid'
```

### Test DVL vs timed in pick approach

```python
def test_approach_uses_dvl_on_duburi45():
    duburi = MagicMock()
    duburi.vision.home.return_value = MagicMock(success=True)
    state = VisionHomeState(duburi, VehicleProfile.duburi45(), target='object_c',
                             yaw=True, lat=True, forward=True, dist=0.15)
    state.execute(Blackboard())
    # VisionHomeState delegates entirely to vision.home — dist passed correctly
    _, kw = duburi.vision.home.call_args
    assert kw['dist'] == pytest.approx(0.15)
```
