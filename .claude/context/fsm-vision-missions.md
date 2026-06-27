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
| **Vision-as-control** | Continuously centres / approaches a target via closed-loop | `duburi.vision.align(...)` / `duburi.vision.move(...)` | Yes — runs until reached/timeout |

```
Vision-as-TRIGGER (inside a state's _run loop):

  while not detected('gate'):
      move_forward(0.5)        ← open-loop step
  return SUCCEED               ← gate seen → next state


Vision-as-CONTROL (hand off to DSL):

  result = duburi.vision.align('gate', yaw=0, lat=0)
  return SUCCEED if result.ok else FAILED
  ← DSL runs a 20 Hz P-loop until every active axis is within
    err px for vision.align_stable_frames ticks
```

**Rule:** use `detected()` to decide *when* to change state; use
`vision.align` (centre) and `vision.move` (approach) to do the actual
alignment work inside a dedicated state — `VisionAlignState` /
`VisionMoveState` wrap them, `VisionSearchState` wraps the search.

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
| **Approach to standoff** | `vision.move(fwd=40, mode='area')` | same | Vision-measured (bbox fill) — DVL irrelevant |
| **Lateral clear of obstacle** | `move_lateral_dist(0.8m, gain=50)` | `move_right(1.5s, gain=50)` | Lateral precision matters less |
| **Return through gate** | `move_forward_dist(1.5m, gain=80)` | `move_forward(3.0s, gain=80)` | |
| **Depth change** | `set_depth(-5.0m)` | same | ArduSub ALT_HOLD always; DVL irrelevant |
| **Yaw search sweep** | `yaw_right(20°)` × N | same | Heading-based; DVL irrelevant |
| **Hold position** | DVL auto-helps via EKF | POSHOLD not available | Duburi can hold; Dubomini drifts |
| **Forward search steps** | `move_forward(0.5s, gain=30)` | same | Short steps — precision irrelevant |
| **Orbit around target** | `VisionSearchState(pattern='yaw')` / `yaw_right(20°)` + `detected()` | same | Vision-guided; DVL irrelevant |
| **Pick approach** | `vision.move(fwd=X, mode='height')` | same | Vision-measured (bbox fill) |
| **Drop manoeuvre** | `move_lateral_dist` or stay in place | timed lateral | DVL for precision |

**Core rule: any open-loop distance move should use DVL when available.
Vision-closed-loop moves (`align`, `move`) are DVL-agnostic.**

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

### 4.2 Yaw sweep with trigger

```
AUV rotates in place (POSHOLD), stops every N° to observe.
Exits on first detection.

       20°   20°   20°   20°
    ──▶rot──▶rot──▶rot──▶[DETECTED]──▶ next state

Best for: post-pass searching for next task object, orbit around marker.
```

Use `VisionSearchState(duburi, profile, target='flare', pattern='yaw', yaw_step=20, timeout=90)`.

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

Two states: `SetDepthState(-5.0)` → `VisionSearchState(pattern='yaw')` or `MarchSearchState`.

### 4.5 Standoff approach (vision-measured distance)

```
Drive forward until bbox fills the frame. DVL-agnostic.

    ────────────────────────▶ [bbox grows] ──▶ SUCCEED when fill ≥ fwd%
    vision.move(fwd=40, mode='area')
```

Use `VisionMoveState(target='gate', fwd=40, mode='area')`.
For bins seen from above, centre first with
`VisionAlignState(target='bin', lat=True, depth=True)`, then approach.

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
SEARCH (march forward)                VisionSearchState
  │ SUCCEED
ALIGN (yaw + lat, centre on 0,0)      VisionAlignState
  │ SUCCEED
MOVE (drive to fwd=80, mode='height') VisionMoveState
  │ SUCCEED (fill reached)
PASS (DVL 3.5m or timed 5s)           MoveForwardState
```

State graph:
```
SEARCH_GATE ──SUCCEED──▶ ALIGN_GATE ──SUCCEED──▶ MOVE_GATE ──SUCCEED──▶ PASS_GATE
    ▲                        │FAILED               │FAILED
    └────────────────────────┴──────────────────────┘ (lost target → re-search)
```

### 5.2 Orbit + Confirm

Fly around a marker to confirm its nature or score points:

```
SEARCH → ALIGN (yaw + lat) → ORBIT (yaw sweep 360°) → SCORE
```

The orbit is a `VisionSearchState(pattern='yaw')` (or an open-loop
`yaw_right` + `detected()` loop) that stops every `yaw_step`° to re-detect.

### 5.3 Depth Dive → Search Below

Target is at known depth (bin, path marker):

```
SET_DEPTH(-5.0m)
  │ SUCCEED
SCAN_AT_DEPTH (VisionSearchState, pattern='yaw')
  │ SUCCEED
ALIGN_ON_TARGET (VisionAlignState, yaw + lat)
  │ SUCCEED
MOVE_ON_TARGET (VisionMoveState, fwd fill)
```

### 5.4 Pick Sequence

```
ALIGN (VisionAlignState, yaw + lat + depth, centre on object)
  │ SUCCEED
APPROACH (VisionMoveState, fwd=high %, mode='height')
  │ SUCCEED
ACTUATE_GRAB (ESP32 grab cmd)
  │
CONFIRM_PICK (poll current or bbox disappear)
  │ SUCCEED | TIMEOUT (assume picked)
ASCEND (set_depth to drop depth)
```

### 5.5 Drop Sequence

```
SEARCH_BIN (yaw sweep looking down-camera)
  │ SUCCEED
LOCK_BIN (VisionAlignState lat + depth, centre over bin in down-cam)
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
ALIGN_A (VisionAlignState yaw+lat, centre object_a)
    │ SUCCEED
    │ FAILED → SEARCH_A (lost object_a → re-search)
    │
MOVE_A (VisionMoveState fwd=high %, mode='height')
    │ SUCCEED
    │ FAILED → SEARCH_A (lost object_a → re-search)
    │
DEEP_DIVE (-5.0m)
    │ SUCCEED
    │ TIMEOUT → SURFACE
    │
SCAN_C (VisionSearchState pattern='yaw', down-cam or forward-cam)
    │ SUCCEED
    │ TIMEOUT → SURFACE
    │
ALIGN_C (VisionAlignState yaw+lat+depth, centre object_c)
    │ SUCCEED
    │ FAILED → SCAN_C
    │
MOVE_C (VisionMoveState fwd=high %, mode='height')
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
SEARCH_BIN (VisionSearchState pattern='yaw', down-camera, target='bin_a')
    │ SUCCEED
    │ TIMEOUT → SURFACE_WITH_OBJ (ascend and keep object)
    │
LOCK_BIN (VisionAlignState lat+depth from above, centre over bin)
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
    """Centre above bin using the downward camera.
    lat + depth centre the bin (down-cam: ex→lateral, ey→depth nudge).
    """
    TIMEOUT_S = 30.0

    def __init__(self, duburi, profile, target='bin_a',
                 camera='downward', duration=20.0):
        super().__init__(duburi, profile, [SUCCEED, FAILED])
        self._target   = target
        self._camera   = camera
        self._duration = duration

    def _run(self, bb):
        # align: lat + depth to centre the bin in the down-cam (0 = centre).
        # A miss returns a non-ALIGNED VisionResult; never raises.
        result = self.duburi.vision.align(
            self._target,
            camera=self._camera,
            lat=0,                 # centre laterally
            depth=0,               # centre fore/aft (down-cam ey → depth nudge)
            err=30,
            gain=30,
            duration=self._duration,
        )
        return SUCCEED if result.ok else FAILED
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
from ..states.vision import VisionSearchState, VisionAlignState, VisionMoveState
from ..states.utility import CountdownState, LogScoreState

# Import custom states defined above (same file or imported)
# from ..states.custom import (SearchAWhileAvoidingState, AvoidBState,
#     ConfirmPickState, LockBinState, ConfirmDropState, PickActuateState, DropActuateState)

PICK_DROP_DEFAULTS = {
    'countdown_s':     10,
    'search_depth_m': -1.2,    # depth while searching for object_a
    'deep_depth_m':   -5.0,    # depth for object_c scan
    'drop_depth_m':   -1.0,    # depth for bin approach
    'approach_fill':   40,     # vision.move fwd fill % for object_a standoff
    'pick_fill':       70,     # vision.move fwd fill % for grabber approach
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
                 transitions={SUCCEED: 'ALIGN_A',
                               'avoid': 'AVOID_B',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('AVOID_B',
                 AvoidBState(duburi, profile),
                 transitions={SUCCEED: 'SEARCH_A',   # ← loops back!
                               ABORT: 'SURFACE'})

    # ── approach object_a: centre (align) then drive in (move) ────────
    sm.add_state('ALIGN_A',
                 VisionAlignState(duburi, profile, target='object_a',
                                  yaw=True, lat=True,
                                  err=40, gain=30, duration=20.0),
                 transitions={SUCCEED: 'MOVE_A',
                               FAILED: 'SEARCH_A',    # lost → re-find
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('MOVE_A',
                 VisionMoveState(duburi, profile, target='object_a',
                                 fwd=p['approach_fill'], mode='height',
                                 gain=35, duration=20.0),
                 transitions={SUCCEED: 'DEEP_DIVE',
                               FAILED: 'SEARCH_A',    # lost → re-find
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── deep dive + scan for object_c ─────────────────────────────────
    sm.add_state('DEEP_DIVE',
                 SetDepthState(duburi, profile, depth_m=p['deep_depth_m']),
                 transitions={SUCCEED: 'SCAN_C', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('SCAN_C',
                 VisionSearchState(duburi, profile, target='object_c',
                                   pattern='yaw', yaw_step=20.0, timeout=120.0),
                 transitions={SUCCEED: 'ALIGN_C', TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    # ── lock on object_c for pick: align (3-axis) then move in ────────
    sm.add_state('ALIGN_C',
                 VisionAlignState(duburi, profile, target='object_c',
                                  yaw=True, lat=True, depth=True,
                                  err=30, gain=30, duration=20.0),
                 transitions={SUCCEED: 'MOVE_C',
                               FAILED: 'SCAN_C',
                               TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})

    sm.add_state('MOVE_C',
                 VisionMoveState(duburi, profile, target='object_c',
                                 fwd=p['pick_fill'], mode='height',
                                 gain=25, duration=20.0),
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
                 VisionSearchState(duburi, profile, target='bin_a',
                                   camera='downward', pattern='yaw',
                                   yaw_step=20.0, timeout=90.0),
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
        'approach_fill':  40,    # vision.move fwd fill % for object_a
    }

    set_ros_loggers()
    sm = build_pick_drop_fsm(duburi, profile, params=params)
    outcome = sm(Blackboard())
    log(f'[FSM] result: {outcome}')
```

---

## 7. Vision Parameter Tuning Reference

### 7.1 `vision.align` axis selection

`align` takes `lat` / `yaw` / `depth`: each is `None` (axis off) or a signed
**pixel offset** from centre (`0` = centre, `+` = right/below, `-` =
left/above). At least one axis. (FSM `VisionAlignState` also accepts `True`
= centre/offset 0 and `False`/`None` = off.) `vision.move` owns the forward
axis separately.

| Situation | `yaw` | `lat` | `depth` | Forward (separate `vision.move`) | Notes |
|---|---|---|---|---|---|
| Centre gate in frame | ✓ | ✓ | — | — | Depth held by ArduSub; no vertical correction |
| Gate pass | ✓ | ✓ | — | `move(fwd=80, mode='height')` | align first, then move through |
| Flare approach | ✓ | — | ✓ | `move(fwd=38, mode='height')` | yaw + depth centre, then close in |
| Object approach for pick | ✓ | ✓ | ✓ | `move(fwd=70, mode='height')` | 3-axis centre, then drive in |
| Lock above bin | — | ✓ | ✓ | — | Down-cam; lat + depth centre |
| Slalom pipe (offset) | ✓ | `±80` | — | `move(fwd=60, mode='height', maintain=±80)` | hold pipe off-centre while passing |

### 7.2 `vision.move` `fwd` fill + `mode` per object

`move` drives forward until the bbox fills `fwd` % of the frame, measured by
`mode` (`area` / `width` / `height`). No more `dist` / `metric` knobs.

| Object | Camera | `mode` | `fwd` % | Rationale |
|---|---|---|---|---|
| Gate | forward | `height` | 80 | Pass when the bar fills 80% of frame height |
| Flare / buoy | forward | `height` | 38–42 | Height more stable for tall targets |
| Pick object | forward | `height` | 60–75 | Close approach; height = real size proxy |
| Slalom pipe | forward | `height` | 55–65 | Tall thin target; pair with `maintain=±px` |
| Torpedo board | forward | `width` | 30–35 | Width = board face width |
| Path marker | downward | `area` | n/a | Centre with `align`; `move` rarely needed |

### 7.3 Target loss — the `fallback` search function

There is no `on_lost` knob. By default a verb coasts for
`vision.lost_grace_s` and then returns a non-`ALIGNED` `VisionResult` (the
state maps it to `FAILED`, so the plan routes back to a search state). Pass a
mission-authored `fallback=fn(duburi)` / `fn(duburi, should_stop)` to recover
*inside* the same state — the verb runs the search once, then re-enters, all
within `duration`:

| Recovery | How | Use for |
|---|---|---|
| Re-search at plan level | no `fallback`; route `FAILED` → search state | Production missions — explicit re-search on loss |
| Recover in-state | pass `fallback=creep_forward` (or a yaw sweep) | Brief dropouts; keep the same state active |

### 7.4 `kp_*` gain tuning guide

Gains are live-tunable via `ros2 param set /duburi_manager vision.kp_yaw 80.0`
(applied on the NEXT goal). Defaults live in `vision_tunables.py`.

| Gain | Too low symptom | Too high symptom | Default |
|---|---|---|---|
| `kp_yaw` | Slow, drifts off-axis during forward motion | Oscillates yaw, never settles | 60 |
| `kp_lat` | Misses lateral centre; passes offset | Jerky strafe, oscillates | 60 |
| `kp_depth` (m/tick) | Misses vertical centre | AUV bobs up/down | 0.05 |
| `kp_forward` | Stays far; doesn't approach | Overshoots, crashes | 200 |

Tune order: `kp_yaw` first (most critical), then `kp_lat`, then `kp_depth`, then `kp_forward`.

Other live `vision.*` params: `vision.lost_grace_s` (1.0 s coast before
LOST), `vision.frame_fill_default` (95 %, the `move` fill used when a mission
leaves `fwd` at 0), `vision.align_stable_frames` (3 ticks every active axis
must stay within `err` px before `align` reports `ALIGNED`).

### 7.5 Forward approach = `vision.move` (replaces the old `vis_approach`)

> **Removed:** the monocular-depth `vis_approach` verb (and its `vis_range`
> threshold / `lock_mode`) no longer exist. Forward approach is now a pure
> bbox-fill `vision.move`.

Drive forward until the target fills the frame, then hand off to `vision.align`
for fine centring if needed:

```python
# Coarse approach, then fine centre
duburi.vision.move('gate', fwd=70, mode='height', gain=35, duration=30)
duburi.vision.align('gate', yaw=0, lat=0, err=20, gain=20, duration=15)
```

FSM state usage wraps the same DSL verb:

```python
# In a plan:
sm.add_state('APPROACH_GATE',
             VisionMoveState(duburi, profile, target='gate',
                             fwd=70, mode='height', duration=30),
             transitions={SUCCEED: 'CENTRE_GATE',
                           FAILED:  'SCAN_FOR_GATE',
                           TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})
```

### 7.6 `err` (pixel tolerance) tuning

`err` is the per-axis pixel tolerance: `align` counts an axis "centred" only
once `|error| ≤ err` px, and reports `ALIGNED` after every active axis holds
that band for `vision.align_stable_frames` ticks. Too tight = chattery / slow
to settle, too loose = sloppy centre. (This replaces the old `deadband`
fraction.)

| Object | `err` (px) | Why |
|---|---|---|
| Gate | 40 | Large target; coarse centering fine |
| Buoy / flare | 30 | Medium target |
| Pick object | 14–20 | Tight precision required for grabber |
| Bin (from above) | 30 | Precision needed but not extreme |

---

## 8. State Outcome Vocabulary (complete reference)

| Outcome | When returned | Typical plan transition |
|---|---|---|
| `SUCCEED` | Task completed successfully | Next task state |
| `FAILED` | Vision verb returned non-`ALIGNED` (target lost / not reached) | Re-search state (avoid surfacing for recoverable error) |
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
- [ ] `VisionAlignState` / `VisionMoveState` route `FAILED` to a re-search state (or pass `fallback=` for in-state recovery)
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

## 11. Payload Fire Patterns (torpedo + dropper)

### 11.1 Torpedo task — align then fire

There is no lock-fire verb. Centre the target with `vision.align` (tight `err`,
slow `gain`); when it returns `ALIGNED`, call `duburi.fire(channel)`
(1/2 = torpedo, 3/4 = dropper). `VisionResult` is truthy only on a real lock,
so `if align(...).ok:` gates the shot.

**As a standalone mission call** (inside `detected()`-paradigm or FSM state body):

```python
duburi.models(torpedo='torpedo_blood_hole')   # classes: torpedo, blood, hole

# Coarse board align, then a precise hole lock before firing
duburi.vision.align('torpedo', yaw=0, lat=0, depth=0,
                    err=40, gain=30, duration=20, fallback=creep_forward)

duburi.set_classes('hole')
# yaw_gain low: a 20 kg hull needs the yaw inertia fought gently to hold a tight
# hole-lock steady enough to fire through (lat/depth stay at the global gain).
# hold=2.0: ACTIVE station-keep -- once centred, keep correcting on the hole for
# 2 s so the hull is held dead-still against water inertia BEFORE the shot
# (exiting the instant it's centred leaves nothing fighting drift -> the hull
# walks off-aim). hold counts against duration, so budget duration >= approach +
# hold. brake=False on the FIRE path: the arrival brake is self-gating and usually
# skips a gently-converged lock, but a fast snap-in can still emit a 0.2 s kick
# right before align() returns -- which would nudge the hull off-aim between
# lock-confirm and fire(). On a fire-from-lock the lateral brake buys nothing
# (you are not moving away), so disable it to keep the shot dead still.
if duburi.vision.align('hole', yaw=0, lat=0, depth=0,
                       err=14, gain=12, yaw_gain=8, brake=False, hold=2.0,
                       duration=25, fallback=creep_forward).ok:
    duburi.fire(1)           # torpedo_1 (ESP32 serial 1/2)
```

> **Per-axis gain for stable holds.** `lat_gain`/`yaw_gain`/`depth_gain` cap one
> axis independently of the global `gain` (unset = inherit it). Yaw is the axis to
> slow down for a torpedo hole-lock: far from the target a brisk yaw overshoots and
> wobbles, so dial `yaw_gain` low (≈8–12) for a slow, settle-able correction while
> `lat`/`depth` stay responsive. The yaw spin-up floor only engages once the bbox is
> large (close), so a low `yaw_gain` far out stays pure-proportional and won't
> limit-cycle.

> **Inertial arrival brake (`brake=True`, default).** Vision verbs reverse-kick on
> arrival to bleed water inertia so a step ends at a predictable, drift-free
> position — `align` brakes lateral, `move` brakes forward (+`maintain`) on a
> fill-stop. It is self-gating on the exit-velocity EMA: a gently-converged lock
> that ramps down into the band usually exits with ~0 momentum and is **not**
> kicked. But a *fast snap-in* (sustained high lateral drive then an abrupt
> centre, more likely at higher `gain`) can still cross the gate and emit a 0.2 s
> kick — which on the torpedo path lands between lock-confirm and `fire()`. So on
> a **fire-from-lock pass `brake=False`** (above): you are not moving away, so the
> lateral brake buys nothing and only risks the shot. Pass-through
> (`move(fwd=None)`) and abort/loss never brake. Yaw/depth never brake.

> **Active station-keep (`hold=`s).** Without `hold`, `align` exits the instant it's
> centred and goes neutral — so on a payload step the hull immediately drifts off-aim
> on water inertia (it has "nothing to stay stable against"). `hold=N` keeps the
> alignment loop *alive and correcting* (lat/yaw/depth) for N seconds after first
> centring, fighting drift, then exits `ALIGNED` while in-band — the corrections are
> the stability. Use it to hold steady through a torpedo/dropper shot
> (`align('hole', …, hold=2.0, brake=False)` then `fire()`). It counts against
> `duration` (budget `duration ≥ approach + hold`, else it `TIMEOUT`s mid-hold and a
> `if align(hold=…): fire()` skips the shot). It holds **lat/yaw/depth only** — not
> forward range (the prior `move` set the standoff). This is the *active* cousin of
> the passive `settle` neutral-hold; `settle` is exactly what fails here.

**As an FSM state** (`VisionAlignState` for the lock, a small fire state for the shot):

```python
sm.add_state('LOCK_HOLE',
             VisionAlignState(duburi, profile, target='hole',
                              yaw=True, lat=True, depth=True,
                              err=14, gain=12, duration=25),
             transitions={SUCCEED: 'FIRE_TORPEDO',
                           FAILED:  'SCAN_BOARD',
                           TIMEOUT: 'SURFACE', ABORT: 'SURFACE'})
# FireState._run() calls duburi.fire(1) and returns SUCCEED.
```

> `duburi.fire(ch)`: 1/2 = torpedo, 3/4 = dropper (ESP32 serial).
> Call `duburi.payload_ready()` before the mission if you need a hard gate.

---

### 11.2 Bin drop — downward camera + align then drop

Axis remap: with `camera='downward'`, `ex`→lateral and `ey`→the depth nudge.
Centre with `lat` + `depth`, then drop.

```python
duburi.set_depth(-1.5)
duburi.use_camera('downward')
duburi.models(bin='bin_fire_blood')          # classes: blood, fire
duburi.set_classes('fire,blood')

# Search the down-cam for the bin marker, then centre and drop
for _ in range(30):
    if duburi.detected('fire', camera='downward', stale_after=0.5):
        break
    duburi.move_forward(0.5, gain=30)

if duburi.vision.align('fire', camera='downward', lat=0, depth=0,
                       err=30, gain=30, duration=25, fallback=creep_forward).ok:
    duburi.fire(3)           # dropper_1 (ESP32 serial 3/4)
```

---

### 11.3 Offset alignment (slalom / off-centre aim)

`align` takes the offset directly: a signed pixel value on `lat` / `yaw` /
`depth` holds the target that many px off centre (no separate `offset_x/y`).
`vision.move` holds a lateral offset while driving with `maintain=±px`.

```python
# Slalom — keep the red pipe 80 px to the right, then drive past it
duburi.models(slalom='slalom_red_pipe')      # class: red_pipe
duburi.vision.align('red_pipe', yaw=0, lat=80, err=40, gain=30, duration=20)
duburi.vision.move('red_pipe', fwd=60, mode='height', maintain=80,
                   gain=35, duration=15)

# Torpedo board — aim right + above the bbox centre
# (+lat/+yaw = right, -depth = above centre)
duburi.vision.align('torpedo', yaw=60, lat=0, depth=-40,
                    err=20, gain=20, duration=15)
```

---

## 12. Testing Vision-Guided FSMs Without Hardware

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
# Vision verbs return a VisionResult; .ok is the truthy success flag.
duburi.vision.align.return_value = MagicMock(ok=True)
duburi.vision.move.return_value = MagicMock(ok=True)
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

### Test the move verb gets the right fill target

```python
def test_move_passes_fill_to_vision():
    duburi = MagicMock()
    duburi.vision.move.return_value = MagicMock(ok=True)
    state = VisionMoveState(duburi, VehicleProfile.duburi45(), target='object_c',
                            fwd=70, mode='height')
    outcome = state.execute(Blackboard())
    # VisionMoveState delegates entirely to vision.move — fwd/mode passed through
    _, kw = duburi.vision.move.call_args
    assert kw['fwd'] == 70
    assert kw['mode'] == 'height'
    assert outcome == SUCCEED

def test_align_succeeds_on_ok_result():
    duburi = MagicMock()
    duburi.vision.align.return_value = MagicMock(ok=True)
    state = VisionAlignState(duburi, VehicleProfile.duburi45(), target='object_c',
                             yaw=True, lat=True, depth=True)
    assert state.execute(Blackboard()) == SUCCEED
```
