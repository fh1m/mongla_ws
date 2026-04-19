# Mission Design & State Machine Architecture — Duburi AUV

> **Top note:** **YASMIN FSM is the TDR target architecture.** It is **not**
> what this repo runs today. Today's mission entry points live in
> [`src/duburi_planner/duburi_planner/missions/`](../../src/duburi_planner/duburi_planner/missions/)
> — plain Python scripts (`square_pattern.py`, `arc_demo.py`,
> `heading_lock_demo.py`) that use `DuburiClient` (a blocking
> `ActionClient` wrapper from `duburi_planner`) against the
> `/duburi/move` action. Run them via `ros2 run duburi_planner mission
> <name>`.
>
> Re-introduce YASMIN (or behavior trees, or `py_trees_ros`) when
> missions outgrow a linear script — typically when conditional
> branches, retries, or vision-driven detours show up. The slot for that
> code is [`src/duburi_planner/duburi_planner/state_machines/`](../../src/duburi_planner/duburi_planner/state_machines/),
> which is intentionally empty until needed.
>
> This file documents the *target* FSM design (good for planning) and
> the *current* mission-script pattern (good for picking up the codebase
> as it is today).

Based on the 2025 RoboSub competition codebase (YASMIN FSM) + 2023 patterns.

---

## 1. Competition Context

### RoboSub Task Structure
Each task has:
- A **detection target** (gate, buoy, bin, torpedo board, bottle)
- An **approach phase** (navigate to task area)
- An **execution phase** (vision-guided alignment + action)
- A **completion signal** (passed through / torpedo hit / grabbed)

### Our Competition History
- 2023: 2nd world — gate, buoy, bin tasks completed
- 2025: 8th world — gate, slalom, bin, torpedo, octagon bottle tasks
- Key insight: **robust control beats complex missions** — a vehicle that moves predictably
  can complete more tasks than one that moves unpredictably but has smart vision

---

## 2. FSM Architecture (YASMIN)

We use YASMIN (Yet Another State MachINe) — a ROS2-native state machine library.

### State Base Class

```python
from yasmin import State, Blackboard

class DuburiState(State):
    """Base class for all mission states."""
    
    TIMEOUT_S = 60.0   # Default state timeout — override per state
    
    def __init__(self, node, outcomes):
        super().__init__(outcomes=outcomes + ['timeout', 'abort'])
        self._node = node     # Reference to driver/control node
        
    def execute(self, blackboard: Blackboard) -> str:
        self._start_time = time.time()
        return self._run(blackboard)
    
    def _run(self, blackboard: Blackboard) -> str:
        raise NotImplementedError
    
    def _timed_out(self) -> bool:
        return time.time() - self._start_time > self.TIMEOUT_S
    
    def _fail_safe(self):
        self._node.stop_thrusters()
        self._node.set_mode("SURFACE")
```

### Standard Outcomes

```python
SUCCEED  = "succeeded"    # Task completed successfully
FAILED   = "failed"       # Task failed (can retry)
TIMEOUT  = "timeout"      # Time limit exceeded
ABORT    = "abort"        # Critical failure, surface now
```

### Mission Blackboard (shared state between states)

```python
# Blackboard is a dict-like shared memory between states
blackboard["start_heading"] = initial_heading    # Set at mission start
blackboard["gate_heading"]  = 63.0              # Compass heading to gate
blackboard["gate_depth"]    = 0.60              # Competition depth (meters)
blackboard["last_detection"] = None              # Vision detection result
blackboard["dvl_position"]  = (0.0, 0.0)        # Running position estimate
blackboard["mission_time"]  = time.time()        # Mission clock
```

---

## 3. State Templates

### ARM State

```python
class ArmState(DuburiState):
    TIMEOUT_S = 30.0
    
    def __init__(self, node):
        super().__init__(node, [SUCCEED, FAILED])
    
    def _run(self, bb: Blackboard) -> str:
        self._node.set_mode("MANUAL")
        time.sleep(1)
        self._node.arm()
        time.sleep(2)
        
        # Verify armed
        if self._node.is_armed():
            bb["start_heading"] = self._node.get_heading()
            return SUCCEED
        return FAILED
```

### SET DEPTH State

```python
class SetDepthState(DuburiState):
    TIMEOUT_S = 30.0
    
    def __init__(self, node, depth_m: float, tolerance_m: float = 0.15):
        super().__init__(node, [SUCCEED, TIMEOUT])
        self._target = depth_m
        self._tolerance = tolerance_m
    
    def _run(self, bb: Blackboard) -> str:
        self._node.set_mode("ALT_HOLD")
        self._node.set_depth(self._target)
        
        while not self._timed_out():
            actual = self._node.get_depth()
            if actual and abs(actual - self._target) < self._tolerance:
                time.sleep(1.0)   # Stabilize
                return SUCCEED
            time.sleep(0.1)
        return TIMEOUT
```

### SET HEADING State

```python
class SetHeadingState(DuburiState):
    TIMEOUT_S = 30.0
    
    def __init__(self, node, heading_deg: float, tolerance_deg: float = 5.0):
        super().__init__(node, [SUCCEED, TIMEOUT])
        self._target = heading_deg
        self._tolerance = tolerance_deg
    
    def _run(self, bb: Blackboard) -> str:
        while not self._timed_out():
            current = self._node.get_heading()
            err = heading_error(self._target, current)
            
            if abs(err) < self._tolerance:
                return SUCCEED
            
            self._node.set_attitude(0, 0, self._target)
            time.sleep(0.1)
        return TIMEOUT
```

### TIMED MOVE State

```python
class TimedMoveState(DuburiState):
    def __init__(self, node, direction: str, duration: float,
                 speed_pct: float = 80, maintain_heading: bool = True):
        super().__init__(node, [SUCCEED])
        self.TIMEOUT_S = duration + 5.0
        self._direction = direction
        self._duration = duration
        self._speed = speed_pct
        self._maintain_heading = maintain_heading
    
    def _run(self, bb: Blackboard) -> str:
        start = time.time()
        locked_heading = self._node.get_heading()
        
        while time.time() < start + self._duration:
            pwm = pct_to_pwm(self._speed)
            
            if self._direction == "forward":
                self._node.send_rc(forward=pwm)
            elif self._direction == "lateral_right":
                self._node.send_rc(lateral=pwm)
            # ... etc
            
            if self._maintain_heading:
                self._node.set_attitude(0, 0, locked_heading)
            
            time.sleep(0.05)   # 20 Hz
        
        self._node.stop_thrusters()
        return SUCCEED
```

### VISION ALIGN State (pattern from 2025 codebase)

```python
class VisionAlignState(DuburiState):
    TIMEOUT_S = 60.0
    THRESHOLD_LAT = 15    # px
    THRESHOLD_VERT = 40   # px
    CONFIRM_FRAMES = 5
    
    def __init__(self, node, target_class: str, next_action: str):
        super().__init__(node, [SUCCEED, TIMEOUT, FAILED])
        self._target = target_class
        self._next = next_action
        self._confirm_count = 0
    
    def _run(self, bb: Blackboard) -> str:
        while not self._timed_out():
            det = bb.get("last_detection")
            
            if det is None or det.class_name != self._target:
                # Search: rotate slowly
                self._node.send_rc(yaw=pct_to_pwm(15))
                self._confirm_count = 0
                time.sleep(0.05)
                continue
            
            # Compute centering errors
            err_x = det.cx - 320   # Frame center 640px wide
            err_y = det.cy - 240   # Frame center 480px tall
            
            centered = (abs(err_x) < self.THRESHOLD_LAT and
                       abs(err_y) < self.THRESHOLD_VERT)
            
            if centered:
                self._confirm_count += 1
                if self._confirm_count >= self.CONFIRM_FRAMES:
                    return SUCCEED
            else:
                self._confirm_count = 0
                # Correct: proportional to error
                lat_speed = int(err_x / 320 * 60)
                vert_speed = int(-err_y / 240 * 60)
                self._node.send_rc(
                    lateral=pct_to_pwm(lat_speed / 100 * 60),
                    throttle=pct_to_pwm(vert_speed / 100 * 60))
            
            time.sleep(0.05)
        
        return TIMEOUT
```

---

## 4. Building the Full Mission (from 2025 competition structure)

```python
from yasmin import StateMachine
import yasmin_ros

def build_mission_sm(node) -> StateMachine:
    sm = StateMachine(outcomes=[SUCCEED, ABORT])
    
    sm.add_state("ARM", ArmState(node),
                 transitions={SUCCEED: "SET_MISSION_DEPTH", FAILED: "ARM"})
    
    sm.add_state("SET_MISSION_DEPTH",
                 SetDepthState(node, depth_m=-0.6, tolerance_m=0.15),
                 transitions={SUCCEED: "SET_GATE_HEADING", TIMEOUT: "SET_MISSION_DEPTH"})
    
    sm.add_state("SET_GATE_HEADING",
                 SetHeadingState(node, heading_deg=63.0, tolerance_deg=5.0),
                 transitions={SUCCEED: "APPROACH_GATE", TIMEOUT: "ABORT"})
    
    sm.add_state("APPROACH_GATE",
                 TimedMoveState(node, "forward", duration=8.0, speed_pct=60),
                 transitions={SUCCEED: "ALIGN_GATE"})
    
    sm.add_state("ALIGN_GATE",
                 VisionAlignState(node, "gate", next_action="PASS_GATE"),
                 transitions={SUCCEED: "PASS_GATE",
                              TIMEOUT: "APPROACH_GATE",   # retry approach
                              FAILED: "ABORT"})
    
    sm.add_state("PASS_GATE",
                 TimedMoveState(node, "forward", duration=3.0, speed_pct=80),
                 transitions={SUCCEED: "SURFACE"})
    
    sm.add_state("SURFACE", SurfaceState(node),
                 transitions={SUCCEED: SUCCEED})
    
    return sm
```

---

## 5. Competition-Specific Parameters (Robosub 2025 Calibrated Values)

```python
# These worked at competition — adjust after each pool test
MISSION_PARAMS = {
    # Depths
    "gate_depth":       -0.60,   # meters (negative = below surface)
    "slalom_depth":     -0.60,
    "bin_depth":        -0.80,
    "torpedo_depth":    -0.60,
    "bottle_depth":     -0.50,
    
    # Headings (competition-specific compass headings)
    "gate_heading":     63.0,    # degrees, pool-specific
    
    # Speeds (% of max, 0-100)
    "forward_speed":    80,
    "search_speed":     30,
    "align_speed":      40,
    "torpedo_speed":    50,
    
    # Vision thresholds (pixels, 640x480 frame)
    "threshold_lat":    15,      # lateral centering tolerance
    "threshold_vert":   40,      # vertical centering tolerance
    "confirm_frames":   5,       # consecutive frames to confirm detection
    
    # Timeouts (seconds)
    "approach_timeout": 30,
    "align_timeout":    60,
    "mission_timeout":  600,     # Total mission time
    
    # DVL calibration
    "dvl_depth_match":  0.78,    # Offset from DVL depth to true depth
    
    # Autonomous mission timer
    "delayed_start_s":  10,      # Seconds after code start before vehicle moves
}
```

---

## 6. Mission Launch Pattern (Autonomous With Delayed Start)

```python
import time

DELAYED_START_S = 10   # Operator removes tether during this window

def run_mission():
    print(f"Mission starting in {DELAYED_START_S} seconds...")
    print("REMOVE TETHER NOW")
    
    for i in range(DELAYED_START_S, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    
    print("MISSION START")
    
    # Build and run state machine
    sm = build_mission_sm(control_node)
    sm.execute(Blackboard())
```

---

## 7. DVL vs Firmware Depth — Strategy

```python
USE_DVL = True   # Set to False if DVL not connected or malfunctioning

if USE_DVL:
    # DVL provides precise depth via Nucleus1000
    # Use custom PID in dvl_node
    # More accurate, handles pressure variations
    control_mode = "dvl_depth_hold"
else:
    # ArduSub ALT_HOLD uses Bar30 barometric sensor via EKF3
    # Less precise but reliable fallback
    control_mode = "ardusub_alt_hold"
```

---

## 8. Safety State (Always Available)

```python
class EmergencySurfaceState(DuburiState):
    """Called on ABORT or SIGINT. Always surfaces vehicle."""
    
    def _run(self, bb: Blackboard) -> str:
        self._node.stop_thrusters()
        self._node.set_mode("SURFACE")
        
        # Wait until at surface (depth > -0.1m)
        timeout = time.time() + 60
        while time.time() < timeout:
            depth = self._node.get_depth()
            if depth and depth > -0.15:
                break
            time.sleep(0.5)
        
        self._node.disarm()
        return SUCCEED

# Register as SIGINT handler:
import signal
def _sigint_handler(sig, frame):
    emergency_surface_state._run(blackboard)
    sys.exit(0)
signal.signal(signal.SIGINT, _sigint_handler)
```
