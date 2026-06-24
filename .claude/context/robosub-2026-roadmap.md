# RoboSub 2026 — Development Roadmap

**Competition date:** July 11, 2026  
**Planning start:** May 7, 2026  
**Available:** ~65 days

---

## Timeline Structure

```
May 7  ──┬── Phase 1: Base Hardening (30 days) ──> Jun 6
         │   Perception pipeline → 30 Hz
         │   Control reliability
         │   Vision-to-thrust loop tested in water
         │
Jun 6  ──┴── Phase 2: Task Development (35 days) ──> Jul 11
             Build + test each RoboSub task
             Full autonomous FSM
             Pool days every ~5 days
```

---

## RoboSub 2026 Task Coverage

Reference: https://robonation.gitbook.io/robosub-resources/section-3-autonomy-challenge

| # | Task | Status | Gap | Points (est.) |
|---|------|--------|-----|---------------|
| 0 | Heading Out — coin flip + submerge at start | ✓ arm/set_depth | Coin flip logic (random or fixed) | 100 |
| 1 | Gate — pass through 120"×60" PVC gate | ✓ `gate_flare_autonomous.py` | Style bonus (roll/pitch) — skip for now | 400 |
| 2 | Slalom — navigate 3 red/white vertical pipe sets | ✗ | Pipe detection + path logic | 300 |
| 3 | Bins — drop marker into role-specific bin | ~ dropper wired | Bin + symbol detection model | 400 |
| 4 | Torpedoes — fire through board openings | ~ torpedo wired | Board opening detection | 400 |
| 5 | Octagon — surface inside 9ft octagon, manipulate | ✗ | Acoustic pinger (hardware) | 600 |
| 6 | Return — pass back through start gate | ✓ gate detection | — | 200 |
| — | Path markers — follow ORANGE markers between tasks | ✗ | Downward cam + HSV tracker | 100 |

**Realistic target:** Tasks 0, 1, 6 + path markers = ~800 pts (safe). Stretch: + Slalom = 1100 pts.

---

## Phase 1 — Base Hardening (May 7 → June 6)

### P0 — Camera 30 Hz (DONE — committed in this session)

| Fix | File | What |
|-----|------|------|
| A1 MJPEG force | `cameras/webcam.py` | `CAP_PROP_FOURCC=MJPG` before res/fps |
| A2 Capture thread | `camera_node.py` | Daemon thread + single-slot queue |
| B1 FP16 | `cameras_.launch.py` | `half: True` in detector params |
| B3 Tracking default | `cameras_.launch.py` | `with_tracking` default → `true` |
| B2 Async inference | `detector_node.py` | `_infer_loop` worker thread |
| C1 monotonic | `heading_lock.py` | `time.time()` → `time.monotonic()` |
| C2 tracking cache | `auv_manager_node.py` | VisionState cached per `camera` (control reads `/detections`; tracker feeds the HUD only) |
| C3 ExternalShutdown | `display_node.py` | Catch `ExternalShutdownException` in spin thread |
| C4 tip log | `display_node.py` | Remove misleading `launch_pipeline` hint |

**Expected after**: camera `pub=30.0Hz`, detector `avg_infer=~7ms` (FP16), bbox tracks moving person.

### P1 — Pool Verification (first pool day, target ~May 17)

```bash
ros2 run duburi_manager bringup_check          # network + serial preflight
ros2 launch duburi_vision cameras_.launch.py   # verify: fourcc=MJPG, pub=30Hz, tracking ON
ros2 run duburi_planner mission gate_flare_autonomous
```

Confirm:
- Gate alignment centres without drift
- Bbox stays on gate as AUV approaches
- Flare detection survives partial occlusion (tracking bridge)

### P2 — DVL Integration (May 17–30)

DVL (`Nortek Nucleus1000`) is hardware-confirmed but driver is stub-only.  
**Priority**: `move_forward_dist` is the cleanest gate-pass control.

1. Test `nucleus_dvl.py` in pool — confirm AHRS + bottom-track packets arrive
2. Validate `dvl_auto_connect:=true` on Jetson Orin Nano
3. Pool-calibrate `dvl_depth_match` (currently `0.78` from 2025 comp)

---

## Phase 2 — Task Development (June 6 → July 11)

### Task 0 — Heading Out (1 day, Jun 6)

**Coin flip logic** — competition rules specify which side of gate to pass.  
Add to `gate_flare_autonomous.py` top:

```python
import random
GATE_SIDE = random.choice(['left', 'right'])   # or read from a config file set pre-run
```

Then lateral offset the gate approach accordingly (pass a small `lat=` pixel offset to `vision.align`).

No new file needed — 5-line addition to existing mission.

### Task 6 — Return Gate (2 days, Jun 8)

Already covered by `gate_flare_autonomous.py` second-pass logic.  
Verify: after flare orbit, AUV re-acquires gate from front-side and passes cleanly.

**Test**: run full mission in pool with gate + flare model.

### Orange Path Markers (4 days, Jun 10–14)

Between tasks, 4'×6" ORANGE markers on pool floor guide AUV.  
Downward camera + HSV segmentation — no YOLO needed.

**New file**: `src/duburi_planner/duburi_planner/missions/_path_follow.py`

```python
import cv2
import numpy as np

_HSV_ORANGE_LO = np.array([8,  120, 120])
_HSV_ORANGE_HI = np.array([22, 255, 255])

def _orange_centroid(frame_bgr):
    """Returns (cx_norm, cy_norm) in [-1,1] or None if no blob found."""
    hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, _HSV_ORANGE_LO, _HSV_ORANGE_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5)))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c   = max(cnts, key=cv2.contourArea)
    M   = cv2.moments(c)
    if M['m00'] < 200:   # too small
        return None
    h, w = frame_bgr.shape[:2]
    cx = (M['m10'] / M['m00']) / w * 2 - 1
    cy = (M['m01'] / M['m00']) / h * 2 - 1
    return cx, cy

def follow_orange_marker(duburi, max_steps=20, camera='downward'):
    """Lateral-align + advance on orange HSV blob from downward camera."""
    for _ in range(max_steps):
        # TODO: wire to duburi.vision once downward camera is live on Jetson
        # For now: open-loop advance (marker visible from start position)
        duburi.move_forward(0.5, gain=40)
```

**When downward camera is wired** (Jetson + Blue Robotics low-light USB): replace
open-loop with `duburi.vision.align(lat=0)` on the HSV-derived centroid published
as a synthetic `Detection2DArray`.

### Task 2 — Slalom (8 days, Jun 15–23)

Red + white vertical PVC pipes. Two approaches:

**Option A (classical, bench-testable today):**
- HSV threshold for red vertical segments + Hough lines → lateral centroid error → Ch6
- `src/duburi_planner/duburi_planner/missions/slalom.py`
- No new model, no GPU dependency

**Option B (YOLO fine-tune, higher reliability):**
- 200 pool images of red+white pipes → `yolo train data=pipes.yaml epochs=100`
- ~1h on RTX 2060 (same GPU, same pipeline as gate model)
- Model file in `src/duburi_vision/models/pipes_nano_100ep.pt`

**Recommendation**: start Option A for Jun 15 pool day. If classical fails (color inconsistency under pool lighting), schedule Option B training for week of Jun 20.

Slalom mission skeleton:
```python
# slalom.py
_PIPES = 3

def run(duburi, log):
    duburi.arm()
    duburi.set_depth(POOL_DEPTH_M, settle=2.0)

    for i in range(_PIPES):
        side = 'right' if i % 2 == 0 else 'left'
        # Find pipe — yaw-scan then lateral align
        while not duburi.detected('pipe'):
            duburi.move_forward(0.5, gain=30)
        duburi.vision.align(target='pipe', yaw=0, lat=0, duration=10)
        # Pass on correct side
        if side == 'right':
            duburi.move_right(1.5, gain=40)
        else:
            duburi.move_left(1.5, gain=40)
        duburi.move_forward(1.0, gain=50)

    duburi.set_depth(0.0)
    duburi.disarm()
```

### Task 3 — Bins (6 days, Jun 24–30)

4 bin symbols: FLAME, BLOOD DROPLET, COMPASS, HAMMER.  
Dropper wired via ESP32 serial (`duburi.fire(3)` or `duburi.fire(4)`).

**Payload verb** — `duburi.fire(3)` fires dropper_1; `duburi.fire(4)` fires dropper_2. These are already wired via ESP32 serial — no new DSL verb needed.

**Detection**: small YOLO model on bin symbols.
- Dataset: render 100 images/class from CAD + pool photos
- `yolo train data=bins.yaml model=yolo11n.pt epochs=150`
- Target: `bins_nano_150ep.pt`

**Mission skeleton**:
```python
# bins.py
TARGET_SYMBOL = 'flame'   # set by competition ops

def run(duburi, log):
    duburi.camera = 'downward'
    duburi.arm()
    duburi.set_depth(SCAN_DEPTH_M, settle=2.0)

    while not duburi.detected(TARGET_SYMBOL, camera='downward'):
        duburi.move_forward(0.5, gain=30)
    duburi.vision.align(target=TARGET_SYMBOL, camera='downward',
                        lat=0, yaw=0, duration=15)
    duburi.fire(3)   # dropper_1

    duburi.set_depth(0.0)
    duburi.disarm()
```

### Task 4 — Torpedoes (6 days, Jul 1–7)

Board openings: circular or rectangular cutouts.  
Torpedo actuator: `duburi.fire(1)` or `duburi.fire(2)` via ESP32 serial.

**Detection**: similar YOLO fine-tune on board openings.  
**Payload verb**: `duburi.fire(1)` = torpedo port, `duburi.fire(2)` = torpedo starboard — already wired via ESP32 serial.

Park until: bins model training is complete (shared GPU time), and pool test confirms torpedo actuator fires correctly.

### Full Competition FSM (4 days, Jul 8–11)

Chain tasks with timed fallbacks. Files: `missions/task_full_2026.py` (detected-paradigm) and `missions/fsm_full_2026.py` (YASMIN FSM — recommended).

```python
# task_full_2026.py — full competition chain (detected-paradigm)
POOL_DEPTH_M = -0.8

def run(duburi, log):
    duburi.arm()
    duburi.set_depth(POOL_DEPTH_M, settle=2.0)

    # Task 1: Gate
    while not duburi.detected('gate'):
        duburi.move_forward(0.5, gain=30)
    duburi.vision.align(target='gate', yaw=0, lat=0, duration=20)
    duburi.vision.move(target='gate', fwd=40, mode='area', duration=20)
    duburi.move_forward_dist(distance_m=3.0, gain=60)
    log('GATE PASS DONE')

    # Path marker → Task 2 area
    follow_orange_marker(duburi, camera='downward', max_steps=30)

    # Task 2: Slalom (timed fallback if no pipe detected after 20 steps)
    slalom_steps = 0
    for i in range(3):
        for _ in range(20):
            if duburi.detected('pipe'):
                break
            duburi.move_forward(0.5, gain=30)
            slalom_steps += 1
        else:
            log(f'pipe {i} not found — advancing blindly')
            duburi.move_forward(2.0, gain=50)
            continue
        side = 'right' if i % 2 == 0 else 'left'
        duburi.vision.align(target='pipe', yaw=0, lat=0, duration=10)
        (duburi.move_right if side == 'right' else duburi.move_left)(1.5, gain=40)
        duburi.move_forward(1.0, gain=50)
    log('SLALOM DONE')

    # Task 6: Return gate
    while not duburi.detected('gate'):
        duburi.move_forward(0.5, gain=30)
    duburi.vision.align(target='gate', yaw=0, lat=0, duration=20)
    duburi.vision.move(target='gate', fwd=40, mode='area', duration=20)
    duburi.move_forward_dist(distance_m=3.0, gain=60)
    log('RETURN GATE DONE')

    duburi.set_depth(0.0)
    duburi.disarm()
```

---

## Pool Day Schedule

| Date | Goal | Pass/Fail Criteria |
|------|------|-------------------|
| ~May 17 | Camera 30 Hz + gate align | `pub=30Hz`, bbox tracks gate, no drift |
| ~May 24 | DVL forward-dist gate pass | AUV drives 3m through gate reliably |
| ~Jun 3  | Full gate_flare_autonomous end-to-end | Mission completes without operator intervention |
| ~Jun 14 | Path markers + orange HSV tracker | AUV follows 2 markers in sequence |
| ~Jun 21 | Slalom (classical HSV) | 2/3 pipes navigated correctly |
| ~Jul 3  | Bins + fire(3) dropper | Marker drops into correct bin |
| ~Jul 9  | Full FSM dry run | Tasks 0, 1, 2, 6 chain — no operator assist |
| Jul 11  | **RoboSub 2026 competition** | — |

---

## Phase 2 — TDR full-scope build (COMMITTED, not yet implemented)

> Reconciliation decision 2026-05-31 (tech lead, audit P0.1): the full TDR is the
> committed 2026 target. The Phase-1 schedule above is what's BUILT/TESTED today
> (single-vehicle Duburi, scripted `detected()`, YOLO11). The items below are
> COMMITTED build tickets with zero-or-partial code — sequence after the Phase-1
> core is pool-proven. `detected()` missions are **kept** as the prototyping /
> per-subsystem unit-test / **FSM-fallback** layer; the FSM wraps the same DSL verbs.

| Ticket | Scope | Depends on | Ref |
|--------|-------|-----------|-----|
| **YASMIN FSM** | states (navigation/perception/manipulation/recovery) wrapping `detected()`/DSL verbs in `duburi_planner/state_machines/`; hosts IVC release-signal transition + bounded-window fallback | Phase-1 verbs (done) | `mission-design.md` |
| **Dubomini 2.0 control path** | profile / `vectored_6dof` frame / 8-thruster map / param-set / mode; VN-200 yaw source | — | `vehicle-spec.md` |
| **IVC** | acoustic-modem transport node + release-signal FSM transition | YASMIN FSM, Dubomini | — |
| **Tasks** Slalom / Bins / Torpedo / Octagon / path-markers | per-task missions + detection models | per-task hardware | this doc Phase-2 sketches |
| **Payload actuation** | ESP32-serial dropper/torpedo client (`duburi.fire(n)`: 1=torpedo port, 2=torpedo starboard, 3=dropper_1, 4=dropper_2) — NOT Pixhawk AUX | ESP32 serial contract | `project_payload_actuation` memory |
| **Stepper grabber** | Actuation-Board step/dir interface + `grab()` verb | grabber wiring | audit G7 |
| **Underwater preprocessing** | colour-cast/haze correction stage ahead of the detector | — | audit G5 |

## Non-Goals (deliberate deferrals)

| Item | Reason |
|------|--------|
| Style maneuvers (gate Task 1 roll/pitch) | ArduSub roll/pitch in MANUAL is risky pool-side; points not worth the risk |
| Multi-camera display | Single `vision_display` window; no additional value for competition |
| CompressedImage transport | Raw frames needed for detector artifact debugging |
| ROS2 + Unreal Engine 5 sim (TDR §III.B) | Gazebo + ArduSub SITL is the working sim; UE5 not pursued in-repo for 2026 |

---

## Key Dependencies

| Dependency | Owner | Needed by |
|------------|-------|-----------|
| DVL Nucleus1000 connected + calibrated | Hardware team | May 24 pool day |
| Downward camera mounted + cabled | Hardware team | Jun 14 path marker test |
| Dropper solenoid wired to AUX | Hardware team | Jun 24 bins test |
| Torpedo actuator wired + verified | Hardware team | Jul 1 (if pursuing Task 4) |
| Pool access (every ~5 days) | Pool team | All dates above |
| Bin symbol dataset + training | Vision team | Jun 24 bins test |
