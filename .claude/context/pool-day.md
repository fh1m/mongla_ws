# Pool Day Reference

> Single-file guide for the day of testing. Everything here assumes
> the AUV is rigged, tethered, and sitting poolside.
>
> **Quick links:** [Bringup](#1-bringup-sequence) · [Sanity checks](#2-sanity-checks) ·
> [First run](#3-first-run-gate-flare-prequal) · [Live tuning](#4-live-tuning) ·
> [Failure recovery](#5-failure-recovery) · [End of day](#6-end-of-day)

---

## 0. Prerequisites (before you get to the pool)

> **Drive the Jetson the drop-proof way** — run launches/missions in **tmux over mosh**
> (`mosh dubomini@192.168.2.69`) so a dropped tether never kills a run; watch vision in
> Lichtblick/`web_video_server`; use NoMachine (not xrdp) for a desktop. One-time setup +
> emergency recovery: [`remote-access.md`](remote-access.md).

```bash
# Sim test passed (gate_flare_prequal runs end-to-end)
ros2 run duburi_planner mission gate_flare_prequal  # in Gazebo

# Weights live in ~/models (source-of-truth, off git); the build syncs them in.
ls ~/models/                                        # place/update weights here
# Must see the competition stems: gate_rescue_repair, slalom_red_pipe,
#   bin_fire_blood, torpedo_blood_hole, octagon  (.pt + .engine each)

# Build is clean (mirrors ~/models + ~/missions into the tree first, then builds)
cd ~/Ros_workspaces/duburi_ws
./build_dubomini.sh && source install/setup.bash
ls src/duburi_vision/models/                        # confirm the sync landed them

# Jetson FPS prep (ON THE JETSON — raw PyTorch is ~3-4 Hz, TensorRT ~10-30 Hz):
sudo nvpmodel -m 0 && sudo jetson_clocks          # MAXN power (~2× alone)
ros2 run duburi_vision export_engine --all        # build <stem>.engine beside each .pt
# Engines are device + JetPack-version locked: build on the Jetson, rebuild after
# a JetPack/TRT upgrade. Confirm at launch: [YOLO ] backend=TensorRT engine.
```

---

## 1. Bringup sequence

**Run in this order. Do not skip steps.**

### Step 1 — hardware pre-arm check

```bash
ros2 run duburi_manager bringup_check
# Exit 0 = all green. Exit 1 = read the output and fix what it says.
# Probes: Pi ping, Jetson ping, UDP 14550 MAVLink stream, BNO085 USB, Pixhawk USB
```

### Step 2 — full stack with vision

```bash
ros2 launch duburi_manager bringup.launch.py vision:=true
```

This starts:
- `auv_manager_node` (MAVLink + action server)
- `camera_node` (forward camera)
- `detector_node` (`gate_flare_medium_100ep`, conf=0.45, classes=gate)

One terminal is enough. All nodes log to stdout.

### Step 3 — verify telemetry

```bash
# In a second terminal
source ~/Ros_workspaces/duburi_ws/install/setup.bash

ros2 topic echo /duburi/state --once
# Expected: armed=false, mode=MANUAL, yaw_deg≈something, depth_m≈0.0, battery_voltage>10.0
```

### Step 4 — verify vision

```bash
# Camera + detector up?
ros2 run duburi_vision vision_check --camera forward --require-class gate
# Exit 0 = gate detections flowing. Takes a few seconds with model in front.

# Detection → RC chain (does seeing a gate actually produce Ch4 output?)
ros2 run duburi_vision vision_thrust_check --camera forward --duration 4
# You should see [RC] Yaw rows proportional to gate position. Sub stays safe-disarmed.
```

### Step 5 — arm test (out of water)

```bash
ros2 run duburi_planner duburi arm
# Confirm thrusters spin briefly (they will if spin_on_arm is default)
ros2 run duburi_planner duburi disarm
```

---

## 2. Sanity checks (run before every mission)

```bash
# Telemetry alive?
ros2 topic hz /duburi/state     # should be ~1 Hz even when idle

# DVL connected?
ros2 topic echo /duburi/state --once | grep -i dvl   # dvl_source: nucleus_dvl

# Camera flowing?
ros2 topic hz /duburi/vision/forward/image_raw        # should be ~20-30 Hz

# Detections flowing?
ros2 topic hz /duburi/vision/forward/detections       # should be ~15-25 Hz

# Class filter correct for current phase?
ros2 param get /duburi_detector classes              # should print 'gate' at start
```

---

## 3. First run — gate_flare_prequal

```bash
ros2 run duburi_planner mission gate_flare_prequal
```

**Phase summary** (`gate_flare_prequal`):
- Phase 0: countdown (disconnect tether in this window)
- Phase 1: arm → ALT_HOLD → dive to −1.0 m → DVL connect
- Phase 2: home gate — `vision.align(yaw, lat)` then `vision.move(area)` to standoff
- Phase 3: pass gate — DVL forward 3.5 m
- Phase 4: home flare — `vision.align(yaw, depth)` then `vision.move(height)`
- Phase 5: orbit flare — 12 × (yaw_left 30° + re-align)
- Phase 6: return — yaw 180° → re-align gate → DVL forward 3.5 m
- Phase 7: surface → disarm

**Normal output to watch:**

```
Phase 0: tether removal countdown
  T- 10s  [████████████████████████████████████████]
  DUBURI IS NOW AUTONOMOUS. GOOD LUCK.
Phase 1: arm + ALT_HOLD + descend
  arm                    final=-0.00 err=0.00   (armed)
  set_mode               final=-0.00 err=0.00   (ALT_HOLD)
  set_depth              final=-1.02 err=0.02   (converged)
...
Phase 2: homing on gate
  vision_align           final=0     err=22     (ALIGNED, yaw+lat within 22px)
  vision_move            final=0     err=42     (ALIGNED, gate fills 42% of frame)
Phase 3: passing through gate (DVL forward)
  move_forward_dist      final=-1.01 err=0.00   (3.5 m)
...
Mission complete.
```

---

## 4. Live tuning (between runs, no rebuild)

All of these take effect on the NEXT vision command. No node restart.

### Vision gains

```bash
ros2 param set /duburi_manager vision.kp_yaw       70.0  # default 60; raise if slow steering
ros2 param set /duburi_manager vision.kp_lat       65.0  # default 60; lateral centring gain
ros2 param set /duburi_manager vision.kp_forward   180.0 # default 200; lower if distance oscillates
ros2 param set /duburi_manager vision.lost_grace_s 1.5   # default 1.0; raise in murky water
```

> Lock tightness is the **per-goal** `err_px` (default 40 px), not a manager
> param — pass `--err_px` on a CLI goal or set `err=` in the mission. The
> ticks-within-band before ALIGNED is `vision.align_stable_frames` (default 3).

### Gate standoff distance

```bash
# In gate_flare_prequal: GATE_FWD_FILL constant controls it — the % of frame the
# gate bbox must fill before vision.move stops (closer = higher fill).
# For live single-call testing, pass --fwd_fill on the goal, or change the
# default fill the manager uses when a goal leaves fwd_fill unset:
ros2 param set /duburi_manager vision.frame_fill_default 45.0
```

### Class filter (without restarting detector)

```bash
ros2 param set /duburi_detector classes gate
ros2 param set /duburi_detector classes flare
ros2 param set /duburi_detector classes "gate,flare"    # debug: see all
```

### Confidence threshold

```bash
# Requires detector node restart — set in config then rebringup:
#   src/duburi_vision/config/detector.yaml  conf: 0.45
# Start lower (0.35) if target not seen at distance; raise if false positives
```

### DVL gate pass distance

```bash
# Edit in gate_flare_prequal.py:
GATE_PASS_M = 3.5     # → increase if AUV stops short of gate
RETURN_PASS_M = 3.5   # → same for return leg
# Then: colcon build --packages-select duburi_planner && source install/setup.bash
```

---

## 5. Failure recovery

### Sub arm failed / pre-arm check

```
arm: RC_FAIL
```

**Fix:** Check that Pixhawk's RC override is live. Run `duburi stop` first, then retry:

```bash
ros2 run duburi_planner duburi stop
ros2 run duburi_planner duburi arm
```

If still failing, check the BlueOS pre-arm page at http://192.168.2.1.

### Gate not found (search / align times out)

**Symptom:** `[VIS  ] align 'gate': NOT reached (TIMEOUT) -- mission continues`
(or `... (LOST)` if the `fallback` search never reacquired the target)

**Checks:**
1. Is the class filter set to 'gate'? → `ros2 param get /duburi_detector classes`
2. Is the camera streaming? → `ros2 topic hz /duburi/vision/forward/image_raw`
3. Is the gate actually in the forward camera's FOV at start position?
4. Try `ros2 run duburi_vision vision_check --camera forward --require-class gate` from deck

**Quick fix:** Lower confidence or manually point the AUV toward the gate, then rerun.

### DVL not connecting

**Symptom:** `Phase 2: DVL connect` hangs or logs `[DVL  ] TCP connection failed`

**Checks:**
1. `ping 192.168.2.201` from Jetson — if no response, DVL power/cable issue
2. `ros2 param get /auv_manager nucleus_dvl_host` — should be `192.168.2.201`
3. Try manual: `ros2 run duburi_planner duburi dvl_connect`

**Fallback:** Start manager with `yaw_source:=bno085` — DVL distance moves fall back to
open-loop time estimates. Gate pass will be approximate but mission still runs.

```bash
ros2 launch duburi_manager bringup.launch.py vision:=true yaw_source:=bno085
```

### AUV not moving (zero thrust)

**Symptom:** Commands complete instantly with `final=0.0 err=0.0` but no physical movement.

**Checks:**
1. Is it armed? → `ros2 topic echo /duburi/state --once | grep armed`
2. Is it in ALT_HOLD? → check mode field
3. Is the Heartbeat running? It should be if manager is up.
4. `ros2 run duburi_planner duburi move_forward --duration 2 --gain 80` — does thruster spin?

### Vision lock oscillating / not settling

```bash
# Accept rougher centring: widen the per-goal err_px (default 40 px).
ros2 run duburi_planner duburi vision_align --camera forward --target_class gate \
    --axes yaw,lat --err_px 60 --gain 25 --duration 20

# Lower the loop gains (gentler corrections):
ros2 param set /duburi_manager vision.kp_yaw 40.0
ros2 param set /duburi_manager vision.kp_lat 40.0
ros2 param set /duburi_manager vision.kp_forward 120.0

# Ride brief detector flickers without re-searching: raise lost_grace_s.
ros2 param set /duburi_manager vision.lost_grace_s 1.5
```

### Emergency disarm

```bash
# From any terminal:
ros2 run duburi_planner duburi disarm

# Or kill the manager (Ctrl+C in its terminal) — Duburi.stop() + disarm() run on shutdown
```

---

## 6. End of day

```bash
# Final telemetry check before power-down
ros2 topic echo /duburi/state --once

# Graceful shutdown
# 1. Ctrl+C on the bringup terminal — manager disarms on SIGINT
# 2. Power off Jetson: ssh jetson@192.168.2.69 'sudo poweroff'
# 3. Power off vehicle (main switch)

# Save run logs (if needed)
ros2 bag record -a -o /tmp/pool_$(date +%Y%m%d_%H%M) &
# Stop with Ctrl+C when done
```

---

## 7. DSL quick reference (pool-day cheat sheet)

### The two vision verbs

```python
# Register model at mission start
duburi.models(gate='gate_flare_medium_100ep')
gate, flare = duburi.models.gate.gate, duburi.models.gate.flare

# align: centre the target on the chosen axes. Each of lat / yaw / depth is a
# SIGNED PIXEL OFFSET from centre (0 = centre); omit / None = axis off.
duburi.vision.align(gate,  yaw=0, lat=0,   err=40, gain=30, duration=20)  # gate: yaw+lat
duburi.vision.align(flare, yaw=0, depth=0, err=40, gain=30, duration=20)  # flare: yaw+depth
duburi.vision.align(gate,  lat=80,         err=40, gain=30, duration=20)  # hold 80px right of centre

# move: drive forward until the bbox fills fwd% of the frame
# (mode = area | width | height). gain is a HARD max-speed cap, not a target.
duburi.vision.move(gate,  fwd=42, mode='area',   gain=45, duration=20)    # gate: area metric
duburi.vision.move(flare, fwd=38, mode='height', gain=45, duration=20)    # tall pipe: height
duburi.vision.move(gate,  fwd=80, mode='area', maintain=0, hold=2, gain=45)  # keep centred + hold 2s

# Recover-don't-fail: pass a mission search fn as fallback; on target loss the
# verb runs it once and re-enters, all inside `duration`.
def creep(duburi):
    duburi.move_forward(0.6, gain=40)
duburi.vision.align(gate, yaw=0, lat=0, duration=20, fallback=creep)

# Both return a VisionResult — truthy ONLY when ALIGNED / fill reached:
if not duburi.vision.align(gate, yaw=0, lat=0):
    log('gate not centred -- mission continues anyway')
```

### Class filter switching

Passing a `ClassRef` (e.g. `duburi.models.gate.flare`) handles model+class switching automatically.
Manual override if needed:

```python
duburi.set_classes('gate')      # filter to gate class (model unchanged)
duburi.set_classes('flare')     # filter to flare class
duburi.set_classes('')          # all classes (debug only)
```

### Useful one-liners from deck

```bash
# Watch state live
watch -n 0.5 "ros2 topic echo /duburi/state --once 2>/dev/null"

# Live camera + detection feed (OpenCV window, no Qt/rqt needed)
ros2 run duburi_vision vision_display --ros-args -p camera:=forward

# See what detector is currently classifying
ros2 topic echo /duburi/vision/forward/detections --once | grep class

# Manual depth change
ros2 run duburi_planner duburi set_depth --target -0.8

# Manual heading lock
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120

# DVL distance moves (with heading lock active)
ros2 run duburi_planner duburi move_forward_dist --distance_m 1.0 --gain 60
ros2 run duburi_planner duburi move_back_dist    --distance_m 1.0 --gain 60

# Vision: centre the gate (yaw+lat), then drive in until it fills 80% (area)
ros2 run duburi_planner duburi vision_align --camera forward --target_class gate \
    --axes yaw,lat --err_px 40 --gain 30 --duration 20
ros2 run duburi_planner duburi vision_move  --camera forward --target_class gate \
    --fwd_fill 80 --mode area --gain 35 --duration 20

# Abort / safe state
ros2 run duburi_planner duburi stop && ros2 run duburi_planner duburi disarm
```

---

## 8. Cross-references

- Full command API: [`command-reference.md`](./command-reference.md)
- DSL methods + client API: [`client-and-dsl-api.md`](./client-and-dsl-api.md)
- Vision verbs deep-dive: [`command-reference.md §9`](./command-reference.md)
- Mission composition patterns: [`mission-cookbook.md`](./mission-cookbook.md)
- Gate+flare mission source: [`src/duburi_planner/duburi_planner/missions/gate_flare_prequal.py`](../../src/duburi_planner/duburi_planner/missions/gate_flare_prequal.py)
- Model README: [`src/duburi_vision/models/README.md`](../../src/duburi_vision/models/README.md)
- DVL reference (Nucleus 1000, POSHOLD, unused capabilities): [`dvl-reference.md`](./dvl-reference.md)
- Known bugs: [`BUGS.md`](./BUGS.md)
