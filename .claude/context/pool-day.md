# Pool Day Reference

> Single-file guide for the day of testing. Everything here assumes
> the AUV is rigged, tethered, and sitting poolside.
>
> **Quick links:** [Bringup](#1-bringup-sequence) · [Sanity checks](#2-sanity-checks) ·
> [First run](#3-first-run-gate-flare-prequal) · [Live tuning](#4-live-tuning) ·
> [Failure recovery](#5-failure-recovery) · [End of day](#6-end-of-day)

---

## 0. Prerequisites (before you get to the pool)

```bash
# Sim test passed (gate_flare_prequal runs end-to-end)
ros2 run duburi_planner mission gate_flare_prequal  # in Gazebo

# Weights are on the vehicle (copy to Jetson if needed)
ls ~/Ros_workspaces/duburi_ws/src/duburi_vision/models/
# Must see: gate_flare_medium_100ep.pt

# Build is clean
cd ~/Ros_workspaces/duburi_ws
./build_duburi.sh && source install/setup.bash
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

**Phase summary:**
- T−10 s: countdown (disconnect tether in this window)
- Phase 1: arm → ALT_HOLD → dive to −1.0 m
- Phase 2: DVL connect
- Phase 3−6: find gate → align (yaw+forward, area) → DVL forward 3.5 m
- Phase 7−10: switch to flare → find flare → 3-axis align → orbit (12×30°)
- Phase 11−14: yaw 180° → reacquire gate → DVL forward 3.5 m
- Phase 15: surface → disarm

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
Phase 5: aligning with gate
  vision_align_3d        final=0.04  err=0.11   (settled yaw+forward)
Phase 6: passing through gate (DVL forward)
  move_forward_dist      final=-1.01 err=0.00   (3.5 m)
...
Mission complete.
```

---

## 4. Live tuning (between runs, no rebuild)

All of these take effect on the NEXT vision command. No node restart.

### Vision gains

```bash
ros2 param set /duburi_manager vision.kp_yaw      70.0   # default 60; raise if slow steering
ros2 param set /duburi_manager vision.kp_forward  180.0  # default 200; lower if oscillating distance
ros2 param set /duburi_manager vision.deadband      0.08  # default 0.18; lower for tighter lock
ros2 param set /duburi_manager vision.stale_after   2.0   # default 1.5; raise in murky water
```

### Gate standoff distance

```bash
# In gate_flare_prequal: GATE_STANDOFF constant controls it per-call
# For live single-call testing:
ros2 param set /duburi_manager vision.target_bbox_h_frac 0.45
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

### Gate not found (timeout in Phase 4)

**Symptom:** `vision_acquire: timeout after 45.0 s`

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
# Raise deadband (accept rougher centering)
ros2 param set /duburi_manager vision.deadband 0.15

# Lower gains
ros2 param set /duburi_manager vision.kp_yaw 40.0
ros2 param set /duburi_manager vision.kp_forward 120.0

# Switch on_lost to hold (less sensitive to flickers)
ros2 param set /duburi_manager vision.on_lost hold
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

### New preferred vision names

```python
duburi.vision.scan(target, sweep='right', timeout=25)   # search
duburi.vision.steer(target, duration=8)                 # Ch4: yaw to centre
duburi.vision.strafe(target, duration=8)                # Ch6: slide to centre
duburi.vision.level(target, duration=8)                 # depth to centre
duburi.vision.approach(target, distance=0.55, duration=12)  # Ch5: close in

# Multi-axis (preferred — no CSV string)
duburi.vision.align(
    yaw=True, forward=True,          # gate: 2-axis
    # yaw=True, forward=True, depth=True,   # flare: 3-axis
    distance=0.42,
    distance_metric='area',          # 'area' for gates, 'height' for flare/poles
    on_lost='hold',
    duration=20.0)

# Continuous track (orbit re-lock)
duburi.vision.align(yaw=True, forward=True, depth=True,
                    distance=0.38, duration=3.0,
                    on_lost='hold', lock_mode='follow')
```

### Class filter switching

```python
duburi.set_classes('gate')      # Phase before gate
duburi.set_classes('flare')     # Phase after passing gate
duburi.set_classes('')          # All classes (debug only)
```

### Useful one-liners from deck

```bash
# Watch state live
watch -n 0.5 "ros2 topic echo /duburi/state --once 2>/dev/null"

# See what detector is currently classifying
ros2 topic echo /duburi/vision/forward/detections --once | grep class

# Manual depth change
ros2 run duburi_planner duburi set_depth --target -0.8

# Manual heading lock
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120

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
- DVL integration: [`dvl-integration.md`](./dvl-integration.md)
- Known bugs: [`known-issues.md`](./known-issues.md)
