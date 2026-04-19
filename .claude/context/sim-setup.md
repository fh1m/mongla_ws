# Simulation Setup Guide — Duburi AUV

Complete step-by-step guide to bring up the SITL + Gazebo simulation stack.
All commands run inside the auv-ros2 distrobox docker.

> **Sim proxy callout:** Gazebo runs the `bluerov2_heavy_underwater.world` model because it shares the BRACU Duburi 4.2's `vectored_6dof` 8-thruster ArduSub frame — so control behavior maps faithfully. The hull shape, exact mass, and payload geometry differ from the real sub. See [`vehicle-spec.md`](./vehicle-spec.md) §"Real vehicle vs sim".

---

## Architecture Overview

```
[ArduSub SITL] ←JSON/UDP→ [ardupilot_gazebo plugin] ←→ [Gazebo Harmonic]
      ↕ UDP:14550
[Our ROS2 node — auv_manager_node — owns the single MAVLink connection]
      ↕
[ /duburi/move action  +  /duburi/state String topic ]
      ↕
[duburi CLI  /  test_runner  /  any external ActionClient]
```

The ardupilot_gazebo plugin bridges physics: Gazebo simulates buoyancy, drag, and
thruster forces; the plugin converts these to JSON and feeds them to ArduSub SITL.
ArduSub runs its own EKF3 and PID loops, sending back motor commands to Gazebo.

---

## Step 1: Environment Check

```bash
# Verify paths are set (should be in ~/.zshrc already)
echo $GZ_SIM_RESOURCE_PATH
echo $GZ_SIM_SYSTEM_PLUGIN_PATH
# Should include ardupilot_gazebo/build and bluerov2_gz paths

# Verify ArduSub SITL is available
which sim_vehicle.py
# → ~/stuff/ardupilot/Tools/autotest/sim_vehicle.py

# Verify Gazebo plugin built
ls ~/stuff/ardupilot_gazebo/build/*.so
# → Should show libArduPilotPlugin.so or similar
```

---

## Step 2: Launch ArduSub SITL

```bash
# Terminal 1 (or tmux pane)
sim_vehicle.py \
  -L RATBeach \
  -v ArduSub \
  -f vectored_6dof \
  --model=JSON \
  --out=udp:0.0.0.0:14550 \
  --out=udp:127.0.0.1:14551 \
  --console

# Flags explained:
# -L RATBeach       : Start location (GPS coordinates for RATBeach test pool)
# -v ArduSub        : Vehicle type
# -f vectored_6dof  : BlueROV2 Heavy frame (8 thrusters, full 6DOF)
# --model=JSON      : Use JSON interface for Gazebo plugin
# --out UDP:14550   : Output to all interfaces:14550 (our ROS nodes connect here)
# --out UDP:14551   : Extra output for monitoring/MAVROS if needed
# --console         : Show MAVProxy text console

# Wait for: "APM: EKF3 IMU0 is using barometer" before proceeding
```

**Troubleshooting SITL:**
- If it hangs: kill and retry — sometimes SITL needs a fresh start
- If GPS warnings: normal for AUV (we don't use GPS)
- If EKF errors: wait 30 seconds, EKF3 needs time to initialize from cold start
- Port already in use: `pkill -f sim_vehicle` and retry

---

## Step 3: Launch Gazebo

```bash
# Terminal 2 (or tmux pane)
cd ~/Ros_workspaces/colcon_ws
gz sim -v 3 -r src/bluerov2_gz/worlds/bluerov2_underwater.world

# Flags:
# -v 3   : Verbosity level 3 (shows warnings, useful for debugging)
# -r     : Run immediately (don't pause at start)

# Available world files:
# bluerov2_underwater.world       → Standard BlueROV2 (vectored, 6 thrusters)
# bluerov2_heavy_underwater.world → Heavy config (vectored_6dof, 8 thrusters) ← USE THIS
# bluerov2_ping.world             → With sonar
```

**Wait for**: Gazebo window to open and vehicle to be visible underwater.
**Expected**: Vehicle should be positionally stable (slight float/sink is normal until SITL connects).

---

## Step 4: Verify SITL-Gazebo Connection

In the SITL console (MAVProxy), you should see:
```
APM: JSON interface: Packet received
APM: Barometer calibrated
APM: EKF3 IMU0 is using barometer
```

If Gazebo and SITL are NOT communicating:
- Check `GZ_SIM_SYSTEM_PLUGIN_PATH` includes ardupilot_gazebo/build
- Check `GZ_SIM_RESOURCE_PATH` includes bluerov2_gz paths
- Restart both Gazebo and SITL

---

## Step 5: Build and Run Our Nodes

```bash
# Terminal 3
cd ~/Ros_workspaces/duburi_ws
./build_duburi.sh
source install/setup.bash

# Run the manager node (connects to SITL on udpin:0.0.0.0:14550)
ros2 run duburi_manager auv_manager_node --ros-args -p mode:=sim
```

> No `duburi_bringup` package exists. We don't use launch files yet — every entry point goes through `ros2 run ... --ros-args -p mode:=...`. See [`ros2-conventions.md`](./ros2-conventions.md) §10.

---

## Step 6: Verify Connectivity

```bash
# Watch the JSON telemetry snapshot (1 line per second-ish)
ros2 topic echo /duburi/state
# Expected: {"armed": false, "mode": "MANUAL", "yaw_deg": ..., "depth_m": ..., "battery_v": ...}

# Confirm the action server is up
ros2 action list | grep duburi
# Expected: /duburi/move

ros2 action info /duburi/move
# Expected: 1 action server, type duburi_interfaces/action/Move
```

---

## Step 7: Test Basic Commands

```bash
# All commands go through the duburi CLI → /duburi/move action.
ros2 run duburi_manager duburi arm           # arms via MAV_CMD_COMPONENT_ARM_DISARM
ros2 run duburi_manager duburi set_depth -0.5  # auto-engages ALT_HOLD, then setpoint stream
ros2 run duburi_manager duburi yaw_right 90    # SET_ATTITUDE_TARGET via ALT_HOLD
ros2 run duburi_manager duburi move_forward 5 80  # RC_CHANNELS_OVERRIDE Ch5 for 5 s @ 80%
ros2 run duburi_manager duburi disarm
```

> Older notes referenced `/duburi/arm` / `/duburi/depth_cmd` as ROS services / topics — they don't exist. Single action surface only.

---

## Sim-Specific Gotchas

### 1. Depth starts at 0 (surface)
SITL starts with vehicle at surface. Depth commands need ALT_HOLD mode first.
After arming in MANUAL, switch to ALT_HOLD, then send depth setpoint.

### 2. Vehicle may drift horizontally
SITL doesn't have GPS lock → EKF3 may drift in XY. For depth-only missions,
this is fine. For position hold, need to bridge Gazebo ground truth as fake GPS.

### 3. Thrusters in Gazebo respond instantly
Real thrusters have motor ramp-up time. Sim PID gains may need to be more
conservative on hardware (reduce Kp by ~30% as starting point).

### 4. AHRS2 altitude in SITL
In SITL, `AHRS2.altitude` reflects simulated depth from Gazebo physics.
It should match what you see in the Gazebo viewport.

### 5. RC override in MANUAL mode
In MANUAL mode, RC override directly controls thrusters.
In ALT_HOLD, ch3 (throttle) is relative: 1500=hold, >1500=rise, <1500=dive.

---

## Quick Restart Procedure

When something goes wrong and you need a fresh start:

```bash
# Kill everything
pkill -f sim_vehicle
pkill -f gz
pkill -f duburi

# Wait 3 seconds, then restart in order:
# 1. SITL (wait for "EKF3 ready")
# 2. Gazebo (wait for window)
# 3. Our nodes
```

---

## Tmux Session Layout (Recommended)

```
Window 0: SITL
Window 1: Gazebo
Window 2: ROS2 duburi nodes
Window 3: ros2 topic echo / debugging
Window 4: Mission execution
```

Or use the `gazebo_up` alias already in ~/.zshrc for Gazebo.

---

## MAVROS vs Direct Pymavlink in SIM

Two approaches available:

| | Direct Pymavlink | MAVROS |
|---|---|---|
| **Connection** | `udpin:0.0.0.0:14550` | `fcu_url: udp://:14551@localhost:14550` |
| **Used in** | `auv_manager_node` (the only MAVLink consumer in the live path) | ardusub-interface reference |
| **Pros** | Proven, simple, direct | ROS2 native, services/actions |
| **Cons** | More boilerplate | MAVROS overhead, extra node |
| **Our choice** | ✅ Used today | Read-only telemetry consumer planned for Phase 3 (separate endpoint) |

If you bring up MAVROS alongside the manager, point it at port `14551` (the second `--out` we added to SITL). Do not have it bind `14550` — it will steal packets from the manager.
