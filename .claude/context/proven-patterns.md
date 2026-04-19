# Proven Code Patterns — Duburi AUV Reference Codebases

> **REFERENCE PATTERNS — extracted from BracU_Duburi_Control 2023/2025
> codebases.** Do **NOT** assume the package or node names below
> (`duburi_driver`, `duburi_bringup`, `duburi_teleop`, ...) exist in
> *this* workspace. They don't. This file is for the *shape* of the
> patterns (ARM sequence, RC override, AHRS reading, mode switching,
> servo PWM, etc.) — copy the pattern, not the imports.
>
> For the **live** architecture, package layout, and ROS2 surface in
> *this* workspace see:
> * [README §3 Architecture](../../README.md#3-architecture)
> * [`vehicle-spec.md`](./vehicle-spec.md) — canonical hardware
> * [`ros2-conventions.md`](./ros2-conventions.md) — actual `/duburi/move` action + params
>
> The live MAVLink wrapper that *uses* these patterns lives at
> `src/duburi_control/duburi_control/pixhawk.py` (class `Pixhawk`).

All patterns below are battle-tested from our competition-grade codebases.
Source priority: 2023 (2nd world) > 2025 (8th world) > ardusub-interface > others.

---

## CONNECTION SETUP (canonical)

```python
import os
os.environ['MAVLINK20'] = '1'          # MUST be before mavutil import
from pymavlink import mavutil
import time, math
from pymavlink.quaternion import QuaternionBase

CONNECTION_STRING = "udpin:0.0.0.0:14550"   # SIM and POOL modes

master = mavutil.mavlink_connection(CONNECTION_STRING)
master.wait_heartbeat()
boot_time = time.time()

print(f"Connected — System: {master.target_system}, "
      f"Component: {master.target_component}")
```

**Why `udpin`?** BlueOS/SITL is the client that sends. Jetson listens (server mode). 
**Why `MAVLINK20`?** Enables 18 RC channels instead of 8. Set BEFORE import.

---

## RC CHANNEL OVERRIDE (primary thruster control)

```python
# Channel mapping (1-indexed MAVLink ↔ 0-indexed array)
# Ch1=Pitch  Ch2=Roll  Ch3=Throttle  Ch4=Yaw  Ch5=Forward  Ch6=Lateral
# PWM: 1100–1900 µs | 1500=neutral | 65535=no-change (don't override)

def send_rc_override(pitch=1500, roll=1500, throttle=1500,
                     yaw=1500, forward=1500, lateral=1500):
    vals = [65535] * 18          # 18-channel MAVLink 2.0
    vals[0] = int(pitch)
    vals[1] = int(roll)
    vals[2] = int(throttle)
    vals[3] = int(yaw)
    vals[4] = int(forward)
    vals[5] = int(lateral)
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *vals)

def pct_to_pwm(pct: float) -> int:
    """Convert -100..+100 % to 1100..1900 PWM. Clamps at limits."""
    pwm = int(1500 + (pct / 100.0) * 400)
    return max(1100, min(1900, pwm))

def stop_thrusters():
    send_rc_override(1500, 1500, 1500, 1500, 1500, 1500)
```

**Direction conventions (from 2025 codebase control.py)**:
```
status="forward"   → ch5 = 1500 + speed
status="back"      → ch5 = 1500 - speed
status="right"     → ch6 = 1500 + speed
status="left"      → ch6 = 1500 - speed
status="up"        → ch3 = 1500 - speed   (up = less throttle in water)
status="down"      → ch3 = 1500 + speed
status="yaw_right" → ch4 = 1500 + speed
status="yaw_left"  → ch4 = 1500 - speed
```

---

## ARMING SEQUENCE (canonical — from BracU_Duburi_Control utils.py)

```python
def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)    # param1=1 → ARM
    master.motors_armed_wait()
    print("ARMED")

def disarm():
    _set_mode("MANUAL")             # Must be MANUAL before disarm
    time.sleep(3)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)    # param1=0 → DISARM
    master.motors_disarmed_wait()
    print("DISARMED")

# Cold-start pattern: always disarm first, then arm fresh
def cold_start_arm():
    disarm()
    time.sleep(1)
    arm()
```

---

## MODE SWITCHING

```python
def _set_mode(mode_name: str) -> bool:
    if mode_name not in master.mode_mapping():
        print(f"Unknown mode: {mode_name}. Available: {list(master.mode_mapping())}")
        return False
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    # Confirm with heartbeat
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
    return ack is not None and ack.result == 0

# Usage:
_set_mode("MANUAL")
_set_mode("ALT_HOLD")    # = DEPTH_HOLD in ArduSub
_set_mode("STABILIZE")
_set_mode("GUIDED")
```

---

## ATTITUDE TARGET (yaw/heading control — works in ALT_HOLD + STABILIZE)

```python
from pymavlink.quaternion import QuaternionBase
import math

def set_attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        QuaternionBase([math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg)]),
        0, 0, 0, 0)           # roll_rate, pitch_rate, yaw_rate, thrust
```

**Usage**: In ALT_HOLD mode, ArduSub holds depth; call `set_attitude` repeatedly to maintain heading. Combined with RC override for forward/lateral = full AUV control.

---

## DEPTH SETPOINT (SET_POSITION_TARGET — ALT_HOLD required)

```python
def set_depth(depth_m: float):
    """depth_m is NEGATIVE for below-surface. e.g. -0.5 = 50cm deep."""
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        0, 0, depth_m,         # lat_int, lon_int, alt (Z)
        0, 0, 0,               # vx, vy, vz
        0, 0, 0, 0, 0)         # afx, afy, afz, yaw, yaw_rate

# Binary bitmask shorthand (same result, from 2023 codebase):
# type_mask = 0b0000111111111000
```

---

## TELEMETRY READING

```python
# Attitude + Depth (primary source — AHRS2)
def get_attitude_depth():
    msg = master.recv_match(type='AHRS2', blocking=True, timeout=1.0)
    if not msg:
        return None
    yaw = math.degrees(msg.yaw)
    if yaw < 0:
        yaw += 360.0
    return {
        'yaw': yaw,
        'roll': math.degrees(msg.roll),
        'pitch': math.degrees(msg.pitch),
        'depth': msg.altitude     # Negative when underwater (meters)
    }

# Battery (from 2025 codebase sharp_value() pattern)
def get_battery():
    msg = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1.0)
    if not msg:
        return None
    return {
        'voltage': msg.voltages[0] / 1000.0,    # mV → V
        'current': msg.current_battery / 100.0, # cA → A
        'mah_consumed': msg.current_consumed
    }

# Non-blocking latest message (for high-frequency loops)
def peek_attitude():
    if 'AHRS2' in master.messages:
        msg = master.messages['AHRS2']
        yaw = math.degrees(msg.yaw)
        if yaw < 0:
            yaw += 360
        return yaw, msg.altitude
    return None, None
```

---

## HEARTBEAT (MANDATORY — run in separate thread at ≥ 1 Hz)

```python
import threading

def _heartbeat_loop():
    while _running:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.5)       # 2 Hz — safe margin above 1 Hz minimum

_running = True
hb_thread = threading.Thread(target=_heartbeat_loop, daemon=True)
hb_thread.start()
```

**Critical**: Without heartbeat, ArduSub triggers RC failsafe and disarms after ~3 seconds.

---

## SERVO CONTROL (Gripper / Torpedo launcher)

```python
def set_servo_pwm(servo_n: int, microseconds: int):
    """servo_n: 1-3 for AUX1-AUX3. Note: Pixhawk offset = servo_n + 8."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        servo_n + 8,        # AUX servo index offset
        microseconds,
        0, 0, 0, 0, 0)

# Examples:
set_servo_pwm(1, 1500)   # AUX1 neutral
set_servo_pwm(1, 1900)   # AUX1 full (gripper open)
set_servo_pwm(1, 1100)   # AUX1 min (gripper close)
```

---

## MISSION TIMING PATTERN (from team archive `pre_auv` — proven at competition)

```python
def timed_move(direction: str, duration: float, speed: int = 80):
    """Move in direction for duration seconds then stop."""
    start = time.time()
    while time.time() < start + duration:
        if direction == "forward":
            send_rc_override(forward=pct_to_pwm(speed))
        elif direction == "lateral_right":
            send_rc_override(lateral=pct_to_pwm(speed))
        # ... etc
        # Maintain heading during movement:
        set_attitude(0, 0, current_heading)
        time.sleep(0.05)   # 20 Hz
    stop_thrusters()
    time.sleep(0.5)

# Heading hold during movement (from demo.py — most polished):
def forward_with_heading(duration: float, heading: float, speed: int = 80):
    start = time.time()
    while time.time() < start + duration:
        set_attitude(0, 0, heading)           # Hold heading
        send_rc_override(forward=pct_to_pwm(speed))
        time.sleep(0.05)
    stop_thrusters()
```

---

## HEADING WRAP-AROUND (use everywhere heading math happens)

```python
def heading_error(target: float, current: float) -> float:
    """Shortest-path signed error on 0-360 circle. Result: -180 to +180."""
    return (target - current + 540) % 360 - 180

def normalize_heading(h: float) -> float:
    """Normalize any angle to 0-360."""
    return h % 360
```

---

## ROS2 NODE PATTERN (driver node skeleton — from 2025 architecture)

```python
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
import threading, time, math
from pymavlink import mavutil

class DriverNode(Node):
    def __init__(self):
        super().__init__('duburi_driver')
        
        # Declare params
        self.declare_parameter('connection_string', 'udpin:0.0.0.0:14550')
        conn = self.get_parameter('connection_string').value
        
        # Connect
        self.master = mavutil.mavlink_connection(conn)
        self.master.wait_heartbeat()
        self.boot_time = time.time()
        self.get_logger().info("MAVLink connected")
        
        # Publishers
        self.attitude_pub = self.create_publisher(Attitude, '/duburi/attitude', 10)
        self.state_pub = self.create_publisher(VehicleState, '/duburi/state', 10)
        
        # Subscribers
        self.create_subscription(RCOverride, '/duburi/rc_override',
                                 self._rc_override_cb, 10)
        
        # Timers
        self.create_timer(0.5, self._heartbeat_cb)    # 2 Hz HB
        self.create_timer(0.1, self._telemetry_cb)   # 10 Hz telemetry

    def _heartbeat_cb(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def _telemetry_cb(self):
        msg = self.master.recv_match(type='AHRS2', blocking=False)
        if msg:
            # Publish to /duburi/attitude
            pass

def main():
    rclpy.init()
    node = DriverNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

---

## YASMIN STATE MACHINE PATTERN (from 2025 robosub_smach.py)

```python
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

class DepthSetState(State):
    def __init__(self, node, depth_m: float):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self._node = node
        self._depth = depth_m

    def execute(self, blackboard: Blackboard) -> str:
        self._node.set_depth(self._depth)
        time.sleep(2.0)   # Wait for depth to settle
        actual = self._node.get_depth()
        if abs(actual - self._depth) < 0.15:
            return SUCCEED
        return ABORT

# Build mission:
sm = StateMachine(outcomes=[SUCCEED, ABORT])
sm.add_state("ARM", ArmState(node), transitions={SUCCEED: "SET_DEPTH"})
sm.add_state("SET_DEPTH", DepthSetState(node, -0.6),
             transitions={SUCCEED: "GATE", ABORT: "SET_DEPTH"})  # retry
```

---

## DVL INTEGRATION PATTERNS (from 2025 dvl_listerner.py)

```python
# Heading hold using DVL (proportional only — sufficient for competition)
def dvl_heading_correction(target: float, current: float) -> int:
    err = heading_error(target, current)
    speed = min(300, int(abs(err) / 180 * 300))
    if speed < 5:
        return 0              # Deadband — ignore tiny errors
    speed = max(36, speed)    # Minimum effective speed
    if speed < 70:
        speed = int(speed * 1.5)   # Boost low speeds (thruster dead zone)
    return int(math.copysign(speed, err))

# Depth hold using DVL (proportional + soft boost)
def dvl_depth_correction(target: float, current: float) -> int:
    err = target - current
    if abs(err) < 0.1:
        return 0              # ±10cm deadband
    speed = int(max(-300, min(300, err * 300)))
    if 15 < abs(speed) < 70:
        speed = int(math.copysign(speed * 1.5, speed))
    return -speed             # Sign inversion (DVL convention)

# DVL depth offset calibration (measured at Robosub 2025)
DVL_DEPTH_MATCH = 0.78        # Subtract from DVL reading to get true depth
```
