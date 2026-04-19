# PID Control Theory & Implementation Guide — Duburi AUV

> **REFERENCE — kept for tuning intuition; not the live control path.**
>
> The live path delegates depth and yaw to ArduSub's onboard 400 Hz
> stabilizer + EKF3:
>
> | Axis    | What we send                                | Loop that closes it           |
> |---------|---------------------------------------------|-------------------------------|
> | Yaw     | `SET_ATTITUDE_TARGET` @ 10 Hz               | ArduSub attitude PID          |
> | Depth   | `SET_POSITION_TARGET_GLOBAL_INT` @ 5 Hz     | ArduSub ALT_HOLD position PID |
> | Linear  | `RC_CHANNELS_OVERRIDE` Ch5/Ch6 @ 20 Hz      | open loop (timed thrust)      |
>
> Earlier revisions kept Python `DepthPID` / `YawPID` classes in
> `movement_pids.py` as a documented fallback. That file was deleted in
> the 2026-04 cleanup — ArduSub's onboard PID is the only loop in the
> live path now. The Python implementation can be recovered from git
> history if it ever becomes a hot-fix fallback.
>
> The rest of this file is the underlying theory; if you need to know
> **what the live code does**, read `motion_yaw.py`, `motion_depth.py`,
> `motion_forward.py` / `motion_lateral.py` (with shared logic in `motion_common.py`), and the [README's tuning guide](../../README.md#11-tuning-guide)
> first.

First-principles approach to robust AUV control. This is the theoretical foundation
for any future custom control work in `duburi_control`.

---

## 1. Why Control Theory Matters for AUVs

An AUV in water is a dynamically coupled 6-DOF system:
- External forces: drag (velocity²), buoyancy, water currents
- Thruster nonlinearity: dead zone ~±3% PWM, response curve not linear
- Sensor noise: IMU high-frequency noise, DVL latency, depth quantization
- Coupling: yaw corrections cause lateral drift, depth changes affect pitch

A robust control stack must handle all of these **without manual tuning per pool**.

---

## 2. PID from First Principles

### The Plant
The "plant" is ArduSub + thrusters + vehicle dynamics. Our controller closes the loop:

```
Setpoint (r) → [PID Controller] → u(t) → [Thrusters/ArduSub] → y(t) → Measurement
                    ↑___________________________________________|
                                 error e(t) = r - y(t)
```

### The PID Equation

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(de/dt)
```

| Term | Symbol | What it does | Problem if too high |
|---|---|---|---|
| Proportional | `Kp·e` | Corrects current error | Oscillation, overshoot |
| Integral | `Ki·∫e` | Eliminates steady-state error | Integrator windup, slow |
| Derivative | `Kd·de/dt` | Predicts future error, damps | Noise amplification |

### Discrete Implementation (what we actually code)

```python
dt = current_time - last_time

error = setpoint - measurement

proportional = kp * error

integral += error * dt
integral = clamp(integral, -integral_max, integral_max)  # Anti-windup
integrated = ki * integral

# Derivative on MEASUREMENT (not error) → no derivative kick on setpoint change
derivative = -kd * (measurement - last_measurement) / dt

output = proportional + integrated + derivative
output = clamp(output, out_min, out_max)

last_measurement = measurement
last_time = current_time
```

**Why derivative on measurement?** When setpoint changes, `d(error)/dt` spikes.
`-d(measurement)/dt` is smooth — the sign is negative because measurement increase
= error decrease = derivative should push output down.

---

## 3. Anti-Windup (Critical for AUVs)

**Problem**: When output saturates (e.g., thruster at max), integral keeps accumulating.
When error reverses, integrator must unwind before output decreases → sluggish response.

**Solution — Clamp Method**:
```python
# Clamp integral only when output is saturated AND error is in the same direction
# This is the "back-calculation" anti-windup approximation

def compute(self, error: float, measurement: float, dt: float) -> float:
    self._integral += error * dt

    output_unlim = self._kp * error + self._ki * self._integral \
                   - self._kd * (measurement - self._prev_meas) / dt

    # Clamp output
    output = max(self._out_min, min(self._out_max, output_unlim))

    # If saturated and error would increase saturation, stop integrating
    if (output == self._out_max and error > 0) or \
       (output == self._out_min and error < 0):
        self._integral -= error * dt  # Undo this integration step

    self._prev_meas = measurement
    return output
```

---

## 4. Derivative Filtering (Critical for noisy IMU)

Raw derivative = `(error_now - error_prev) / dt` amplifies high-frequency IMU noise.

**Low-Pass Filter on Derivative**:
```python
# Simple first-order filter: α controls cutoff (0 → no filtering, 1 → no derivative)
# Cutoff frequency: fc = (1-α) / (2π·α·dt)

alpha = 0.1   # Typical starting value for AUV

derivative_raw = (measurement - self._prev_meas) / dt
self._derivative_filtered = alpha * self._derivative_filtered + (1-alpha) * derivative_raw
```

For depth and heading control, α = 0.1 (light filtering) is usually sufficient.
For position control with DVL, α = 0.2–0.3 (more filtering).

---

## 5. Deadband (Prevents Thruster Jitter)

Thrusters have a mechanical dead zone (~±3% of full range = ~±24 µs PWM).
Constant tiny corrections wear out thrusters and look ugly in water.

```python
DEADBAND_HEADING_DEG = 2.0    # Don't correct errors < 2 degrees
DEADBAND_DEPTH_M     = 0.05   # Don't correct errors < 5 cm
DEADBAND_POSITION_M  = 0.10   # Don't correct errors < 10 cm

def apply_deadband(error: float, deadband: float) -> float:
    if abs(error) < deadband:
        return 0.0
    return error
```

---

## 6. PWM Output Clamping

Always clamp PID output to safe PWM range. Hard limits prevent thruster burnout.

```python
# Safe operational range (not full 1100-1900 — leaves room for emergency)
PWM_NEUTRAL  = 1500
PWM_MIN_SAFE = 1350   # -60% thrust
PWM_MAX_SAFE = 1650   # +60% thrust

# Convert PID output (delta from neutral) to PWM:
def pid_to_pwm(pid_output: float) -> int:
    pwm = int(PWM_NEUTRAL + pid_output)
    return max(PWM_MIN_SAFE, min(PWM_MAX_SAFE, pwm))
```

---

## 7. Heading Control — Special Considerations

Heading is a circular quantity (0–360°). NEVER compute error as simple subtraction.

```python
def heading_error(target: float, current: float) -> float:
    """Returns signed error in -180..+180 range (shortest path)."""
    err = (target - current + 540) % 360 - 180
    return err

# Example:
# target=10°, current=350° → error = (10-350+540)%360-180 = 200%360-180 = 200-180 = 20°
# (Correct: turn 20° right, not -340°)
```

**Output mapping** (heading PID output → yaw channel PWM):
```python
yaw_pwm = int(1500 + heading_pid_output)
# PID output range: typically ±300 (safe) up to ±500 (aggressive)
yaw_pwm = max(1200, min(1800, yaw_pwm))
```

---

## 8. Tuned Starting Values (From Competition Data)

### Depth Control (PID via RC throttle channel)

```
Setpoint: depth in meters (negative, from AHRS2.altitude)
Measurement: AHRS2.altitude
Output: delta PWM added to ch3 (throttle, base=1500)

Kp = 0.9   (proven 2023)
Ki = 0.01
Kd = 0.1
Deadband: ±5 cm
Output range: ±150 PWM from 1500 (→ 1350–1650)
Update rate: 10 Hz
```

### Heading Control (PID via RC yaw channel)

```
Setpoint: desired heading 0–360°
Measurement: math.degrees(AHRS2.yaw) normalized 0–360
Output: delta PWM added to ch4 (yaw, base=1500)

Kp = 1.5   (average of 2023 values: 1.0 fwd, 2.0 lateral)
Ki = 0.0   (no integral — heading drift is slow)
Kd = 0.1
Deadband: ±2°
Output range: ±400 PWM from 1500
Update rate: 10 Hz
```

### Position Hold (DVL, proportional only)

```
Setpoint: target position in DVL frame (meters)
Measurement: DVL INS position
Output: delta forward/lateral PWM

Kp_forward = 70   (scaled 0-300 range from dvl_listerner)
Kp_lateral = 70
Deadband: ±10 cm
Output range: ±70 PWM
Update rate: 100 Hz (DVL output rate)
```

---

## 9. PID Class Design (JSF-AV Principles)

```python
class PIDController:
    """Single-axis PID controller with anti-windup and derivative filtering."""
    
    def __init__(
        self,
        kp: float, ki: float, kd: float,
        out_min: float, out_max: float,
        integral_max: float = 1000.0,
        derivative_alpha: float = 0.1,
        deadband: float = 0.0
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._out_min = out_min
        self._out_max = out_max
        self._integral_max = integral_max
        self._alpha = derivative_alpha
        self._deadband = deadband
        
        self._integral = 0.0
        self._prev_meas = 0.0
        self._deriv_filtered = 0.0
        self._last_time = None
    
    def reset(self):
        """Call on mode change, re-arm, or setpoint jump."""
        self._integral = 0.0
        self._prev_meas = 0.0
        self._deriv_filtered = 0.0
        self._last_time = None
    
    def compute(self, setpoint: float, measurement: float,
                now: float = None) -> float:
        import time
        if now is None:
            now = time.time()
        
        if self._last_time is None:
            self._last_time = now
            self._prev_meas = measurement
            return 0.0
        
        dt = now - self._last_time
        if dt <= 0:
            return 0.0
        
        error = setpoint - measurement
        
        # Deadband
        if abs(error) < self._deadband:
            error = 0.0
        
        # Proportional
        p = self._kp * error
        
        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self._integral_max,
                             min(self._integral_max, self._integral))
        i = self._ki * self._integral
        
        # Derivative on measurement, filtered
        raw_d = -(measurement - self._prev_meas) / dt
        self._deriv_filtered = (self._alpha * self._deriv_filtered
                                + (1 - self._alpha) * raw_d)
        d = self._kd * self._deriv_filtered
        
        output = p + i + d
        output_clamped = max(self._out_min, min(self._out_max, output))
        
        # Anti-windup: undo integral if saturated
        if (output_clamped == self._out_max and error > 0) or \
           (output_clamped == self._out_min and error < 0):
            self._integral -= error * dt
        
        self._prev_meas = measurement
        self._last_time = now
        
        return output_clamped
```

---

## 10. Control Loop Architecture

```
[Setpoint Topic]         [Sensor Topic /duburi/attitude]
       ↓                          ↓
  [PIDController.compute(setpoint, measurement)]
       ↓
  [PWM output]
       ↓
  [/duburi/rc_override topic]
       ↓
  [driver_node → RC_CHANNELS_OVERRIDE → Pixhawk]
```

**Update rates:**
- Depth PID: 10 Hz (AHRS2 rate)
- Heading PID: 10 Hz
- Position PID: 20–100 Hz (DVL rate)
- RC override send: same as PID rate
- Heartbeat: 2 Hz (independent timer)

**Rule**: Never block the control loop. Use `recv_match(blocking=False)` or cached `master.messages`.

---

## 11. ArduSub Internal Control (What Firmware Does For Us)

ArduSub has its own internal PIDs. The live policy in this codebase
is "ArduSub does the inner loop everywhere":

| Axis           | Live policy (this repo)                   | When you'd reach for a custom Python PID instead         |
|----------------|-------------------------------------------|----------------------------------------------------------|
| Depth          | ArduSub ALT_HOLD via `SET_POSITION_TARGET_GLOBAL_INT` (the only path) | Hot-fix only — if onboard depth-hold becomes unreliable on real hardware. The deleted `DepthPID` (recoverable from git history) is a starting point; the math earlier in this file is the rebuild guide. |
| Yaw            | ArduSub attitude PID via `SET_ATTITUDE_TARGET`, optionally with `smoothstep` setpoint sweep (`smooth_yaw:=true`) | Vision-guided centering (out of scope here; lives in future `duburi_vision`) |
| Motor mixing   | ArduSub (8-thruster `vectored_6dof`)      | Never — let firmware do this                             |
| IMU fusion     | ArduSub EKF3                              | Never — let firmware do this                             |
| Translation    | `RC_CHANNELS_OVERRIDE` Ch5 (forward) / Ch6 (lateral) @ 20 Hz, optional `trapezoid_ramp` (`smooth_translate:=true`); `arc` adds Ch4 yaw stick to Ch5 in the same packet | When DVL lands, replace open-loop time with closed-loop distance |

**Best practice (today):** Use ArduSub's ALT_HOLD for both depth *and*
absolute yaw setpoints. Stream setpoints; never close a Python loop
in parallel with an ArduSub one. The two `smooth_*` ROS params shape
the *setpoint*, not the inner loop.
