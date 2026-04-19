#!/usr/bin/env python3
"""Pure-math motion profiles. No ROS / MAVLink deps -- trivially unit-testable.

Used by:
  motion_yaw.yaw_glide                          -> smootherstep setpoint sweep
  motion_forward.drive_forward_eased / arc      -> trapezoid_ramp thrust envelope
  motion_lateral.drive_lateral_eased            -> trapezoid_ramp thrust envelope
"""


def smoothstep(t: float) -> float:
    """
    Hermite cubic: 3t² - 2t³  on  t ∈ [0, 1]
    C¹ continuous (velocity is zero at t=0 and t=1). Peak derivative 1.5.
    """
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def smootherstep(t: float) -> float:
    """
    Perlin quintic: 6t⁵ - 15t⁴ + 10t³  on  t ∈ [0, 1]
    C² continuous (velocity AND acceleration are zero at both endpoints).
    Peak derivative 1.875. The extra smoothness matters for yaw because
    ArduSub's attitude stabiliser differentiates the setpoint to get rate —
    a smoothstep corner would appear as a rate-command spike.
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def trapezoid_ramp(elapsed: float, total: float, ramp_sec: float) -> float:
    """
    Thrust envelope with smooth edges. Returns a scale factor in [0, 1]:

        ┌─ smootherstep ease-in for the first `ramp_sec`
        ├─ cruise at 1.0 for the middle
        └─ smootherstep ease-out for the last `ramp_sec`

    Outside the window or with non-positive duration, returns 0.
    If `total` < 2·ramp_sec the ramp collapses symmetrically into a
    triangle (no cruise phase) so short moves still end with zero scale.
    """
    if total <= 0.0 or elapsed <= 0.0 or elapsed >= total:
        return 0.0
    r = min(ramp_sec, total / 2.0)
    if elapsed < r:
        return smootherstep(elapsed / r)
    if elapsed > total - r:
        return smootherstep((total - elapsed) / r)
    return 1.0
