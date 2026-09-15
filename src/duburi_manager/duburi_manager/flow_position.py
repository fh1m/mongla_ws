"""FlowPositionSource -- the bottom camera standing in for the DVL.

`drive_forward_dist` / `drive_lateral_dist` need exactly two methods from
`yaw_source`: `get_position()` and `reset_position()`. That contract is
duck-typed, which is what lets flow supply it WITHOUT EDITING THE MOTION
LAYER -- and not editing it matters, because those files carry the runaway
guards `main` added after measuring 11.3 m of travel on a 1.0 m command and an
open-loop fallback that drove 2.361 m for 1.0 m while reporting "completed".
Every one of those guards keeps working here, unchanged.

WHAT THIS IS. A thin wrapper that DELEGATES everything to the real yaw source
and adds a leg displacement integrated from the vehicle's ONE estimator: the
RIEKF's body velocity and attitude on `/duburi/odom`.

⛔ ONE ESTIMATOR, NOT TWO. This used to own a private `NavEstimator` fed by the
same flow topic the RIEKF already consumes. That filter saw flow and yaw; the
RIEKF sees flow with its per-sample variance, the IMU, depth, ZUPT when still,
the learned demand model when flow drops, prop fixes, and the floor grid. Two
filters on one topic give two velocities, and the worse one was driving the
distance verbs. So the private filter is gone and this reads the RIEKF.

WHY VELOCITY, NOT A DIFFERENCE OF POSITIONS. `/duburi/odom` also carries a
position, and `p_now - p_reset` looks simpler. It is wrong in the way that
matters here: a prop fix mid-leg corrects the absolute position, and the part
of that correction that was error laid down BEFORE the reset lands in the leg
displacement. Integrating velocity gives the displacement of THIS leg only.

WHY THE ROTATION USES THE ODOM ATTITUDE. The velocity is the RIEKF's, so it is
rotated by the RIEKF's attitude from the same message. Mixing it with
`yaw_source` would put one frame's velocity through another frame's heading.

⛔ WHAT IT IS NOT: A DVL.

    DVL-aided INS      ~0.08 % of distance travelled
    Nortek bottom-track 0.5-1 % of measured velocity
    this, in water      UNMEASURED

Dry-bench flow on a 50 cm slide came out at 103.4 % of truth with a +-7 %
height uncertainty. So this source REFUSES rather than starts when it cannot
justify a velocity, and a pool session decides whether it is usable.

⛔ AND THE TRAP THIS AVOIDS. Returning the last good value when the input goes
quiet converts "I do not know where I am" into "I have not moved", which reads
as arrival to a tolerance check. So staleness is surfaced, not smoothed.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt8

from duburi_vision.stamps import capture_monotonic

# Refuse to ARM a distance move if the last flow fix, or the last odometry, is
# older than this. It catches "flow_node / localization is not running, the
# lens is covered, the floor has no texture" -- not individual intervals.
FIX_STALE_S = 1.0

# Refuse to arm when the RIEKF's own horizontal velocity sigma exceeds this.
# Tied to the leg budget, not guessed separately: a velocity error that stays
# correlated over a leg grows position error as sigma_v * t, so 0.10 m/s is
# 0.5 m over a 5 s leg -- the position bound the old private filter enforced.
VEL_SIGMA_MAX_MS = 0.10

# An odometry interval longer than this is a GAP. Integrating it as one step
# would multiply a stale velocity by a long dt; it is skipped and counted.
MAX_INTEGRATE_DT_S = 0.25


def _yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate_xy(w, x, y, z, v):
    """World-frame (x, y) of body vector `v` under unit quaternion (w, x, y, z)."""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    vx, vy, vz = v
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
    )


class FlowPositionSource:
    """Wrap a yaw source; add a leg displacement from the RIEKF. Delegates the rest."""

    @property
    def name(self) -> str:
        """Report BOTH sources, because the wrap hides the heading one.

        `name` exists on both sides of the wrapper, so `__getattr__` never
        fires for it; a bare 'flow' would rename the operator's one statement
        of which HEADING sensor is live.
        """
        inner = getattr(self._inner, 'name', '?')
        return f'{inner}+flow'

    def __init__(self, node, inner, camera: str = 'downward',
                 fix_stale_s: float = FIX_STALE_S,
                 vel_sigma_max_ms: float = VEL_SIGMA_MAX_MS):
        self._node = node
        self._inner = inner
        self._log = node.get_logger()
        self._fix_stale_s = float(fix_stale_s)
        self._vel_sigma_max = float(vel_sigma_max_ms)

        self._lock = threading.Lock()
        # flow: freshness and quality only; the VALUE comes from the RIEKF
        self._last_fix_t: Optional[float] = None
        self._last_quality = 0
        self._n_fix = 0
        self._n_zero_quality = 0
        # odometry
        self._odom_t: Optional[float] = None        # capture instant, monotonic
        self._odom_rx: Optional[float] = None       # arrival, monotonic
        self._v_world = (0.0, 0.0)
        self._vel_sigma = math.inf
        self._yaw = 0.0
        self._n_odom = 0
        self._n_gap = 0
        self._n_stamp_fallback = 0
        self._warned_stamp = False
        # the leg
        self._dx = 0.0
        self._dy = 0.0
        self._leg_sigma = 0.0
        self._ref_yaw = 0.0

        ns = f'/duburi/vision/{camera}'
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub_vel = node.create_subscription(
            TwistWithCovarianceStamped, f'{ns}/velocity', self._on_velocity, qos)
        self._sub_q = node.create_subscription(
            UInt8, f'{ns}/flow_quality', self._on_quality, qos)
        self._sub_odom = node.create_subscription(
            Odometry, '/duburi/odom', self._on_odom, 10)
        self._log.info(
            f'[FLOWP] leg displacement from /duburi/odom velocity, gated on '
            f'{ns}/velocity (NOT a DVL -- in-water accuracy is unvalidated; '
            f'refuses above {self._vel_sigma_max:.2f} m/s of velocity sigma or '
            f'{self._fix_stale_s:.1f} s without flow or odometry)')

    # Anything we do not implement belongs to the real yaw source.
    def __getattr__(self, item):
        return getattr(self._inner, item)

    # ── inputs ──────────────────────────────────────────────────────────────
    def _on_quality(self, msg: UInt8) -> None:
        q = int(msg.data)
        with self._lock:
            self._last_quality = q
            if q == 0:
                self._n_zero_quality += 1

    def _on_velocity(self, msg: TwistWithCovarianceStamped) -> None:
        # ⛔ WHEN THE VELOCITY HAPPENED, NOT WHEN IT ARRIVED. `flow_node` stamps
        # the capture-interval midpoint; staleness is measured from there.
        t, _why = capture_monotonic(msg.header)
        with self._lock:
            self._n_fix += 1
            self._last_fix_t = t

    def _on_odom(self, msg: Odometry) -> None:
        # ⛔ dt from the STAMP. The RIEKF stamps its output with its latest
        # input's board-clock instant; arrival time would multiply host
        # scheduling jitter into position.
        t, why = capture_monotonic(msg.header)
        rx = time.monotonic()
        if why:
            self._n_stamp_fallback += 1
            if not self._warned_stamp:
                self._warned_stamp = True
                self._log.warning(
                    f'[FLOWP] odometry stamp unusable ({why}) -- integrating on '
                    f'ARRIVAL time. Treat distances as indicative only.')
        q = msg.pose.pose.orientation
        tw = msg.twist.twist.linear
        v_world = _rotate_xy(q.w, q.x, q.y, q.z, (tw.x, tw.y, tw.z))
        cov = msg.twist.covariance
        var = max(float(cov[0]), float(cov[7]))
        sigma = math.sqrt(var) if var > 0.0 and math.isfinite(var) else math.inf
        with self._lock:
            prev = self._odom_t
            if prev is not None:
                dt = t - prev
                if 0.0 < dt <= MAX_INTEGRATE_DT_S:
                    # Trapezoid: the two samples bracket the interval.
                    self._dx += 0.5 * (self._v_world[0] + v_world[0]) * dt
                    self._dy += 0.5 * (self._v_world[1] + v_world[1]) * dt
                    self._leg_sigma += sigma * dt
                elif dt > MAX_INTEGRATE_DT_S:
                    self._n_gap += 1
            self._odom_t, self._odom_rx = t, rx
            self._v_world = v_world
            self._vel_sigma = sigma
            self._yaw = _yaw_from_quat(q.w, q.x, q.y, q.z)
            self._n_odom += 1

    # ── the DVL contract ────────────────────────────────────────────────────
    def get_position(self) -> Tuple[float, float]:
        """(x, y) metres in the frame latched at `reset_position()`.

        x is along the heading held at the reset, y to its right -- the same
        semantics `NucleusDVLSource.get_position` documents. Extrapolated from
        the last odometry to now (capped), so a 10 Hz input does not read as a
        hull that pauses between messages.
        """
        with self._lock:
            dx, dy = self._dx, self._dy
            if self._odom_rx is not None:
                lag = min(max(time.monotonic() - self._odom_rx, 0.0),
                          MAX_INTEGRATE_DT_S)
                dx += self._v_world[0] * lag
                dy += self._v_world[1] * lag
            ref = self._ref_yaw
        c, s = math.cos(ref), math.sin(ref)
        return float(dx * c + dy * s), float(-dx * s + dy * c)

    def reset_position(self) -> None:
        """Make here the origin -- and REFUSE if the inputs cannot justify one.

        This is the arming step of every distance verb, so it is the right
        place to fail: refusing here aborts the move before the thrusters run.
        """
        ok, why = self.position_ready()
        if not ok:
            from duburi_control.errors import MovementError
            raise MovementError(
                f'flow position: {why}. The bottom camera is the position '
                f'source on this vehicle and it cannot supply one right now, '
                f'so the distance cannot be measured. Check flow_node is '
                f'running with pool_depth_m set and the lens sees texture '
                f'(ros2 topic echo /duburi/vision/downward/flow_quality) and '
                f'that localization:=true, or use the explicitly timed '
                f'move_forward instead.')
        with self._lock:
            self._dx = self._dy = self._leg_sigma = 0.0
            self._ref_yaw = self._yaw
            self._log.info(
                f'[FLOWP] origin latched at yaw={math.degrees(self._ref_yaw):.1f} '
                f'(quality={self._last_quality}, vel sigma '
                f'{self._vel_sigma:.3f} m/s)')

    # ── health ──────────────────────────────────────────────────────────────
    def position_ready(self) -> Tuple[bool, str]:
        """Can this source justify a displacement right now? (ok, why_not)."""
        now = time.monotonic()
        with self._lock:
            last, q = self._last_fix_t, self._last_quality
            odom_t, sigma = self._odom_t, self._vel_sigma
        if last is None:
            return False, ('no velocity fix has ever arrived (is flow_node '
                           'running?)')
        age = now - last
        if age > self._fix_stale_s:
            return False, (f'the last velocity fix is {age:.1f} s old '
                           f'(quality={q}); flow is refusing intervals')
        if odom_t is None:
            return False, ('no /duburi/odom has arrived (is the localization '
                           'node running?)')
        odom_age = now - odom_t
        if odom_age > self._fix_stale_s:
            return False, f'the last /duburi/odom is {odom_age:.1f} s old'
        if not sigma <= self._vel_sigma_max:
            return False, (f'velocity sigma {sigma:.3f} m/s exceeds the '
                           f'{self._vel_sigma_max:.2f} m/s limit')
        return True, ''

    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            return {
                'quality': self._last_quality,
                'fix_age_s': (now - self._last_fix_t) if self._last_fix_t else math.inf,
                'odom_age_s': (now - self._odom_t) if self._odom_t else math.inf,
                'n_fixes': self._n_fix,
                'n_odom': self._n_odom,
                'n_gap': self._n_gap,
                'n_zero_quality': self._n_zero_quality,
                'n_stamp_fallback': self._n_stamp_fallback,
                'vel_sigma_ms': self._vel_sigma,
                'leg_sigma_m': self._leg_sigma,
                'speed_ms': math.hypot(*self._v_world),
            }

    def close(self) -> None:
        """Close our subscriptions AND the source we wrap.

        ⛔ `close` exists on BOTH this wrapper and the inner source, so
        `__getattr__` never fires for it. Without the explicit call below the
        real source is never closed: a BNO085 leaves its serial port open, a
        Nortek its TCP session.
        """
        for attr in ('_sub_vel', '_sub_q', '_sub_odom'):
            sub = getattr(self, attr, None)
            if sub is not None:
                try:
                    self._node.destroy_subscription(sub)
                except Exception:
                    pass
        inner_close = getattr(self._inner, 'close', None)
        if callable(inner_close):
            try:
                inner_close()
            except Exception as exc:      # noqa: BLE001 -- shutdown is best-effort
                self._log.warning(f'[FLOWP] inner source close() failed: {exc}')
