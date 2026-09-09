"""FlowPositionSource -- the bottom camera standing in for the DVL.

`drive_forward_dist` / `drive_lateral_dist` need exactly two methods from
`yaw_source`: `get_position()` and `reset_position()`. That contract is
duck-typed, which is what lets flow supply it WITHOUT EDITING THE MOTION
LAYER -- and not editing it matters, because those files carry the runaway
guards `main` added after measuring 11.3 m of travel on a 1.0 m command and an
open-loop fallback that drove 2.361 m for 1.0 m while reporting "completed".
Every one of those guards keeps working here, unchanged.

WHAT THIS IS. A thin wrapper that DELEGATES everything to the real yaw source
(heading, calibration, connect, whatever it has) and adds position from
`NavEstimator`, fed by `<ns>/velocity` off `flow_node`.

⛔ WHAT IT IS NOT: A DVL. Say the numbers out loud, because the verb it plugs
into was written against a sensor an order of magnitude better:

    DVL-aided INS      ~0.08 % of distance travelled
    Nortek bottom-track 0.5-1 % of measured velocity
    this, in water      UNMEASURED

Dry-bench flow on a 50 cm slide came out at 103.4 % of truth with a +-7 %
height uncertainty, and that was a hand-slid rig in air with a known height.
In water the flat-port focal length changes by 44 %, the height comes from a
barometer minus a pool depth someone typed in, and nothing has been validated
against a tape. So this source REFUSES rather than starts when it cannot
justify a position, and the honest summary is: it makes the distance verbs
runnable without a DVL, and a pool session decides whether they are usable.

⛔ AND THE TRAP THIS AVOIDS. It would be easy to have `get_position()` keep
returning the last good value when flow goes quiet. That converts "I do not
know where I am" into "I have not moved", which is a measurement -- and the
one the caller acts on. A stalled position reads as arrival to a tolerance
check. So staleness is surfaced, not smoothed.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import UInt8

from duburi_vision.stamps import capture_monotonic

from .estimator.nav_estimator import NavEstimator

# Refuse to ARM a distance move if the last velocity fix is older than this.
# It is generous on purpose: the point is to catch "the flow node is not
# running / the lens is covered / the floor has no texture", not to police
# individual intervals, which `flow_node`'s own gates already do.
FIX_STALE_S = 1.0

# Refuse to arm if dead-reckoned position uncertainty already exceeds this.
# Position drifts without bound, so the only honest test is against the
# filter's own growing sigma -- `NavState.position_trustworthy` is that test.
POS_SIGMA_MAX_M = 0.50


class FlowPositionSource:
    """Wrap a yaw source; add flow-derived position. Delegates the rest."""

    name = 'flow'

    def __init__(self, node, inner, camera: str = 'downward',
                 fix_stale_s: float = FIX_STALE_S,
                 pos_sigma_max_m: float = POS_SIGMA_MAX_M):
        self._node = node
        self._inner = inner
        self._log = node.get_logger()
        self._fix_stale_s = float(fix_stale_s)
        self._pos_sigma_max = float(pos_sigma_max_m)

        self._nav = NavEstimator()
        self._lock = threading.Lock()
        self._last_fix_t: Optional[float] = None
        self._last_quality = 0
        self._n_fix = 0
        self._n_zero_quality = 0
        self._n_stamp_fallback = 0
        self._warned_stamp = False
        # Heading at the last reset_position(). The DVL contract is
        # "body-frame position since last reset", and NavEstimator integrates
        # in a LOCAL frame, so the two differ by exactly this rotation. Without
        # it a hull that yaws mid-move reports its displacement in the wrong
        # axes -- forward travel would leak into the lateral channel.
        self._ref_yaw_rad = 0.0

        ns = f'/duburi/vision/{camera}'
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._sub_vel = node.create_subscription(
            TwistWithCovarianceStamped, f'{ns}/velocity', self._on_velocity, qos)
        self._sub_q = node.create_subscription(
            UInt8, f'{ns}/flow_quality', self._on_quality, qos)
        self._log.info(
            f'[FLOWP] position from {ns}/velocity (NOT a DVL -- in-water '
            f'accuracy is unvalidated; refuses above {self._pos_sigma_max:.2f} m '
            f'of position sigma or {self._fix_stale_s:.1f} s without a fix)')

    # Anything we do not implement belongs to the real yaw source.
    def __getattr__(self, item):
        return getattr(self._inner, item)

    # ── inputs ──────────────────────────────────────────────────────────────
    def _yaw_rad(self) -> float:
        try:
            y = self._inner.read_yaw()
        except Exception:
            return 0.0
        return math.radians(y) if y is not None and math.isfinite(y) else 0.0

    def _on_quality(self, msg: UInt8) -> None:
        q = int(msg.data)
        with self._lock:
            self._last_quality = q
            if q == 0:
                self._n_zero_quality += 1

    def _on_velocity(self, msg: TwistWithCovarianceStamped) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        # The node publishes a covariance derived from the flow itself
        # (scale^2 * var/N), so the filter trusts a fix taken over a textured
        # floor more than one over a bare patch. A constant here would throw
        # that away -- which is the whole reason it is computed.
        var = float(msg.twist.covariance[0])
        sigma = math.sqrt(var) if var > 0.0 else 0.05
        # ⛔ WHEN THIS VELOCITY HAPPENED, NOT WHEN IT ARRIVED. `flow_node`
        # stamps at the CAPTURE-INTERVAL MIDPOINT -- flow gives an average
        # velocity over an interval, so it belongs at the middle, and with an
        # adaptive baseline reaching 0.75 s that correction is worth up to
        # 375 ms. Reading `time.monotonic()` here threw all of that away and
        # replaced it with host arrival jitter, measured at 30.85 ms p2p
        # against 0.000 ms of board jitter.
        #
        # This is the FIFTH instance of the defect `duburi_vision.stamps`
        # exists to stop (camera_node, vision_state, srot_replay, lock_node),
        # and the worst placed: dt here multiplies velocity into POSITION, so
        # the error integrates instead of decaying. Every one of those flatters
        # -- a too-small age reads as fresher than earned.
        t, why = capture_monotonic(msg.header)
        if why:
            self._n_stamp_fallback += 1
            if not self._warned_stamp:
                self._warned_stamp = True
                self._log.warning(
                    f'[FLOWP] velocity stamp unusable ({why}) -- integrating '
                    f'on ARRIVAL time. Position now carries host scheduling '
                    f'jitter; treat distances as indicative only.')
        yaw = self._yaw_rad()
        with self._lock:
            self._nav.predict(t, yaw)
            if self._nav.update_velocity(vx, vy, sigma):
                self._n_fix += 1
                self._last_fix_t = t

    # ── the DVL contract ────────────────────────────────────────────────────
    def get_position(self) -> Tuple[float, float]:
        """(x, y) metres in the frame latched at `reset_position()`.

        x is along the heading held at the reset, y to its right -- the same
        semantics `NucleusDVLSource.get_position` documents, so the motion
        layer cannot tell the two apart.
        """
        with self._lock:
            self._nav.predict(time.monotonic(), self._yaw_rad())
            st = self._nav.state()
            ref = self._ref_yaw_rad
        c, s = math.cos(ref), math.sin(ref)
        # Rotate the local displacement back into the latched frame.
        x = st.px * c + st.py * s
        y = -st.px * s + st.py * c
        return float(x), float(y)

    def reset_position(self) -> None:
        """Make here the origin -- and REFUSE if flow cannot justify one.

        This is the arming step of every distance verb, so it is the right
        place to fail: refusing here aborts the move before the thrusters run,
        where refusing mid-move would leave the hull travelling on an estimate
        nobody trusts.
        """
        ok, why = self.position_ready()
        if not ok:
            from duburi_control.errors import MovementError
            raise MovementError(
                f'flow position: {why}. The bottom camera is the position '
                f'source on this vehicle and it cannot supply one right now, '
                f'so the distance cannot be measured. Check the flow node is '
                f'running with pool_depth_m set and the lens sees texture '
                f'(ros2 topic echo /duburi/vision/downward/flow_quality), or '
                f'use the explicitly timed move_forward instead.')
        with self._lock:
            self._nav.reset_position()
            self._ref_yaw_rad = self._yaw_rad()
            self._log.info(
                f'[FLOWP] origin latched at yaw={math.degrees(self._ref_yaw_rad):.1f} '
                f'(quality={self._last_quality}, {self._n_fix} fixes)')

    # ── health ──────────────────────────────────────────────────────────────
    def position_ready(self) -> Tuple[bool, str]:
        """Can this source justify a position right now? (ok, why_not)."""
        with self._lock:
            last = self._last_fix_t
            q = self._last_quality
            st = self._nav.state()
        if last is None:
            return False, ('no velocity fix has ever arrived (is flow_node '
                           'running?)')
        age = time.monotonic() - last
        if age > self._fix_stale_s:
            return False, (f'the last velocity fix is {age:.1f} s old '
                           f'(quality={q}); flow is refusing intervals')
        if not st.position_trustworthy(self._pos_sigma_max):
            return False, (f'position sigma {st.pos_sigma:.2f} m exceeds the '
                           f'{self._pos_sigma_max:.2f} m limit after '
                           f'{st.n_fixes} fixes')
        return True, ''

    def status(self) -> dict:
        with self._lock:
            st = self._nav.state()
            q, last = self._last_quality, self._last_fix_t
            zero = self._n_zero_quality
            stamp_fb = self._n_stamp_fallback
        return {
            'quality': q,
            'fix_age_s': (time.monotonic() - last) if last else float('inf'),
            'n_fixes': st.n_fixes,
            'n_rejected': st.n_rejected,
            'n_zero_quality': zero,
            'n_stamp_fallback': stamp_fb,
            'pos_sigma_m': st.pos_sigma,
            'vel_sigma_ms': st.vel_sigma,
            'speed_ms': st.speed,
        }

    def close(self) -> None:
        """Close our subscriptions AND the source we wrap.

        ⛔ `close` is the one method that exists on BOTH this wrapper and the
        inner source, so `__getattr__` never fires for it -- delegation works
        everywhere else precisely because those attributes are absent here.
        The manager shuts down with `node.yaw_source.close()`, which after
        wrapping reaches this method, so without the explicit call below the
        real source is never closed: a BNO085 leaves its serial port open, a
        Nortek leaves its TCP session open, and nothing reports either.
        """
        for sub in (getattr(self, '_sub_vel', None), getattr(self, '_sub_q', None)):
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
