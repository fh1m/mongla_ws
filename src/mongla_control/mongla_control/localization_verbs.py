"""Localization verbs for the Mongla facade: `locate` and `go_to`.

`locate` reads `/mongla/localization/state` and says whether the estimate is
usable, and why not. `go_to` drives to a pool-frame point on that estimate.

`go_to` streams MANUAL_CONTROL in STABILIZE, as the vision verbs do: the board
holds depth and heading at 500 Hz and this loop adds surge and sway, rotated
from the pool error into the body by the filter's heading. It never commands
yaw or depth: `turn` and `set_depth` do those, on the board.

It refuses rather than guesses. No state, an unusable state (IMU silent,
heading not anchored, position unaided), or a position known worse than the
arrival tolerance: each is refused before a thruster moves, and the same
checks run every tick, so an estimate that goes bad mid-run stops the hull.
A 'blocked' motion verdict (thrust commanded, floor not moving) stops it too.

BUILT, NEVER FLOWN. `GO_TO_FULL_SPEED_M` is a starting value, not a
measurement: the first pool run sets it (measured-bars, "go_to").

Mixed into `Mongla`; the state arrives through `localization_provider`,
injected by the manager, so this package stays rclpy-free.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .errors import MovementError
from .motion_vision.srot_drive import _srot_drive

# The error at which `go_to` reaches its speed cap; closer in, the demand
# falls linearly to zero at the goal. Not measured (see the module docstring).
GO_TO_FULL_SPEED_M = 1.0
# The loop rate: the state topic's own 10 Hz, so every tick sees a new estimate.
GO_TO_PERIOD_S = 0.1

ARRIVED, TIMEOUT, LOST, BLOCKED, ABORTED, REFUSED = range(6)
_NAMES = {ARRIVED: 'ARRIVED', TIMEOUT: 'TIMEOUT', LOST: 'LOST',
          BLOCKED: 'BLOCKED', ABORTED: 'ABORTED', REFUSED: 'REFUSED'}


def describe(s) -> str:
    """One operator line for a LocalizationState (or None)."""
    if s is None:
        return ('no localization state: localization:=false, or the node '
                'stopped publishing')
    pos = ('no position yet' if not math.isfinite(s.x_m) else
           f'({s.x_m:+.2f}, {s.y_m:+.2f}) m in {s.frame}, depth {s.depth_m:+.2f} m, '
           f'yaw {s.yaw_deg:+.1f} deg, sigma {s.sigma_xy_m:.2f} m / '
           f'{s.sigma_yaw_deg:.1f} deg')
    head = 'USABLE' if s.usable else f'NOT USABLE: {s.reason}'
    return (f'{head} | {pos} | aiding={s.aiding or "?"} motion={s.motion or "?"} '
            f'flow {s.flow_fused} fused / {s.flow_refused} refused, '
            f'{s.fixes} fixes')


def refusal(s, tolerance_m: float) -> str:
    """Why `go_to` must not steer on this state, or ''. Pure."""
    if s is None:
        return describe(None)
    if not s.usable:
        return f'localization not usable: {s.reason}'
    if not math.isfinite(s.sigma_xy_m) or s.sigma_xy_m > tolerance_m:
        return (f'position known to {s.sigma_xy_m:.2f} m (1 sigma), worse than '
                f'the {tolerance_m:.2f} m tolerance: arriving could not be told '
                f'from missing. Fix position from a prop first')
    if s.motion == 'blocked':
        return 'motion is blocked: thrust commanded and the floor not moving'
    return ''


def body_demand(dx: float, dy: float, yaw_deg: float, gain_pct: float,
                full_speed_m: float = GO_TO_FULL_SPEED_M) -> tuple[float, float]:
    """(forward %, starboard %) toward a pool-frame error. Pure.

    Pool and body are both NED/FRD, so the error rotates into the body by the
    heading alone. The magnitude ramps to `gain_pct` at `full_speed_m` and is
    capped there, keeping the direction exact under the cap.
    """
    psi = math.radians(yaw_deg)
    fwd = math.cos(psi) * dx + math.sin(psi) * dy
    stb = -math.sin(psi) * dx + math.cos(psi) * dy
    dist = math.hypot(fwd, stb)
    if dist == 0.0:
        return 0.0, 0.0
    mag = gain_pct * min(dist / full_speed_m, 1.0)
    return mag * fwd / dist, mag * stb / dist


@dataclass
class GoToOutcome:
    code: int
    reason: str
    remaining_m: float
    sigma_m: float
    elapsed_s: float

    @property
    def name(self) -> str:
        return _NAMES[self.code]


def go_to_loop(*, fc, provider, x_m, y_m, tolerance_m, duration, gain,
               abort_fn, log, clock=time.monotonic, sleep=time.sleep,
               mixer_aware=True) -> GoToOutcome:
    """Drive until within `tolerance_m` of (x_m, y_m), or say why not."""
    t0 = clock()
    remaining = sigma = math.nan

    def done(code, why):
        return GoToOutcome(code, why, remaining, sigma, clock() - t0)

    while True:
        if abort_fn():
            return done(ABORTED, 'aborted')
        s = provider()
        why = refusal(s, tolerance_m)
        if why:
            return done(BLOCKED if 'blocked' in why else LOST, why)
        sigma = float(s.sigma_xy_m)
        dx, dy = x_m - s.x_m, y_m - s.y_m
        remaining = math.hypot(dx, dy)
        if remaining <= tolerance_m:
            return done(ARRIVED, f'within {remaining:.2f} m of the goal')
        if clock() - t0 >= duration:
            return done(TIMEOUT, f'{remaining:.2f} m short after {duration:.0f} s')
        fwd, stb = body_demand(dx, dy, s.yaw_deg, gain)
        _srot_drive(fc, fwd_pct=fwd, lat_pct=stb, yaw_pct=0.0,
                    prioritise=mixer_aware)
        sleep(GO_TO_PERIOD_S)


class LocalizationVerbs:
    """`locate` and `go_to`; mixed into `Mongla`."""

    def _localization_state(self):
        prov = getattr(self, 'localization_provider', None)
        return prov() if callable(prov) else None

    def locate(self):
        """Is localization usable, and where does it put the hull?

        `success` is `usable`; `final_value` is the horizontal sigma (NaN when
        there is none); the message is the whole state on one line.
        """
        with self._command_scope('locate'):
            s = self._localization_state()
            line = describe(s)
            self.log.info(f'[LOCAL] {line}')
            ok = bool(s is not None and s.usable)
            return self._make_result(
                ok, f'locate: {line}',
                final_value=float(s.sigma_xy_m) if s is not None else math.nan)

    def go_to(self, x_m, y_m, tolerance_m=0.5, duration=60.0, gain=30.0):
        """Drive to pool point (x_m, y_m) on the filter's estimate.

        `success` is ARRIVED; `final_value` the distance left, `error_value`
        the position sigma at the end. Refused (nothing moves) when the
        estimate is not usable or not known to within `tolerance_m`.
        """
        x_m, y_m, tol = float(x_m), float(y_m), float(tolerance_m)
        if not (math.isfinite(x_m) and math.isfinite(y_m)) or not tol > 0.0:
            return self._make_result(
                False, f'go_to: needs a finite x_m, y_m and tolerance_m > 0 '
                       f'(got {x_m}, {y_m}, {tol})', final_value=math.nan)
        with self._command_scope('go_to'):
            why = refusal(self._localization_state(), tol)
            if why:
                self.log.warning(f'[LOCAL] go_to refused: {why}')
                return self._make_result(False, f'go_to REFUSED: {why}',
                                         final_value=math.nan)
            from .vision_verbs import _require_srot_vision_mode
            try:
                _require_srot_vision_mode(self.fc, self.log, 'go_to')
            except MovementError as exc:
                return self._make_result(False, str(exc), final_value=math.nan)
            self.log.info(f'[LOCAL] go_to ({x_m:+.2f}, {y_m:+.2f}) m, '
                          f'tolerance {tol:.2f} m, gain {float(gain):.0f}%')
            out = go_to_loop(
                fc=self.fc, provider=self._localization_state,
                x_m=x_m, y_m=y_m, tolerance_m=tol,
                duration=float(duration), gain=float(gain),
                abort_fn=self._abort_fn, log=self.log,
                mixer_aware=bool(getattr(self, 'vision_mixer_aware', True)))
            self._send_neutral_and_settle()
            (self.log.info if out.code == ARRIVED else self.log.warning)(
                f'[LOCAL] go_to {out.name}: {out.reason}')
            return self._make_result(
                out.code == ARRIVED, f'go_to {out.name}: {out.reason}',
                final_value=out.remaining_m, error_value=out.sigma_m,
                elapsed_s=out.elapsed_s)
