"""Timing. The thing that decides whether de-rotation helps or hurts.

⛔ WHY THIS MODULE EXISTS, in one number. De-rotation subtracts `f·ω·Δt`, so a
timing error between the camera and the gyro leaves uncorrected flow
PROPORTIONAL TO ω. Measured on this rig: 5 ms costs 1.6 px at 0.64 rad/s and
2.9 px at 1.13. Qin & Shen (IROS 2018) put the tolerance for an uncompensated
offset at **6 ms** before VIO degrades substantially.

Measured on OUR vehicle, host arrival of ATTITUDE at 50 Hz:

    board interval   median 20.00 ms   sd 0.00 ms     <- the board's clock is exact
    host  interval   median 20.00 ms   sd 6.67 ms     <- what we inherit by
                                       p2p 35.12 ms      stamping on arrival

We were outside the tolerance before counting any offset at all, and the
jitter is entirely transport: ESP32 UART FIFO thresholding plus USB-serial
scheduling. The board's own `time_boot_ms` has none of it and we were
discarding it -- it was read only by the reboot detector.

THE ORDER HERE IS DELIBERATE: remove what can be KNOWN before estimating what
cannot. Three deterministic errors, then one estimator for the remainder.

  1. `ClockMap`      board time -> host time, without TIMESYNC (the board does
                     not implement it -- 0 of 12 replies, measured).
  2. exposure/2      the image timestamp convention is MID-EXPOSURE, and ours
                     runs on AUTO exposure, so this term MOVES with the light.
  3. interval mid    an adaptive-baseline flow velocity is an AVERAGE over its
                     interval and belongs at the midpoint, not the end.
  4. `TimeOffset`    trace correlation + parabolic fit for what is left.

Every one of the first three is larger than the fourth.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional, Sequence, Tuple

import numpy as np


class ClockMap:
    """Map a remote device clock onto the host clock. Offset AND skew.

    ⛔ THE PROBLEM. MAVLink `TIMESYNC` would solve this with a ping-pong, and
    the srot board does not implement it: 0 of 12 requests answered, measured.
    So the only signal available is one-way -- pairs of (board_time,
    host_arrival) -- and the transport delay in between is variable and
    unknown.

    THE METHOD, and it is not a least-squares fit. Transport delay is strictly
    NON-NEGATIVE and mostly small, so `host - board` is the true offset PLUS a
    positive, heavy-tailed delay. A regression through the middle of that
    cloud estimates the mean delay, not the offset, and every scheduling
    hiccup drags it further. The samples that carry the least delay are the
    ones nearest the truth, so the answer lies on the LOWER ENVELOPE of the
    cloud -- the convex-hull / minimum-delay construction from the network
    clock-synchronisation literature, where only the lower boundary is
    relevant to skew.

    Measured here: the board-vs-host difference drifted -31.76 ms over 6 s, so
    a pure offset with no skew term would be wrong by 5 ms after one second.
    Both are estimated.

    It reports `ready` rather than guessing: with too few samples, or a window
    too short to separate skew from offset, the honest answer is that the
    mapping is not known yet.
    """

    __slots__ = ('_pairs', '_window_s', '_min_pairs', '_skew', '_offset',
                 '_n_fit', '_resid_ms', 'steps')

    # ⛔ A STEP IN THE HOST CLOCK IS NOT DELAY. `host - board` is offset plus a
    # NON-NEGATIVE transport delay, so it can rise by a stall but can only FALL
    # by as much delay as the previous sample carried -- milliseconds. A fall
    # past STEP_BACK_S, or a rise past STEP_FWD_S, is the host clock jumping
    # (measured 2026-09-14: the Pi's first NTP sync stepped CLOCK_REALTIME
    # mid-run). The pairs before it describe a different clock and are dropped.
    # Without this a BACKWARD step pruned nothing, since pruning is by host
    # age, and the fit mixed both clocks for the size of the step plus the
    # window -- hours for an hour.
    STEP_BACK_S = 0.5
    STEP_FWD_S = 2.0

    def __init__(self, window_s: float = 20.0, min_pairs: int = 40):
        self._pairs: deque = deque()          # (board_s, host_s)
        self._window_s = float(window_s)
        self._min_pairs = int(min_pairs)
        self._skew = 1.0                      # host seconds per board second
        self._offset = 0.0
        self._n_fit = 0
        self._resid_ms = float('nan')
        self.steps = 0                        # host-clock steps detected

    def add(self, board_s: float, host_s: float) -> None:
        if not (math.isfinite(board_s) and math.isfinite(host_s)):
            return
        if self._pairs:
            lb, lh = self._pairs[-1]
            jump = (host_s - board_s) - (lh - lb)
            if jump < -self.STEP_BACK_S or jump > self.STEP_FWD_S:
                self._pairs.clear()
                self._n_fit = 0               # the old fit is the other clock's
                self.steps += 1
        self._pairs.append((board_s, host_s))
        while self._pairs and (host_s - self._pairs[0][1]) > self._window_s:
            self._pairs.popleft()

    def fit(self) -> bool:
        """Refit from the lower envelope. Returns whether the fit is usable."""
        n = len(self._pairs)
        if n < self._min_pairs:
            return False
        b = np.fromiter((p[0] for p in self._pairs), float, n)
        h = np.fromiter((p[1] for p in self._pairs), float, n)
        span = b[-1] - b[0]
        if span < 1.0:
            # Skew and offset are not separable over a short window: a small
            # slope error is indistinguishable from a small offset error.
            return False
        b0 = b - b[0]
        d = h - b                         # offset + (positive) transport delay

        # Lower envelope by bucketed minima: within each slice take the
        # least-delayed sample, then fit a line through those. Bucketing keeps
        # it linear in n and stops one lucky sample from setting the slope.
        nb = max(4, min(24, int(span / 0.5)))
        edges = np.linspace(0.0, b0[-1] + 1e-9, nb + 1)
        idx = np.clip(np.digitize(b0, edges) - 1, 0, nb - 1)
        xs, ys = [], []
        for k in range(nb):
            m = idx == k
            if not m.any():
                continue
            j = np.argmin(d[m])
            xs.append(b0[m][j])
            ys.append(d[m][j])
        if len(xs) < 3:
            return False
        xs = np.asarray(xs)
        ys = np.asarray(ys)
        A = np.vstack([xs, np.ones_like(xs)]).T
        try:
            slope, intercept = np.linalg.lstsq(A, ys, rcond=None)[0]
        except np.linalg.LinAlgError:
            return False
        # The fit is d(board) = slope*(board - b0) + intercept, where
        # d = host - board. So
        #     host = board + slope*(board - b0) + intercept
        #          = board*(1 + slope) + (intercept - slope*b0)
        self._skew = 1.0 + float(slope)
        self._offset = float(intercept - slope * b[0])
        self._n_fit = len(xs)
        self._resid_ms = float(np.median(np.abs(ys - (slope * xs + intercept)))
                               * 1000.0)
        return True

    def to_host(self, board_s: float) -> float:
        """Board clock -> host clock."""
        return board_s * self._skew + self._offset

    @property
    def ready(self) -> bool:
        return self._n_fit >= 3

    @property
    def skew_ppm(self) -> float:
        return (self._skew - 1.0) * 1e6

    @property
    def n_pairs(self) -> int:
        return len(self._pairs)

    @property
    def residual_ms(self) -> float:
        return self._resid_ms

    def __repr__(self):
        return (f'ClockMap(ready={self.ready} pairs={self.n_pairs} '
                f'skew={self.skew_ppm:+.0f}ppm resid={self._resid_ms:.2f}ms)')


def exposure_offset_s(exposure_time_absolute: Optional[float]) -> float:
    """Half the exposure, in seconds. The MID-EXPOSURE convention.

    ⛔ NOT A REFINEMENT, AND NOT CONSTANT. The standard image timestamp is the
    MIDDLE of the exposure, because the average position of everything in the
    frame is where it was when half the exposure had elapsed. The literature
    states the camera-IMU offset as "fixed communication and triggering delays
    PLUS HALF EXPOSURE TIME".

    Ours is 15.7 ms on auto exposure in this light -- **7.85 ms**, larger than
    the transport jitter and larger than the offset most papers estimate. And
    because it is AUTO, it MOVES: someone measured calibrated offsets shifting
    by 2 ms as exposure went 1 -> 2 -> 4 ms. So this is read live per frame
    rather than baked in, and the argument is in V4L2's own units (100 µs),
    which is what `exposure_time_absolute` reports.
    """
    if exposure_time_absolute is None:
        return 0.0
    try:
        e = float(exposure_time_absolute)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(e) or e <= 0.0:
        return 0.0
    return (e * 1e-4) * 0.5           # V4L2 units are 100 us


def interval_midpoint(t_start: float, t_end: float) -> float:
    """When an average velocity over [t_start, t_end] actually happened.

    ⛔ THE BIGGEST TIMING ERROR IN THIS PIPELINE, and it is not subtle.
    Flow gives DISPLACEMENT over an interval, so dividing by dt gives the
    AVERAGE velocity across it -- which belongs at the midpoint. PX4 defines
    its own flow delay parameter exactly this way: "to the middle of the
    optical flow integration interval".

    We stamp at the interval's END, and our baseline is ADAPTIVE: it stretches
    until ~8 px of displacement has accumulated, reaching `max_baseline_s`
    (0.75 s) when the hull is slow. So the error is up to **375 ms** -- two
    orders of magnitude beyond the 6 ms tolerance -- and it is WORST exactly
    when the vehicle is station-keeping, which is the regime the sensor exists
    for.
    """
    return 0.5 * (float(t_start) + float(t_end))


class TimeOffset:
    """Camera<->gyro offset by trace correlation with a parabolic peak fit.

    ⛔ WHY THIS METHOD AND NOT THE OTHERS. From the 2025 review of camera /
    LiDAR / IMU spatiotemporal calibration:

        hardware sync            best, needs a strobe line          we have none
        cross-correlation (Mair) 360 ms error at large offsets      fails
        phase congruency         19.3 ms
        trace corr + quadratic   **< 0.5 ms**, robust to large      <- this
        filter-based (Li)        0.1 - 1.5 ms, needs a noise model
        Qin optimisation         0.01 - 0.3 ms, needs a VIO bundle  no optimiser
        Furgale batch            < 0.04 ms, 300 s convergence

    We do not run a bundle adjustment, so the optimisation-based methods do not
    apply. What we DO have is two independent measurements of one scalar: the
    image-derived yaw rate from the planar rigid fit, and the board's gyro. That
    is exactly the input trace correlation wants, and it reaches sub-millisecond
    accuracy without an estimator, a noise model, or a target.

    THE PARABOLIC FIT IS WHAT MAKES IT SUB-SAMPLE. The correlation is evaluated
    on a discrete lag grid, so the raw peak is quantised to one step. Fitting a
    parabola through the peak and its two neighbours recovers the true maximum
    between samples -- the standard sub-sample time-delay estimator.

    OBSERVABILITY, stated because it decides the procedure. Li & Mourikis prove
    td is identifiable except in degenerate motions, and **constant velocity is
    degenerate** -- a steady turn tells you nothing. The excitation needed is
    CHANGING angular rate, and the literature's own recipe is "applying
    sinusoidal signals to move the camera around the z-axis". For a downward
    camera that is a gentle oscillation about the optical axis: a few seconds
    of it, by hand, is enough. No rig translation, no target, no pool.
    """

    __slots__ = ('_img', '_gyro', '_max_lag_s', '_min_rms', '_td', '_quality',
                 '_n', '_peak_corr')

    def __init__(self, max_lag_s: float = 0.20, min_rms: float = 0.05):
        self._img: deque = deque(maxlen=4096)     # (t, yaw_rate)
        self._gyro: deque = deque(maxlen=8192)
        self._max_lag_s = float(max_lag_s)
        self._min_rms = float(min_rms)
        self._td = 0.0
        self._quality = 0.0
        self._n = 0
        self._peak_corr = 0.0

    def add_image_yaw(self, t: float, yaw_rate: float) -> None:
        if math.isfinite(t) and math.isfinite(yaw_rate):
            self._img.append((float(t), float(yaw_rate)))

    def add_gyro_yaw(self, t: float, yaw_rate: float) -> None:
        if math.isfinite(t) and math.isfinite(yaw_rate):
            self._gyro.append((float(t), float(yaw_rate)))

    def estimate(self, step_s: float = 0.002) -> Optional[float]:
        """Return td in seconds, or None.

        ⛔ SIGN CONVENTION, stated because a sign error here steers the
        de-rotation the WRONG WAY and doubles the residual instead of removing
        it -- and it produces a plausible number while doing so.

            td > 0  means the IMAGE timestamps are LATE relative to the gyro:
                    an image stamped t actually shows the world at t - td.

        So the correction is to look the gyro up at `t_image - td` (equivalently
        to subtract td from image stamps). The correlation peak itself lands at
        lag = -td, because comparing image(t) against gyro(t + lag) matches when
        t - td = t + lag; the negation below is that identity, not a fudge, and
        a unit test pins it against an injected delay of known sign.

        None means "not enough evidence", never 0.0 -- a zero offset and an
        unmeasurable one are different claims, and only one is safe to act on.
        """
        if len(self._img) < 20 or len(self._gyro) < 40:
            return None
        it = np.fromiter((p[0] for p in self._img), float, len(self._img))
        iv = np.fromiter((p[1] for p in self._img), float, len(self._img))
        gt = np.fromiter((p[0] for p in self._gyro), float, len(self._gyro))
        gv = np.fromiter((p[1] for p in self._gyro), float, len(self._gyro))

        # EXCITATION GATE. Constant (or absent) rotation is degenerate: the
        # correlation of two flat signals peaks wherever noise says, and the
        # parabola will happily interpolate a confident answer out of it.
        if float(np.std(iv)) < self._min_rms or float(np.std(gv)) < self._min_rms:
            self._quality = 0.0
            return None

        lo = max(it[0], gt[0]) + self._max_lag_s
        hi = min(it[-1], gt[-1]) - self._max_lag_s
        if hi - lo < 1.0:
            return None
        # RESAMPLE GRID = 2 ms, and it is the binding quantisation, not the
        # lag step. Measured against injected offsets from 0 to 150 ms:
        #
        #     grid   step    mean |err|   max |err|
        #     10 ms  2 ms      0.259 ms    0.552 ms
        #      5 ms  2 ms      0.216 ms    0.394 ms
        #      2 ms  2 ms      0.195 ms    0.336 ms   <- knee
        #      1 ms  2 ms      0.195 ms    0.337 ms   (no further gain)
        #
        # A FINER LAG STEP MAKES IT WORSE, which is worth stating because the
        # instinct is to shrink both: at a 10 ms grid, going 2 -> 0.5 ms took
        # the mean from 0.259 to 0.451. The parabola is fitted to three points
        # spanning the peak, and steps far narrower than the interpolation
        # scale sample the peak's flat top instead of its curvature, so the
        # vertex is fitted to noise. Coarse steps + a fine grid + the parabola
        # is the combination that wins.
        grid = np.arange(lo, hi, 0.002)
        if len(grid) < 20:
            return None
        a = np.interp(grid, it, iv)
        a = a - a.mean()
        na = np.linalg.norm(a)
        if na < 1e-9:
            return None

        lags = np.arange(-self._max_lag_s, self._max_lag_s + step_s, step_s)
        corr = np.empty(len(lags))
        for k, lag in enumerate(lags):
            b = np.interp(grid + lag, gt, gv)
            b = b - b.mean()
            nb = np.linalg.norm(b)
            corr[k] = float(a @ b) / (na * nb) if nb > 1e-9 else 0.0

        k = int(np.argmax(corr))
        if k == 0 or k == len(corr) - 1:
            # The true peak is outside the search window; reporting the edge
            # would be reporting the window, not the offset.
            self._quality = 0.0
            return None
        # PARABOLIC SUB-SAMPLE PEAK. y = ax^2+bx+c through three points; the
        # vertex offset is the classic (y- - y+) / (2*(y- - 2y0 + y+)).
        y0, ym, yp = corr[k], corr[k - 1], corr[k + 1]
        den = (ym - 2.0 * y0 + yp)
        delta = 0.5 * (ym - yp) / den if abs(den) > 1e-12 else 0.0
        delta = max(-1.0, min(1.0, delta))
        # See the sign note above: the peak sits at lag = -td.
        self._td = -float(lags[k] + delta * step_s)
        self._peak_corr = float(y0)
        self._n = len(grid)
        # QUALITY = how high the peak is AND how much it stands out. A high
        # correlation with a flat top means the lag is poorly determined even
        # though the signals agree, so prominence against the body of the
        # curve is the part that matters -- measuring only the peak height
        # would call a broad, ambiguous maximum excellent.
        prominence = float(y0 - np.percentile(corr, 10.0))
        self._quality = (max(0.0, min(1.0, y0))
                         * max(0.0, min(1.0, prominence * 1.5)))
        return self._td

    @property
    def td(self) -> float:
        return self._td

    @property
    def quality(self) -> float:
        return self._quality

    @property
    def peak_correlation(self) -> float:
        return self._peak_corr

    def clear(self) -> None:
        self._img.clear()
        self._gyro.clear()
