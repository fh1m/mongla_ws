"""Per-track 4-state constant-velocity Kalman smoother.

State vector:  x = [cx, cy, vx, vy]^T
Measurement:   z = [cx, cy]

One KalmanFilter instance per active track_id. Filters are created
lazily on first measurement and pruned when a track drops.

This is an OUTPUT smoother — it operates on ByteTrack's output bbox
centers, not on raw detections. Its job is to remove per-frame jitter
(±5–15 px from YOLO NMS) before the error signals reach motion_vision.

It does NOT replace ByteTrack's internal Kalman (which works in image
space for IoU association). The two Kalmans serve different purposes.

Predicted frames
----------------
When `update()` is called with predicted=True (ByteTrack held the track
but the detector fired nothing), the Kalman propagates forward using
`predict()` only (no measurement correction). The returned position is
the Kalman's best guess. `score` stays 0.0 so the control loop can
tell it apart from a corrected frame.

After `max_predict_frames` consecutive predicted frames, the filter is
dropped — the track is considered gone.

Install: pip install filterpy
"""

from __future__ import annotations

from .confidence import ConfidenceModel
from typing import Dict, Tuple

import numpy as np


# Reference frame interval for the process-noise scale. This is the MEASURED
# p50 detection interval on this stack (32.5 ms; p95 48.4 ms, gaps to 2.4 s), not
# a nominal camera rate -- the filter is driven by detections, not by frames.
_DT_REF_S = 0.0325


def _q_for_dt(dt: float, process_noise: float) -> "np.ndarray":
    """Discrete process noise for a constant-velocity model at interval `dt`.

    The standard white-noise-acceleration (Wiener) discretisation. Per axis the
    2x2 block is

        [[dt^4/4, dt^3/2],
         [dt^3/2, dt^2  ]] * sigma_a^2

    with the state ordered [x, y, vx, vy], so each axis's block is spread across
    (0,2) and (1,3) rather than being contiguous.

    WHY THE REFERENCE SCALING, which is the part worth reading. `Q` used to be
    `np.eye(4) * process_noise` -- constant, therefore correct at exactly one dt.
    Simply substituting the WNA form fixes the dt-dependence but also moves the
    operating point by ~10^6 at a typical interval, because `process_noise` was
    tuned (tracker.yaml: 0.05) against that identity matrix and means something
    different here.

    So sigma_a^2 is anchored: at `_DT_REF_S` the VELOCITY term reproduces the old
    `process_noise` exactly, and everything else follows the correct dt law. That
    makes this strictly a scaling correction rather than a silent re-tune -- the
    filter behaves as it always did at the nominal rate and stops being
    over-confident away from it, which is the whole defect.

    `process_noise` therefore keeps its tuned VALUE and its role, but its units
    are now an acceleration variance. Re-tuning against pool video is a genuine
    follow-up; it is not a prerequisite for this fix being an improvement.
    """
    dt = max(float(dt), 1e-6)          # a zero interval would zero all of Q
    sigma_a2 = float(process_noise) / (_DT_REF_S * _DT_REF_S)
    dt2, dt3, dt4 = dt * dt, dt * dt * dt, dt * dt * dt * dt
    pp, pv, vv = dt4 / 4.0, dt3 / 2.0, dt2
    return np.array([
        [pp, 0,  pv, 0 ],
        [0,  pp, 0,  pv],
        [pv, 0,  vv, 0 ],
        [0,  pv, 0,  vv],
    ], dtype=float) * sigma_a2


class PerTrackKalman:
    """One constant-velocity Kalman filter for one track."""

    def __init__(self, cx: float, cy: float, dt: float,
                 process_noise: float, measurement_noise: float):
        try:
            from filterpy.kalman import KalmanFilter
        except ImportError as exc:
            raise ImportError(
                "filterpy is required for Kalman smoothing. "
                "Install it with: pip install filterpy"
            ) from exc

        self._kf = KalmanFilter(dim_x=4, dim_z=2)

        # State transition: constant velocity
        self._kf.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # Measurement function: observe cx, cy only
        self._kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        self._kf.R  = np.eye(2) * measurement_noise
        self._kf.Q  = _q_for_dt(dt, process_noise)
        self._kf.P  = np.eye(4) * 10.0  # initial uncertainty

        self._kf.x = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)

        self._process_noise     = process_noise
        self._measurement_noise = measurement_noise
        self._last_dt           = dt
        self.predict_streak     = 0

    def step(self, cx: float, cy: float, dt: float,
             predicted: bool, r_scale: float = 1.0) -> Tuple[float, float]:
        """Advance one frame. Returns smoothed (cx_hat, cy_hat).

        `r_scale` multiplies the measurement noise for THIS update only (NSA):
        <1 trusts the detection more than the static R, >1 trusts the filter's
        own prediction instead. Restored afterwards so one doubtful frame does
        not permanently change how the filter weighs every later one.
        """
        self._update_F(dt)
        self._kf.predict()
        if not predicted:
            if r_scale != 1.0:
                self._kf.R = np.eye(2) * (self._measurement_noise * r_scale)
            self._kf.update(np.array([[cx], [cy]], dtype=float))
            if r_scale != 1.0:
                self._kf.R = np.eye(2) * self._measurement_noise
            self.predict_streak = 0
        else:
            self.predict_streak += 1
        cx_hat = float(self._kf.x[0])
        cy_hat = float(self._kf.x[1])
        return cx_hat, cy_hat

    def _update_F(self, dt: float) -> None:
        """Re-discretise the model for this frame interval.

        F AND Q. F alone was updated here; Q was built once as
        `np.eye(4) * process_noise` and never moved again -- which makes it
        correct at exactly one dt and wrong at every other. The author clearly
        knew dt was variable, since that is what this method exists for; Q was
        simply not carried through the same reasoning.

        It matters because the error is not small and runs the wrong way.
        Measured detection intervals on this stack are p50 32.5 ms / p95
        48.4 ms, with gaps to 2.4 s. The position term of the true Q scales as
        dt^4, so p50 -> p95 alone is (48.4/32.5)^4 = 4.9x, and a 2.4 s gap is
        ~10^5. A fixed Q therefore leaves the filter OVER-CONFIDENT exactly when
        detections are missing -- the case the smoother exists to handle -- so a
        coasted box is trusted far more than it has earned.
        """
        if abs(dt - self._last_dt) > 1e-4:
            self._kf.F[0, 2] = dt
            self._kf.F[1, 3] = dt
            self._kf.Q = _q_for_dt(dt, self._process_noise)
            self._last_dt = dt


class TrackKalmanSmoother:
    """Manages one PerTrackKalman per active track_id.

    Parameters
    ----------
    process_noise : float
        Q diagonal — how much we trust the constant-velocity model.
        Lower = smoother, more lag. Default 0.1.
    measurement_noise : float
        R diagonal — how much we trust the detector output.
        Lower = tighter tracking, more jitter. Default 1.0.
    max_predict_frames : int
        After this many consecutive predicted frames the filter is
        dropped (track has been gone too long). Default 5.
    default_dt : float
        Fallback dt for the first frame (seconds). Default 1/20.
    """

    def __init__(self, *, adaptive_noise: bool = False,
                 process_noise: float = 0.1,
                 measurement_noise: float = 1.0,
                 max_predict_frames: int = 5,
                 default_dt: float = 1.0 / 20.0):
        self._process_noise      = process_noise
        self._measurement_noise  = measurement_noise
        self._max_predict_frames = max_predict_frames
        self._default_dt         = default_dt

        self._filters:    Dict[int, PerTrackKalman] = {}
        self._last_t:     Dict[int, float]           = {}
        # One model per smoother (i.e. per camera): the score distribution is a
        # property of this detector on this water, shared across its tracks.
        self._conf_model = ConfidenceModel() if adaptive_noise else None

    def smooth(self, track_id: int, cx: float, cy: float,
               frame_t: float, predicted: bool,
               conf: float = float('nan')) -> Tuple[float, float]:
        """Return Kalman-smoothed (cx_hat, cy_hat) for this track.

        Creates a new filter on first sight of a track_id.
        Returns raw (cx, cy) for the very first frame (no velocity yet).
        """
        if track_id not in self._filters:
            dt = self._default_dt
            self._filters[track_id]  = PerTrackKalman(
                cx, cy, dt,
                self._process_noise, self._measurement_noise)
            self._last_t[track_id] = frame_t
            return cx, cy

        last_t = self._last_t[track_id]
        dt     = max(frame_t - last_t, 1e-4)
        self._last_t[track_id] = frame_t

        flt = self._filters[track_id]
        # NSA: scale this update's measurement noise by how much the detector
        # trusts its own box, normalised against what THIS detector produces.
        # Off (1.0) when no confidence is supplied or the model is disabled, so
        # an existing caller keeps the previous behaviour exactly.
        r_scale = 1.0
        if self._conf_model is not None and conf == conf:
            self._conf_model.observe(conf)
            r_scale = self._conf_model.nsa_factor(conf)
        cx_hat, cy_hat = flt.step(cx, cy, dt, predicted, r_scale)
        return cx_hat, cy_hat

    def is_expired(self, track_id: int) -> bool:
        """True when the track has been predicting too long to trust."""
        flt = self._filters.get(track_id)
        if flt is None:
            return False
        return flt.predict_streak >= self._max_predict_frames

    def drop(self, track_id: int) -> None:
        self._filters.pop(track_id, None)
        self._last_t.pop(track_id, None)

    def prune(self, active_ids: set) -> None:
        """Remove filters for tracks that are no longer active."""
        stale = [tid for tid in self._filters if tid not in active_ids]
        for tid in stale:
            self.drop(tid)

    def reset(self) -> None:
        self._filters.clear()
        self._last_t.clear()
