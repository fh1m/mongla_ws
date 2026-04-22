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

from typing import Dict, Tuple

import numpy as np


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
        self._kf.Q  = np.eye(4) * process_noise
        self._kf.P  = np.eye(4) * 10.0  # initial uncertainty

        self._kf.x = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)

        self._process_noise     = process_noise
        self._measurement_noise = measurement_noise
        self._last_dt           = dt
        self.predict_streak     = 0

    def step(self, cx: float, cy: float, dt: float,
             predicted: bool) -> Tuple[float, float]:
        """Advance one frame. Returns smoothed (cx_hat, cy_hat)."""
        self._update_F(dt)
        self._kf.predict()
        if not predicted:
            self._kf.update(np.array([[cx], [cy]], dtype=float))
            self.predict_streak = 0
        else:
            self.predict_streak += 1
        cx_hat = float(self._kf.x[0])
        cy_hat = float(self._kf.x[1])
        return cx_hat, cy_hat

    def _update_F(self, dt: float) -> None:
        if abs(dt - self._last_dt) > 1e-4:
            self._kf.F[0, 2] = dt
            self._kf.F[1, 3] = dt
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

    def __init__(self, *,
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

    def smooth(self, track_id: int, cx: float, cy: float,
               frame_t: float, predicted: bool) -> Tuple[float, float]:
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
        cx_hat, cy_hat = flt.step(cx, cy, dt, predicted)
        return cx_hat, cy_hat

    def is_expired(self, track_id: int) -> bool:
        """True when the track has been predicting too long to trust."""
        flt = self._filters.get(track_id)
        if flt is None:
            return True
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
