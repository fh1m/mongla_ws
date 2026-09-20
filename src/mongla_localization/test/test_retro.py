"""Retrodiction: a late measurement must land where an on-time one would have.

Truth by construction. Three filters see the SAME IMU and attitude stream:
  on_time  -- the measurement is applied at its instant (the reference),
  retro    -- it arrives 60 ms late through `Retrodictor`,
  arrival  -- it arrives 60 ms late and is applied to the current state (the
              pre-retrodiction behaviour).
`retro` must equal `on_time` to numerical precision; `arrival` must not.
"""
import copy

import numpy as np
import pytest

from mongla_localization import inekf as I
from mongla_localization.retro import Retrodictor


DT = 0.02


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _stream(n=100, rate=0.4):
    """IMU + attitude for a hull turning while accelerating forward."""
    out, yaw = [], 0.0
    for k in range(n):
        t = k * DT
        R = _Rz(yaw)
        a_body = np.array([0.3, 0.0, 0.0])
        out.append((t, np.array([0.0, 0.0, rate]), a_body - R.T @ I.GRAVITY, R))
        yaw += rate * DT
    return out


def _step(f, gyro, acc, R):
    f.predict(gyro, acc, DT)
    f.update_attitude(R, sigma_deg=0.5)


def _vel(f):
    f.update_body_velocity_xy(0.9, -0.2, 1e-3, 1e-3)    # deliberately off the state


def test_a_late_measurement_through_the_retrodictor_equals_the_on_time_one():
    stream = _stream()
    t_meas_index = 60                                     # measured at t=1.20 s
    late_by = 3                                           # arrives 60 ms later

    on_time = I.RIEKF()
    for k, (t, g, a, R) in enumerate(stream):
        _step(on_time, g, a, R)
        if k == t_meas_index:
            _vel(on_time)

    retro_f = I.RIEKF()
    rd = Retrodictor(retro_f, horizon_s=1.0)
    arrival = I.RIEKF()
    for k, (t, g, a, R) in enumerate(stream):
        rd.run(t, lambda f, g=g, a=a, R=R: _step(f, g, a, R))
        _step(arrival, g, a, R)
        if k == t_meas_index + late_by:
            t_meas = stream[t_meas_index][0] + 1e-6       # just after that IMU event
            assert rd.run(t_meas, _vel)
            _vel(arrival)

    np.testing.assert_allclose(retro_f.X.p, on_time.X.p, atol=1e-9)
    np.testing.assert_allclose(retro_f.X.v, on_time.X.v, atol=1e-9)
    np.testing.assert_allclose(retro_f.P, on_time.P, atol=1e-9)
    assert retro_f.accepted == on_time.accepted           # counters not double-counted
    assert rd.late == 1 and rd.replayed == late_by
    assert np.abs(arrival.X.v - on_time.X.v).max() > 1e-4  # the defect is real


def test_a_measurement_older_than_the_horizon_is_refused_and_counted():
    f = I.RIEKF()
    rd = Retrodictor(f, horizon_s=0.5)
    for t, g, a, R in _stream(80):
        rd.run(t, lambda f, g=g, a=a, R=R: _step(f, g, a, R))
    before = copy.deepcopy(f.X)
    assert rd.run(0.1, _vel) is False
    assert rd.refused == 1
    np.testing.assert_array_equal(f.X.p, before.p)


def test_in_order_events_never_replay():
    f = I.RIEKF()
    rd = Retrodictor(f, horizon_s=1.0)
    for t, g, a, R in _stream(50):
        rd.run(t, lambda f, g=g, a=a, R=R: _step(f, g, a, R))
    assert rd.replayed == 0 and rd.late == 0


@pytest.mark.parametrize('late_ms', [60, 1000])
def test_replay_cost_is_printed(late_ms, capsys):
    import time
    f = I.RIEKF()
    rd = Retrodictor(f, horizon_s=2.0)
    stream = _stream(150)
    for t, g, a, R in stream:
        rd.run(t, lambda f, g=g, a=a, R=R: _step(f, g, a, R))
    t0 = time.perf_counter()
    assert rd.run(stream[-1][0] - late_ms / 1000.0 + 1e-6, _vel)
    ms = (time.perf_counter() - t0) * 1000.0
    with capsys.disabled():
        print(f'\n[retro] {late_ms} ms late: replayed {rd.replayed} events in {ms:.2f} ms')
