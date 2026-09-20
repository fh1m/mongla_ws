"""20 s of the real SROT board at rest, through the real filter.

Every other test here builds its IMU from the filter's own convention, so a
wrong convention passes them all -- which is exactly what happened: world z
was UP while the attitude fed in was NED, and the board's accel arrived in
the BNO's sensor axes. On the vehicle that ran to 1160 m in 30 s at rest.
This fixture is the only test that holds the filter to the board's frames.
"""
import json
from pathlib import Path

import numpy as np

from mongla_localization.inekf import RIEKF
from mongla_localization.localization_node import _R_from_quat

DATA = Path(__file__).parent / 'data' / 'board_still_2026-09-15.json'


def _run(map_accel: bool):
    rows = json.loads(DATA.read_text())['rows']
    f = RIEKF()
    prev = None
    rejected = n = 0
    for k, r in enumerate(rows):
        t, q, g, a = r[0], r[1:5], r[5:8], r[8:11]
        R = _R_from_quat(*q)
        # SrotFC's proven sensor->FRD mapping, (x, y, z) -> (y, x, -z)
        acc = (a[1], a[0], -a[2]) if map_accel else tuple(a)
        if prev is None:
            f.X.R, prev = R, t
            continue
        dt, prev = t - prev, t
        f.predict(tuple(g), acc, dt)
        f.update_attitude(R, sigma_deg=0.5)
        if k % 50 == 0:
            n += 1
            rejected += 0 if f.update_zero_velocity(sigma=0.01) else 1
    return np.linalg.norm(f.X.p), np.linalg.norm(f.X.v), rejected, n


def test_the_board_at_rest_stays_at_rest():
    p, v, rejected, n = _run(map_accel=True)
    assert p < 0.1, f'{p:.3f} m of travel on a still bench'
    # The last ZUPT is ~1 s before the end, and between ZUPTs the board's
    # residual (level trim is subtracted from its Euler angles but not from
    # its gravity report, ~1 deg) integrates at ~0.14 m/s^2 -- measured 0.138
    # m/s here. Unmapped, the same data ends at 390 m/s.
    assert v < 0.25
    assert rejected == 0, f'{rejected}/{n} ZUPTs rejected'


def test_and_the_unmapped_accel_is_what_ran_away():
    """The control: without the frame fix the same data diverges. If this
    ever passes bounded, the fixture no longer discriminates."""
    p, _v, rejected, n = _run(map_accel=False)
    assert p > 10.0 and rejected > n // 2
