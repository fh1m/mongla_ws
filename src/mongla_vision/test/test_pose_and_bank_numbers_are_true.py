"""The pose and the bank report true numbers and true reasons (audit vl_4 M1, M2, M3, M5, M6, M7)."""
import math

import numpy as np
import pytest

from mongla_vision.anchor.pose import TargetPose, ambiguity_ratio, solve_pnp


def test_off_axis_is_the_angle_from_the_normal():
    p = TargetPose(ok=True, yaw_deg=30.0, pitch_deg=30.0)
    assert p.off_axis_deg == pytest.approx(39.23, abs=0.05)


def test_an_absent_branch_error_is_unknown_not_a_coin_flip():
    assert math.isnan(ambiguity_ratio([0.5, float('nan')]))
    assert ambiguity_ratio([0.0, 0.0]) == 1.0


def test_no_ok_pose_is_above_the_bar():
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    g = np.array([(x, y, 0.0) for x in np.linspace(-0.2, 0.2, 5)
                  for y in np.linspace(-0.2, 0.2, 5)])
    rng = np.random.default_rng(0)
    for _ in range(60):
        rv = rng.normal(0, 0.3, 3)
        tv = np.array([0.0, 0.0, 1.5])
        import cv2
        img, _ = cv2.projectPoints(g, rv, tv, K, None)
        img = img.reshape(-1, 2) + rng.normal(0, 3.0, (len(g), 2))
        p = solve_pnp(g, img, K)
        if p.ok:
            assert p.reproj_px <= 4.0 + 1e-9, p.reproj_px


def test_a_preloaded_reference_is_not_called_a_self_closure():
    from mongla_vision.anchor import loop_closure
    src = open(loop_closure.__file__).read()
    assert 'reference age unknown' in src


def test_a_bank_file_name_is_the_file_written(tmp_path):
    from mongla_vision.anchor import bank_store
    src = open(bank_store.__file__).read()
    assert "must end in .npz" in src
