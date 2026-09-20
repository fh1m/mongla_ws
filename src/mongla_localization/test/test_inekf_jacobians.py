"""Every update's H must be the derivative of its prediction THROUGH `_inject`.

The filter's error is defined by how a correction is applied (R <- dR R,
v <- dR v + dv, p <- dR p + dp). An H written from a different convention still
runs, still converges on easy data, and quietly mis-weights attitude against
velocity and position. So each H is checked against a finite difference of the
predicted measurement taken through `_inject` itself, at a tilted, moving,
off-origin state where every coupling is non-zero.
"""
import copy

import numpy as np
import pytest

from mongla_localization import inekf as I


def _state():
    f = I.RIEKF()
    f.X.R = I.so3_exp(np.array([0.35, -0.25, 1.1]))
    f.X.v = np.array([0.4, -0.3, 0.1])
    f.X.p = np.array([8.0, -5.0, 1.2])
    return f


def _captured_H(f, call):
    seen = {}

    def spy(H, y, R):
        seen['H'] = np.array(H)
        return True
    f._apply = spy
    call(f)
    return seen['H']


def _numeric(f, h, eps=1e-6):
    J = np.zeros((len(h(f.X)), I.RIEKF.DIM))
    base = h(f.X)
    for i in range(9):
        g = copy.deepcopy(f)
        dx = np.zeros(I.RIEKF.DIM)
        dx[i] = eps
        g._inject(dx)
        J[:, i] = (h(g.X) - base) / eps
    return J


CASES = {
    'body_velocity': (lambda f: f.update_body_velocity(f.X.R.T @ f.X.v),
                      lambda X: X.R.T @ X.v),
    'body_velocity_xy': (lambda f: f.update_body_velocity_xy(0.0, 0.0, 1e-3, 1e-3),
                         lambda X: (X.R.T @ X.v)[:2]),
    'depth': (lambda f: f.update_depth(-f.X.p[2]),
              lambda X: np.array([X.p[2]])),
    'position': (lambda f: f.update_position(f.X.p[:2]),
                 lambda X: X.p[:2]),
    'yaw': (lambda f: f.update_yaw(f.X.yaw_deg()),
            lambda X: np.array([np.arctan2(X.R[1, 0], X.R[0, 0])])),
}


@pytest.mark.parametrize('name', sorted(CASES))
def test_H_is_the_derivative_through_inject(name):
    call, h = CASES[name]
    f = _state()
    J = _numeric(f, h)
    H = _captured_H(f, call)
    np.testing.assert_allclose(H[:, :9], J[:, :9], atol=1e-4, err_msg=name)
