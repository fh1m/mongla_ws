"""The calibration panel shows the CURRENT fit, in the medium it was taken in.

`guide_capture.assess` refits as captures arrive. It set `assess_err` on a
failure and never cleared it, kept a suggestion the solver no longer made,
and labelled an in-water FOV as the air one (then refracted it again for the
water figure) -- the solver itself already corrects that with
`fov_for_medium`.
"""
import numpy as np

from mongla_vision.calibration import guide_capture as g
from mongla_vision.calibration import solver


class _FS:
    board_points = staticmethod(solver.board_points)
    fov_from_K = staticmethod(solver.fov_from_K)
    fov_for_medium = staticmethod(solver.fov_for_medium)

    def __init__(self, st):
        self.st = st

    def detect(self, files, cols, rows):
        return [np.zeros((48, 1, 2), np.float32)] * 6, [], (1280, 720)

    def kfold_std(self, *a):
        K = np.array([[700.0, 0, 640.0], [0, 700.0, 360.0], [0, 0, 1]])
        return [(K, np.zeros(5)), (K, np.zeros(5))]

    def max_ere(self, samples, w, h):
        return 0.5, (0.1, 0.1)

    def fit_standard(self, *a):
        return {'K': self.kfold_std()[0][0], 'D': np.zeros(5)}

    def suggest_next_pose(self, *a):
        self.st.stopped = True
        return None, float('nan'), []


def test_a_good_fit_clears_the_old_error_and_suggestion_and_keeps_the_medium(tmp_path):
    for i in range(6):
        (tmp_path / f'cal_{i:03d}.png').write_bytes(b'')
    st = g.State(8, 6, str(tmp_path), g.build_steps())
    st.medium = 'water'
    st.assess_err, st.suggest = 'too few boards', ((0, 0), 25)
    g.assess(st, 8, 6, 0.025, (1280, 720), _FS(st))
    assert st.assess_err is None
    assert st.suggest is None
    fit = solver.fov_from_K(_FS(st).fit_standard()['K'], 1280, 720)
    assert st.hfov_water == fit['hfov_air']          # measured in water
    assert st.hfov_air > st.hfov_water
