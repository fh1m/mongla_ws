"""A missing measurement is never read as a perfect one (audit vl_10 F3, vl_11 G3)."""
import math
from types import SimpleNamespace

from mongla_vision.flow import flow_checks
from mongla_vision.flow.flow_math import floor_depth_at


def test_too_few_survey_points_measure_no_spread():
    for pts in ([(0, 0, 1.0)], [(0, 0, 1.0), (1, 0, 1.5)],
                [(0, 0, 1.0), (1, 0, 1.5), (0, 1, 1.2)]):
        assert math.isnan(floor_depth_at(pts, 0.3, 0.3)[1]), pts


def test_four_points_measure_a_spread():
    pts = [(0, 0, 1.0), (1, 0, 1.5), (0, 1, 1.2), (1, 1, 1.0)]
    assert floor_depth_at(pts, 0.3, 0.3)[1] > 0.0


def test_a_missing_dispersion_earns_no_coherence_credit():
    cls = [v for v in vars(flow_checks).values()
           if isinstance(v, type) and hasattr(v, '_quality')][0]
    v = SimpleNamespace(net_flow_px=2.0, rot_fraction=0.0)
    me = SimpleNamespace(_max_disp=0.5, _rot_max=0.5)
    unknown = cls._quality(me, v, 0, None)
    coherent = cls._quality(me, v, 0, 0.0)
    assert unknown < coherent, (unknown, coherent)
