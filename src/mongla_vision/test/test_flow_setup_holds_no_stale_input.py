"""Three places the flow node held or invented an input (audit 2026-11-24,
vl_12 F12-2, F12-3, F12-4)."""
import math
from types import SimpleNamespace

import numpy as np

from mongla_vision.flow import flow_setup


def _setup_cls():
    return [v for v in vars(flow_setup).values()
            if isinstance(v, type) and hasattr(v, '_on_state')][0]


def test_a_nan_heading_clears_the_held_one(monkeypatch):
    me = SimpleNamespace(_yaw_deg=90.0, _state_stamp_warned=True,
                         _depth_hist=[], get_logger=None)
    monkeypatch.setattr(flow_setup, 'capture_monotonic', lambda h: (1.0, ''))
    msg = SimpleNamespace(header=None, depth_m=np.nan, yaw_deg=np.nan)
    _setup_cls()._on_state(me, msg)
    assert me._yaw_deg is None


def test_a_position_with_no_spread_is_not_given_one():
    me = SimpleNamespace(_odom_xy=(0, 0, 0, 0))
    msg = SimpleNamespace(frame='pool', anchored=True, fixes=1, x_m=1.0, y_m=2.0,
                          sigma_xy_m=math.nan)
    cls = [v for v in vars(flow_setup).values()
           if isinstance(v, type) and hasattr(v, '_on_loc_state')][0]
    cls._on_loc_state(me, msg)
    assert me._odom_xy is None


def test_the_water_focal_is_scaled_to_the_frame_before_it_is_compared():
    src = (flow_setup.__file__)
    text = open(src).read()
    assert 'k = f_water / self._intr.fx' not in text
    assert 'FOCAL_PARAMS_WIDTH) / self._intr.fx' in text
