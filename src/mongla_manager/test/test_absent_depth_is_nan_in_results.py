"""A result's depth is NaN when there is no depth, never 0.0 (#21 item 3).

0.0 is the surface -- a real value -- so a result carrying it says the hull
surfaced when nothing was measured at all.
"""
import math
import types

import pytest

pytest.importorskip('rclpy')


def test_surface_reports_nan_depth_without_an_attitude():
    from mongla_manager.auv_manager_node import AUVManagerNode
    fc = types.SimpleNamespace(stop_motion=lambda: None,
                               set_mode=lambda m: (True, 'ok'),
                               get_attitude=lambda: None)
    log = types.SimpleNamespace(info=lambda *a, **k: None,
                                warning=lambda *a, **k: None,
                                error=lambda *a, **k: None)
    node = types.SimpleNamespace(fc=fc, get_logger=lambda: log)
    out = AUVManagerNode._run_srot_surface(node, {'timeout': 1.0})
    assert math.isnan(out.final_value)
