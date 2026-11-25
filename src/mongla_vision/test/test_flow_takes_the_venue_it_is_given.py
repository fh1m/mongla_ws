"""Flow takes every venue value bringup says it applied, on either vision stack,
and the floor's own tiles are a height source (audit vl_1 #3, vl_12 F12-1)."""
import math
import pathlib
from types import SimpleNamespace

from mongla_vision.flow import flow_setup

SRC = pathlib.Path(__file__).resolve().parents[2]


def test_the_generic_stack_forwards_the_whole_venue():
    vis = (SRC / 'mongla_vision/launch/vision.launch.py').read_text()
    up = (SRC / 'mongla_manager/launch/bringup.launch.py').read_text()
    generic = up[up.index('vision_launch = IncludeLaunchDescription('):]
    for k in ('pool_depth_min_m', 'pool_depth_max_m', 'tile_m', 'lane_lines'):
        assert f"DeclareLaunchArgument('{k}'" in vis, k
        assert f"'{k}': " in vis.split("executable='flow_node'")[-1] or f"'{k}':" in vis, k
        assert f"'{k}'" in generic[:generic.index('launch_arguments') + 3000], k


def test_a_tiled_floor_alone_gives_the_scale():
    cls = [v for v in vars(flow_setup).values()
           if isinstance(v, type) and hasattr(v, '_scale_ready')][0]
    me = SimpleNamespace(_pool_depth=math.nan, _floor_pts=[], _tile_height=1.4,
                         _medium='water', _f_px=741.0, _cal_problem='', _allow_uncal=False)
    assert cls._scale_ready(me) == (True, '')
    me._tile_height = None
    assert cls._scale_ready(me)[0] is False
