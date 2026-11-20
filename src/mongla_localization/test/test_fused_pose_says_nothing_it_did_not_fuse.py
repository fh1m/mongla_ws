"""The fused pose carries only what the fuse computed; the rest is NaN.

`pose_fuse_node` filled yaw, range and the spread and left every other float
at the message default 0.0. Batighor plots the fused bearing and
reprojection, so the operator saw a bearing of exactly zero on every fused
pose, and a yaw and range of zero while the fuse was undecided.
"""
import math

from mongla_localization.pose_cluster import Fused
from mongla_localization.pose_fuse_node import _FLOATS, fused_message


class _Msg:
    """A TargetPose's fields at their message defaults."""

    def __init__(self):
        for f in _FLOATS:
            setattr(self, f, 0.0)
        self.ok, self.reason, self.n_points = False, '', 0


def test_a_decided_pose_has_no_invented_fields():
    out = fused_message(_Msg(), Fused(decided=True, yaw_deg=12.0, range_m=3.0,
                                      support=7, spread_deg=1.5, rule='support'),
                        header=None, label='gate')
    assert out.ok and out.yaw_deg == 12.0 and out.range_m == 3.0 and out.n_points == 7
    for f in ('bearing_deg', 'reproj_px', 'pitch_deg', 'roll_deg', 'ambiguity',
              'alt_yaw_deg', 'off_axis_deg', 'pitch_spread_deg'):
        assert math.isnan(getattr(out, f)), f


def test_an_undecided_pose_has_no_yaw_or_range():
    out = fused_message(_Msg(), Fused(decided=False, reason='too few'),
                        header=None, label='gate')
    assert not out.ok
    assert math.isnan(out.yaw_deg) and math.isnan(out.range_m)
    assert math.isnan(out.yaw_spread_deg)


def test_a_mirror_pair_still_reports_its_separation():
    out = fused_message(_Msg(), Fused(decided=False, reason='mirror',
                                      candidates=(20.0, -20.0)),
                        header=None, label='gate')
    assert out.yaw_spread_deg == 40.0


def test_every_float_of_the_message_is_listed():
    """A field added to TargetPose must be decided here, not left at 0.0."""
    from mongla_interfaces.msg import TargetPose
    floats = {k for k, v in TargetPose.get_fields_and_field_types().items() if v == 'float'}
    assert floats == set(_FLOATS)
