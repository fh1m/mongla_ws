"""Anchoring must not change what `turn()` means for a mission that never anchors.

Every mission in the tree is written against boot-relative headings. Making
`turn()` silently mean world-absolute the moment a landmark is seen would
change the course of every one of them, which is a far worse failure than not
having an absolute reference at all. So the conversion is explicit, and
unanchored it is the identity.
"""
from unittest.mock import MagicMock

import pytest

from mongla_planner.mongla_dsl import MonglaMission
from mongla_localization.heading_anchor import Anchor
from mongla_localization.pose_cluster import Fused


def _fake(head=30.0, fused=None, offset=None):
    m = MagicMock()
    m.camera = 'forward'
    m.head.return_value = head
    m._heading_offset = offset
    m._wait_fused_pose.return_value = fused
    return m


def test_a_good_fuse_sets_the_offset():
    fake = _fake(head=30.0, fused=Fused(True, yaw_deg=0.0, support=10,
                                        spread_deg=1.0, rule='egomotion'))
    got = MonglaMission.anchor_heading(fake, bearing_deg=0.0)
    assert got.ok
    assert got.absolute_deg == pytest.approx(180.0)
    assert fake._heading_offset == pytest.approx(150.0)


def test_no_fused_pose_leaves_the_heading_unanchored():
    fake = _fake(fused=None)
    got = MonglaMission.anchor_heading(fake, bearing_deg=0.0)
    assert not got.ok and 'no fused pose' in got.reason
    assert fake._heading_offset is None, 'a failed anchor must not write one'


def test_a_refused_fuse_leaves_the_heading_unanchored():
    fake = _fake(fused=Fused(True, yaw_deg=0.0, support=2, spread_deg=1.0))
    got = MonglaMission.anchor_heading(fake, bearing_deg=0.0)
    assert not got.ok
    assert fake._heading_offset is None


def test_unanchored_absolute_heading_is_the_relative_one():
    assert MonglaMission.absolute_heading(_fake(head=42.0)) == pytest.approx(42.0)


def test_anchored_absolute_heading_applies_the_offset():
    assert MonglaMission.absolute_heading(
        _fake(head=30.0, offset=150.0)) == pytest.approx(180.0)


def test_absolute_to_relative_is_the_identity_when_unanchored():
    # THE LOAD-BEARING ONE. A mission written in world headings still runs on a
    # hull that never saw its landmark -- just without the correction.
    assert MonglaMission.absolute_to_relative(_fake(), 90.0) == pytest.approx(90.0)


def test_absolute_to_relative_undoes_the_offset():
    fake = _fake(offset=150.0)
    assert MonglaMission.absolute_to_relative(fake, 180.0) == pytest.approx(30.0)


def test_a_round_trip_closes():
    fake = _fake(head=30.0, offset=150.0)
    world = MonglaMission.absolute_heading(fake)
    assert MonglaMission.absolute_to_relative(fake, world) == pytest.approx(30.0)
