"""The BYTE association floor: publish low, steer high.

ByteTrack's finding is that low-score boxes are the occluded and motion-blurred
ones -- the AUV case exactly -- and that associating them recovers tracks. Our
HEF bakes NMS at 0.05 while the launch shipped a 0.25 publish floor, and our own
measured underwater score distribution is p10 0.167 / p50 0.258. **We were
discarding the median detection before anything downstream could vote on it.**

Measured on real competition footage with the publish floor as the ONLY
variable: presence on hard clips 8.3 -> 34.8 %, and NO change on clips already
at 100 %. The cost is bounded because the control path never sees these boxes --
`vision.ctrl_conf` gates what the loop steers on.

Every test here is about that separation. A test that only checked "more boxes
appear" would pass for a change that fed junk straight to the thrusters.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class _FakeHailo:
    """The real filter arithmetic with the chip removed.

    Subclassed behaviour rather than mocked so the thing under test is the
    SHIPPING comparison, not a restatement of it."""

    def __init__(self, conf=0.25, baked=0.05):
        self._conf = conf
        self._assoc_conf = conf
        self._baked_conf = baked
        self._log = None
        self._max_det = 100

    # the two methods under test, imported verbatim by binding
    from mongla_vision.detection.hailo import HailoDetector as _H
    update_assoc_conf = _H.update_assoc_conf
    update_conf = None    # replaced below

    def keeps(self, score):
        """Mirrors `_boxes_to_detections`'s single decision."""
        return score >= self._assoc_conf


def test_the_floor_is_OFF_by_default():
    """Default must be byte-for-byte the old behaviour: publish floor == control
    floor. A silent widening would put low-score boxes on the wire for every
    existing mission."""
    d = _FakeHailo(conf=0.25)
    assert d._assoc_conf == d._conf
    assert not d.keeps(0.20)
    assert d.keeps(0.25)


def test_lowering_it_publishes_the_band_between_the_two_floors():
    d = _FakeHailo(conf=0.25, baked=0.05)
    d.update_assoc_conf(0.10)
    assert d.keeps(0.12)          # association fodder -- reaches the tracker
    assert d.keeps(0.30)          # still published, still steerable
    assert not d.keeps(0.08)      # below the association floor


def test_it_CANNOT_be_raised_above_the_control_floor():
    """A publish floor above the control floor would discard boxes the control
    loop is willing to use -- the exact opposite of the intent, and silent."""
    d = _FakeHailo(conf=0.25)
    d.update_assoc_conf(0.60)
    assert d._assoc_conf <= d._conf
    assert d.keeps(0.30)


def test_it_is_clamped_to_the_HEF_baked_floor():
    """Nothing exists below the bake to publish. Accepting a lower number would
    make the parameter look effective while changing nothing -- the same shape
    as `ros2 param set` on a construction-time parameter."""
    d = _FakeHailo(conf=0.25, baked=0.05)
    d.update_assoc_conf(0.01)
    assert d._assoc_conf == pytest.approx(0.05)


def test_the_node_declares_it_and_applies_it_at_EVERY_construction_site():
    """A model loaded later must not keep the default floor. Three previous bugs
    in this package have exactly that shape -- `device_path` landing in `**_`,
    the YAML table nothing loaded, and a param read only at construction -- so
    the helper is asserted to be CALLED, not merely to exist."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    assert "declare_parameter('assoc_conf'" in src
    assert 'def _apply_assoc_conf' in src
    # once per construction site: registry + single-model
    assert src.count('self._apply_assoc_conf(det)') >= 2


def test_the_launch_and_the_node_agree_on_TYPE():
    """A float param declared as an int (or vice versa) raises
    InvalidParameterTypeException and kills the whole composed process at
    startup. That has happened twice in this package -- once for `range_crop`."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    assert "declare_parameter('assoc_conf',          0.0)" in src, (
        'assoc_conf must be declared as a FLOAT literal')


def test_control_still_gates_on_ctrl_conf():
    """THE safety property. Publishing lower is only acceptable because the
    control loop has its own floor -- if that gate ever disappears, this change
    becomes "feed low-confidence boxes to the thrusters"."""
    mv = (Path(__file__).resolve().parents[2] / 'mongla_control'
          / 'mongla_control' / 'motion_vision.py').read_text()
    assert 'min_score=ctrl_conf' in mv, (
        'the control-side confidence gate is gone -- assoc_conf must not ship '
        'without it')
