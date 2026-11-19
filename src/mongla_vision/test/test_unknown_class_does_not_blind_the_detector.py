"""An unknown class name must not filter EVERY detection away, on any backend.

Mapping requested names to class ids, an unknown name contributes no id; if
EVERY name is unknown the filter is an empty set, which drops everything. The
detector then returns [] on every frame, the graph publishes at full rate, and
one WARN is the only sign: the silent-[] failure `CLAUDE.md` section 4 has a
rule against, reached from one typo'd class name. The Hailo backend refused to
narrow in that case; the PyTorch backend, at construction and live, narrowed
to nothing. Both now call `detector.allow_ids_for`.

Refusing to narrow is the safe direction -- the caller gets everything the
model found and can filter downstream, rather than getting nothing and
believing the water is empty.
"""
import pytest

from mongla_vision.detection.detector import allow_ids_for
from mongla_vision.detection.hailo import HailoDetector
from mongla_vision.detection.yolo import YoloDetector

NAMES = {0: 'gate', 1: 'buoy'}


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(m)

    warning = error = warn


def test_the_rule():
    assert allow_ids_for(NAMES, None) is None
    assert allow_ids_for(NAMES, ['', ' ']) is None
    log = _Log()
    assert allow_ids_for(NAMES, ['Gate', 'nope'], log=log) == {0}
    assert any("['nope']" in m for m in log.lines)
    log = _Log()
    assert allow_ids_for(NAMES, ['gat'], log=log, tag='[X]') is None
    assert any('NONE of' in m for m in log.lines)


@pytest.mark.parametrize('cls', [HailoDetector, YoloDetector])
def test_every_backend_widens_on_a_typo_and_narrows_on_a_name(cls):
    d = cls.__new__(cls)
    d._names, d._log = NAMES, None
    d.update_allowlist(['gat'])
    assert d._allow_ids is None
    d.update_allowlist([''])
    assert d._allow_ids is None
    d.update_allowlist(['buoy'])
    assert d._allow_ids == {1}
