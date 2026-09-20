"""The uplink must send the bearing IN THE WATER, from the one index source.

`bearing.py` can refract, but a correction the caller never asks for is the
knob-wired-to-nothing defect: `vision.lock_s` was declared, documented, mapped
and never passed, so the ladder was unreachable for the life of the feature.
This pins the uplink call site itself.

⛔ AND the index must come from `mongla_vision.optics`, which exists to be "the
ONE place the water refractive index lives" (B22, where the literal 1.333 was
written out at five more sites and forward/inverse stopped being inverses).
`mongla_control` must NOT depend on `mongla_vision`, so the CALLER supplies it
rather than the library keeping a second copy.
"""
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_manager.auv_manager_node import AUVManagerNode      # noqa: E402

_SRC = (Path(__file__).resolve().parents[1] / 'mongla_manager'
        / 'auv_manager_node.py').read_text()


class _Log:
    def __init__(self): self.lines = []
    def error(self, m): self.lines.append(m)
    def warn(self, m): self.lines.append(m)
    def warning(self, m): self.lines.append(m)


class _Fake:
    def __init__(self, medium):
        self._m = medium
        self._log = _Log()
    def get_parameter(self, name):
        assert name == 'vision_uplink_medium', name
        return types.SimpleNamespace(value=self._m)
    def get_logger(self):
        return self._log


def _n(medium):
    return AUVManagerNode._uplink_n(_Fake(medium))


def test_water_is_the_default_and_uses_the_shared_constant():
    from mongla_vision.optics import N_WATER
    assert _n('water') == pytest.approx(float(N_WATER))
    import re
    m = re.search(r"declare_parameter\('vision_uplink_medium',\s*'(\w+)'\)", _SRC)
    assert m and m.group(1) == 'water', 'the vehicle operates in water'


def test_air_is_the_identity():
    assert _n('air') == 1.0


@pytest.mark.parametrize('bad', ['', 'Water ', 'seawater', 'none', 'AIR2'])
def test_an_unknown_medium_refuses_loudly_and_fails_toward_water(bad):
    """Silently picking a medium is silently picking a 33 % bearing error, and
    it would present as a mis-tuned gain rather than a units bug. Note 'Water '
    with whitespace/case IS accepted -- only genuinely unknown values warn."""
    f = _Fake(bad)
    got = AUVManagerNode._uplink_n(f)
    from mongla_vision.optics import N_WATER
    assert got == pytest.approx(float(N_WATER)), 'must fail toward the real medium'
    if bad.strip().lower() not in ('water', 'air'):
        assert f._log.lines, f'{bad!r} was accepted silently'


def test_case_and_whitespace_are_tolerated():
    assert _n('  WATER ') == _n('water')
    assert _n(' Air') == 1.0


def test_the_uplink_call_site_actually_PASSES_the_index():
    """The half that was missing for `lock_s`. A refraction the caller never
    requests is a correction that does not exist."""
    i = _SRC.index('b = bearing_from_normalised(')
    call = _SRC[i:i + 400]
    assert 'n_medium=self._uplink_n()' in call, (
        'the uplink builds its bearing without n_medium, so every bearing on '
        'the wire is ~33 % too large off-axis')


def test_the_index_is_not_copied_into_mongla_control():
    """One truth, one copy. `bearing.py` must take the index, never define it."""
    b = (Path(__file__).resolve().parents[2] / 'mongla_control'
         / 'mongla_control' / 'bearing.py').read_text()
    assert '1.333' not in b, 'the refractive index was copied into mongla_control'
    assert 'n_medium' in b
    pkg = (Path(__file__).resolve().parents[2] / 'mongla_control'
           / 'package.xml').read_text()
    assert 'mongla_vision' not in pkg, (
        'mongla_control now depends on mongla_vision -- the dependency runs the '
        'other way, which is why the caller supplies the index')


# --------------------------------------------------------------------------
#  Aimable while the mission runs
# --------------------------------------------------------------------------

def test_the_uplink_timer_is_created_UNCONDITIONALLY():
    """It used to be created only if `vision_uplink_camera` was set AT STARTUP.

    MEASURED on the vehicle 2026-09-10: `ros2 param set /mongla_manager
    vision_uplink_camera downward` reported success, the parameter read back
    correctly -- and not one uplink tick ever ran, because the timer did not
    exist and never would. A mission could not turn the uplink on without a
    relaunch, and nothing said so.

    Same shape as `vision.lock_s`: declared, documented, mapped, dead.
    """
    i = _SRC.index('LANDING_TARGET uplink armed')
    setup = _SRC[max(0, i - 1200):i]
    assert 'if self._is_srot:' in setup, 'the srot gate is gone'
    assert "if uplink_cam and self._is_srot:" not in _SRC, (
        'the timer is gated on the camera being set at startup again -- setting '
        'it mid-mission will silently do nothing')
    # the tick must still refuse an empty camera, or an idle timer sends garbage
    j = _SRC.index('def _vision_uplink_tick')
    assert 'if not cam:' in _SRC[j:j + 1400], (
        'the tick no longer early-returns on an empty camera, so an armed but '
        'unaimed uplink would run')


def test_a_change_of_uplink_camera_is_ANNOUNCED():
    """Otherwise the operator sets a parameter, gets a success, and has nothing
    distinguishing 'sending now' from the startup-only bug this replaced."""
    j = _SRC.index('def _vision_uplink_tick')
    body = _SRC[j:j + 1400]
    assert '_uplink_cam_logged' in body and 'uplink camera ->' in body
