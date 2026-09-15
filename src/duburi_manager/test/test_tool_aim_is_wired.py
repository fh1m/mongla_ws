"""The tool correction must REACH the loop, and warn when it cannot be applied.

Twice this session a knob was declared, documented, mapped and never passed --
`vision.lock_s` (the whole lock ladder, unreachable) and the uplink timer
(created at startup only). This file exists so the tool aim is not the third.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

_CTL = Path(__file__).resolve().parents[2] / 'duburi_control' / 'duburi_control'
_VV = (_CTL / 'vision_verbs.py').read_text()
_MV = (_CTL / 'motion_vision.py').read_text()
_VS = (Path(__file__).resolve().parents[1] / 'duburi_manager'
       / 'vision_state.py').read_text()


def test_the_verb_passes_the_tool_offset_into_the_loop():
    assert 'tool_offset_fn=' in _VV, 'align_loop is called without the tool aim'
    assert 'vstate.tool_offset_px(tool)' in _VV


def test_the_loop_applies_it_to_EVERY_centring_axis():
    """lat, yaw and depth all centre the target. Correcting one and not the
    others aims the tool horizontally and the camera vertically."""
    assert _MV.count('+ tool_du') == 2, 'lat and yaw must both be corrected'
    assert _MV.count('+ tool_dv') == 2, 'both depth branches must be corrected'


def test_it_is_resolved_PER_TICK_because_the_range_moves():
    """A constant offset computed once would be right at one range only -- and
    the correction is largest exactly where a mission spends its final second."""
    i = _MV.index('tool_du = tool_dv = 0.0')
    j = _MV.index("ctrl = ex_now - (offsets.get('lat', 0.0) + tool_du)")
    assert i < j, 'the offset is resolved after it is used'
    # inside the loop body: same indent as the axis blocks that follow it
    line = _MV[_MV.rindex('\n', 0, i) + 1:i]
    assert len(line) - len(line.lstrip()) == 12, 'not inside the per-tick loop'


def test_a_raising_resolver_does_not_take_the_hull_with_it():
    i = _MV.index('_to = tool_offset_fn()')
    block = _MV[i - 200:i + 400]
    assert 'except Exception' in block and '_to = None' in block


def test_absent_correction_means_previous_behaviour_not_a_refusal():
    """Refusing would break every align without a pose, which is most of them.
    The fallback must be the historical behaviour: aim the camera."""
    i = _MV.index('tool_du = tool_dv = 0.0')
    # Normalised: a comment wraps, so 'the previous\n # behaviour' is one
    # phrase in the file and two lines on disk.
    why = ' '.join(_MV[max(0, i - 900):i].replace('#', ' ').split())
    assert 'previous behaviour' in why and 'aiming the camera' in why


def test_the_resolver_lives_where_the_dependency_allows():
    """`duburi_control` must not import `duburi_vision`. The geometry table is
    in vision, so the manager's VisionState resolves it -- the same rule that
    keeps the refractive index out of bearing.py."""
    assert 'from duburi_vision.tool_geometry import pixel_offset' in _VS
    assert 'duburi_vision' not in _MV, 'motion_vision imports duburi_vision'
    pkg = (Path(__file__).resolve().parents[2] / 'duburi_control'
           / 'package.xml').read_text()
    assert 'duburi_vision' not in pkg


def test_an_unmeasured_tool_WARNS_before_the_shot():
    """It reads as zero -- byte-identical to aiming the camera, and therefore
    silent. But it is exactly the case that misses."""
    assert '_warn_tool_geometry' in _VV
    i = _VV.index('def _warn_tool_geometry')
    body = _VV[i:i + 2200]
    assert 'is_measured' in body and 'UNMEASURED' in body
    assert 'not in known_tools()' in body, 'an unknown tool must be an ERROR'
    assert 'no live' in body, 'measured-but-no-pose must also warn'


def test_the_warning_fires_BEFORE_align_loop_runs():
    """After the fact is after the shot."""
    assert _VV.index('self._warn_tool_geometry(') < _VV.index('outcome = align_loop(')


def test_the_warning_is_once_per_tool_not_per_call():
    i = _VV.index('def _warn_tool_geometry')
    body = _VV[i:i + 2200]
    assert '_tool_warned' in body and 'add(tool)' in body


# --------------------------------------------------------------------------
#  Behavioural. The live path cannot be reached on this hull: `vision_align`
#  refuses while disarmed (correctly), and with no thrusters fitted the board
#  refuses to arm -- so the warning is provably unreachable on the bench.
#  Verified live 2026-09-10: "vision_align: AUV is disarmed -- call arm() first".
#  Driving the real method against a fake is what closes that gap.
# --------------------------------------------------------------------------
import sys as _sys
import types as _types

_sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'duburi_control'))
from duburi_control.vision_verbs import VisionVerbs      # noqa: E402


class _Log:
    def __init__(self): self.warns, self.errors = [], []
    def warning(self, m): self.warns.append(m)
    def warn(self, m): self.warns.append(m)
    def error(self, m): self.errors.append(m)
    def info(self, m): pass


class _Self:
    _tool_warned = None
    def __init__(self): self.log = _Log()


class _VState:
    def __init__(self, px=None): self._px = px
    def tool_offset_px(self, tool): return self._px


def _warn(fake, tool, vstate=None):
    VisionVerbs._warn_tool_geometry(fake, tool, vstate or _VState())
    return fake.log


def test_no_tool_named_is_silent():
    """Aiming the camera is the historical default and a legitimate choice."""
    log = _warn(_Self(), '')
    assert not log.warns and not log.errors


def test_an_unmeasured_tool_warns_and_says_the_miss_does_not_shrink():
    log = _warn(_Self(), 'torpedo')
    assert log.warns, 'an unmeasured tool did not warn'
    m = log.warns[0]
    assert 'UNMEASURED' in m and 'EVERY RANGE' in m.upper()


def test_an_unknown_tool_is_an_ERROR_and_lists_what_is_known():
    log = _warn(_Self(), 'no_such_tool')
    assert log.errors, 'an unknown tool did not error'
    assert 'tool_geometry.yaml' in log.errors[0]
    assert 'torpedo' in log.errors[0], 'the error should name the known tools'


def test_a_MEASURED_tool_with_no_pose_still_warns():
    """The correction is range-dependent, so a measured offset with no range
    cannot be applied -- and that is exactly a shot that will miss."""
    fake = _Self()
    log = _warn(fake, 'camera_forward', _VState(px=None))
    assert any('no live' in w for w in log.warns), log.warns


def test_a_measured_tool_WITH_a_pose_is_silent():
    log = _warn(_Self(), 'camera_forward', _VState(px=(0.0, 0.0)))
    assert not log.warns and not log.errors


def test_it_warns_once_per_tool_not_once_per_call():
    fake = _Self()
    for _ in range(5):
        _warn(fake, 'torpedo')
    assert len(fake.log.warns) == 1, f'warned {len(fake.log.warns)} times'
