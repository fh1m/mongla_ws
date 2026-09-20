"""One word instead of four knobs -- and every value in it is measured.

WHY. The vision path has four settings that interact: `conf`, `preprocess`,
`preprocess_clip`, `range_crop`. Each earned its place from a measurement.
Together they are a puzzle to solve with a run clock going, and a WRONG
combination is worse than the default -- CLAHE on clear water cost 64 points
of recall, a fixed crop costs 31 on a close prop.

These tests pin the two things that make a profile safe to trust: that its
values match the measurements they claim, and that it cannot silently do
nothing.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mongla_vision.detection.profiles import (      # noqa: E402
    DEFAULT, PROFILES, resolve,
)


def test_every_profile_sets_every_knob():
    """A profile that leaves one out inherits whatever the launch default
    happens to be, which is exactly the surprise it exists to remove."""
    keys = {'conf', 'preprocess', 'preprocess_clip', 'range_crop'}
    for name, (settings, _why) in PROFILES.items():
        assert set(settings) == keys, (name, set(settings) ^ keys)


def test_an_unknown_profile_RAISES_rather_than_defaulting():
    """A mistyped profile that quietly runs `fast` in murky water is a
    mission lost to a typo. This repo has shipped the silent-fallback failure
    five times; it does not get a sixth."""
    with pytest.raises(ValueError):
        resolve('murkey')
    with pytest.raises(ValueError):
        resolve('turbid')


def test_an_empty_name_is_the_documented_default_not_an_error():
    """Nothing set is a legitimate state -- it means the operator did not ask
    for a profile, and the individual knobs stand."""
    settings, _why = resolve('')
    assert settings == PROFILES[DEFAULT][0]


def test_murky_and_clear_differ_by_the_CROP_not_by_preprocessing():
    """RETRACTED and replaced. This asserted `murky` turns CLAHE on, which
    was the profile system's original headline. It does not reproduce: on
    raw detection rate CLAHE was never positive in 17 measured
    configurations and took the gate from 30.4 % to 1.2 %.

    What still separates the profiles is real and re-measured -- the range
    crop and the confidence floor -- so the assertion moves onto those
    rather than being deleted, or nothing pins the profiles apart."""
    murky, clear, close = (resolve(n)[0] for n in ('murky', 'clear', 'close'))
    assert murky['preprocess'] == clear['preprocess'] == 'off'
    assert murky['range_crop'] is True and close['range_crop'] is False
    assert murky['conf'] == 0.10 and close['conf'] == 0.15


def test_close_work_turns_the_crop_OFF():
    """The crop LOSES 31 points on a target that fills the frame. A docking
    or firing profile that kept it on would trade the end of the mission for
    the start of it."""
    assert resolve('close')[0]['range_crop'] is False
    assert resolve('murky')[0]['range_crop'] is True
    assert resolve('clear')[0]['range_crop'] is True


def test_conf_matches_the_measured_floor():
    """0.10 came from a sweep on real footage: +8.5 points of presence over
    0.15 with jitter FLAT, while 0.05 raised multi-box frames to 35 % and
    jitter by 53 %. The close profile keeps 0.15 because a false positive
    matters more when the prop is right there."""
    for name in ('murky', 'clear'):
        assert resolve(name)[0]['conf'] == 0.10, name
    assert resolve('close')[0]['conf'] == 0.15


def test_fast_is_everything_OFF():
    """The arm to reach for when loop rate is the binding constraint --
    CLAHE costs 79 -> 50 Hz on the Pi."""
    s, _ = resolve('fast')
    assert s['preprocess'] == 'off' and s['range_crop'] is False


def test_every_profile_explains_ITSELF_with_numbers():
    """A profile that bundles a guess is worse than four honest knobs,
    because it hides the guess. Each `why` is printed at startup, so the
    operator can see what was chosen and on what evidence."""
    for name, (_s, why) in PROFILES.items():
        assert len(why) > 60, name
        assert any(ch.isdigit() for ch in why), \
            f'{name}: the rationale cites no measurement'


def test_resolve_returns_a_COPY():
    """A caller mutating the result must not edit the table for everyone
    else in the process -- two detectors share it."""
    a = resolve('murky')[0]
    a['conf'] = 0.99
    assert resolve('murky')[0]['conf'] == 0.10


def test_unset_is_a_SENTINEL_not_a_comparison_to_the_default():
    """THE DEFECT THE UNIT TESTS COULD NOT SEE, found on hardware.

    A launch file always passes every parameter. The first version decided
    "did the operator set this?" by comparing against the declared default,
    so the launch's own `preprocess:='none'` and `range_crop:=False` looked
    like deliberate choices and the profile never applied. Live on the Pi,
    `vision:=murky` LOGGED that it had applied while `range_crop` read False
    and `conf` read 0.15.

    A knob is unset when it holds a value that means nothing on its own.
    """
    from mongla_vision.detector_node import _UNSET, _is_unset

    assert _UNSET['preprocess'] == 'auto'
    assert _UNSET['range_crop'] == -1
    assert _is_unset('preprocess', 'auto') and not _is_unset('preprocess', 'off')
    assert _is_unset('range_crop', -1) and not _is_unset('range_crop', 0)


def test_range_crop_is_an_INT_so_a_profile_can_turn_it_on():
    """A bool has two states and needs three: on, off, and not-said. With a
    bool a profile can never enable it, because False is indistinguishable
    from unset."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    assert "self.declare_parameter('range_crop',          -1)" in src
    launch = (Path(__file__).resolve().parents[1] / 'launch'
              / 'vision_pi.launch.py').read_text()
    assert "DeclareLaunchArgument('range_crop', default_value='-1')" in launch


def test_conf_is_deliberately_NOT_profile_routed():
    """It also feeds the tracker's confidence clamp through the launch, and a
    sentinel there would silently disable the clamp -- which measured as the
    tracker emitting NOTHING on real water. Documented as the exception
    rather than left as an inconsistency."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    from mongla_vision.detector_node import _UNSET
    assert 'conf' not in _UNSET, (
        "conf must not be profile-routed -- it feeds the tracker clamp")
    assert 'DELIBERATELY ABSENT' in src, 'the exception must be explained'


def test_the_profile_resolves_before_anything_reads_a_value():
    """Ordering, which fails silently: a profile that half-applies still
    logs that it applied."""
    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    assert src.index('prof_name = ') < src.index("_p('preprocess'")


def test_the_composed_launcher_declares_the_SAME_TYPES_as_the_node():
    """rclpy raises InvalidParameterTypeException on a mismatch and the whole
    composed process dies at startup -- every node gone, the clue eleven
    frames down a traceback.

    It has now happened twice: `preprocess` ('off' coerced to bool False) and
    `range_crop` (bool default against an int parameter). Both times the unit
    tests passed, because they construct the node directly and never go
    through the launcher's own `_DEFAULTS` table.
    """
    import re
    from mongla_vision import detector_dual_node as DD

    src = (Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'detector_node.py').read_text()
    declared = dict(
        (m.group(1), m.group(2).strip())
        for m in re.finditer(r"declare_parameter\('(\w+)',\s*([^)]+)\)", src))

    for key, val in DD._DEFAULTS.items():
        if key not in declared:
            continue
        node_default = declared[key]
        # bool literals are the trap: True/False in Python, and an int or a
        # string on the other side reads as a different rclpy type.
        node_is_bool = node_default in ('True', 'False')
        assert node_is_bool == isinstance(val, bool), (
            f'{key}: launcher default {val!r} and node default '
            f'{node_default} are different rclpy TYPES -- the composed '
            f'process will die at declare_parameter')


def test_no_profile_enables_preprocessing():
    """RETRACTION GUARD. `murky` shipped `preprocess='clahe'` on a +42 claim
    that does not reproduce: re-measured on raw detection rate across 17
    configurations -- 4 props, 3 venues, a 39x sharpness range -- CLAHE was
    never positive, and on the very footage the claim came from it took the
    gate from 30.4 % to 1.2 % across five independent frame samples.

    `preprocess:=clahe` stays available as an explicit operator choice. What
    must not come back is a profile turning it on for them."""
    from mongla_vision.detection.profiles import PROFILES
    for name, (settings, _why) in PROFILES.items():
        assert settings.get('preprocess') == 'off', (
            f'profile {name!r} enables preprocessing: '
            f'{settings.get("preprocess")!r}')
