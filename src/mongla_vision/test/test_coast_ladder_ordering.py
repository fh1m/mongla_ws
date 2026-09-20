"""The coast ladder must stay ORDERED, in seconds, at the shipped defaults.

The four rungs, innermost first:

    _freshness 0.4 s  <  vision.coast_s 0.8 s  <  lost_grace_s 1.0 s
                                                <  tracker max_predict

Only the last is expressed in FRAMES: `ceil(max_predict_s * frame_rate)`.
So its real duration is `max_predict_s * frame_rate / actual_detection_rate`,
and a `frame_rate` set below the rate detections actually arrive at shortens
it until it falls under an inner rung. The coast then truncates and a target
is dropped mid-lock, while the tracker keeps publishing and looks healthy.

This has now happened TWICE, both times because perception got faster and a
hand-set rate did not follow:

  * once before, recorded in `tracker_node`'s own comment, "made live by
    perception getting 5x faster";
  * again on 2026-09-09, when pi_forward's fps went 15 -> 60. Measured with
    the full stack up: downward detections 32 Hz against `dwn_frame_rate`
    15.0, giving 1.5*15/32 = 0.70 s of coast under a 0.8 s vision.coast_s.

A static test cannot measure the live rate -- `frame_rate_warn_ratio` does
that at runtime, and it is what surfaced this. What this file pins is the
arithmetic: at the rates we HAVE measured, the ordering must hold.
"""
import math
import pathlib
import re

_PKG = pathlib.Path(__file__).resolve().parents[1]
_PI_LAUNCH = _PKG / 'launch' / 'vision_pi.launch.py'
_TRACKER = _PKG / 'mongla_vision' / 'tracker_node.py'
_TUNABLES = (_PKG.parents[0] / 'mongla_manager' / 'mongla_manager'
             / 'vision_tunables.py')

# HIGHEST detection rate measured on the vehicle, full vision_pi stack up,
# 2026-09-09. The maximum is the load-bearing figure: the outer rung lasts
# `frames / actual_rate` seconds, so a FASTER detector makes it SHORTER. Both
# arms of the lock A/B were sampled and the larger kept:
#
#     forward   30.23 Hz (lock off)   23.14 Hz (lock on)
#     downward  42.71 Hz (lock off)   29.82 Hz (lock on)
#
# ⛔ These were briefly seeded with the values the launch was set to, which
# made the check circular -- it compared the config against itself and passed
# for that reason alone. They are measurements now.
_MEASURED_HZ = {'fwd': 30.23, 'dwn': 42.71}


def _launch_default(name):
    m = re.search(rf"DeclareLaunchArgument\('{name}',\s*default_value='([^']+)'",
                  _PI_LAUNCH.read_text())
    assert m, f'{name} is gone from vision_pi.launch.py'
    return float(m.group(1))


def _node_default(name):
    m = re.search(rf"declare_parameter\('{name}',\s*([0-9.]+)\)",
                  _TRACKER.read_text())
    assert m, f'{name} is gone from tracker_node'
    return float(m.group(1))


def _coast_s():
    m = re.search(r"'vision\.coast_s':\s*([0-9.]+)", _TUNABLES.read_text())
    assert m, 'vision.coast_s is gone'
    return float(m.group(1))


def test_the_outermost_rung_outlasts_the_control_coast_at_measured_rates():
    max_pred_s = _node_default('max_predict_s')
    coast_s = _coast_s()
    for side, actual in _MEASURED_HZ.items():
        cfg = _launch_default(f'{side}_frame_rate')
        frames = max(1, math.ceil(max_pred_s * cfg))
        real_s = frames / actual
        assert real_s >= coast_s, (
            f'{side}: frame_rate={cfg} gives {frames} frames of coast, which '
            f'at the measured {actual} Hz is {real_s:.2f} s -- SHORTER than '
            f'vision.coast_s={coast_s}. The outermost rung has fallen inside '
            f'an inner one, so the coast truncates and a target is dropped '
            f'mid-lock with nothing logged.')


def test_the_ordering_holds_with_MARGIN_not_just_barely():
    """⛔ RETRACTED AND REPLACED: this used to assert
    `frame_rate >= measured_rate`, which is stronger than the real invariant
    and FALSE on true data -- downward runs at 42.71 Hz against a frame_rate
    of 32 and the ladder is still correctly ordered (1.12 s of coast against
    0.8 s needed). It only ever passed because its `_MEASURED_HZ` had been
    seeded from the launch defaults it was checking.

    What actually matters is headroom on the ordering, so that the next
    speed-up does not walk the outer rung inside an inner one before anyone
    notices. 1.25x of vision.coast_s, which both cameras clear today."""
    max_pred_s = _node_default('max_predict_s')
    coast_s = _coast_s()
    for side, actual in _MEASURED_HZ.items():
        cfg = _launch_default(f'{side}_frame_rate')
        real_s = max(1, math.ceil(max_pred_s * cfg)) / actual
        assert real_s >= coast_s * 1.25, (
            f'{side}: {real_s:.2f} s of coast against vision.coast_s='
            f'{coast_s} leaves under 25 % headroom. The next perception '
            f'speed-up takes the outermost rung inside an inner one, and the '
            f'symptom is a target dropped mid-lock with nothing logged.')


def test_max_predict_s_still_exceeds_the_control_coast():
    """The invariant the frame conversion is meant to preserve. If this ever
    fails, no frame_rate can save the ordering."""
    assert _node_default('max_predict_s') >= _coast_s(), (
        'max_predict_s is below vision.coast_s, so the outermost rung is '
        'inside an inner one by construction, at every frame rate.')
