"""The bars in `.claude/context/measured-bars.md` still hold.

A number with no bar cannot be wrong, so it never gets checked -- it just gets
quoted, and eventually shipped. Round 34 produced four retractions in one round
from exactly that, so each shipped constant now has a measurement behind it and
this file is what notices when the two part company.

EVERY CONSTANT IS READ FROM SOURCE, NEVER IMPORTED. In a git worktree
`import mongla_control` resolves to the MAIN workspace's `install/` tree -- a
different commit. That is not hypothetical: it reported `VISION_FRESH_ZERO_S`
as 0.4 against a source saying 0.20, which reads exactly like a drift bug in
the file it guards. Reading the file beside us answers about the branch we are
on, which is the only branch this worktree can ship.

These assert the BAR, not the exact value: a constant is free to move inside
the range its measurement supports, and fails when it leaves it. A test pinned
to the exact number fails on every legitimate retune and gets deleted.
"""
import re
import sys
from pathlib import Path

import pytest

# `mongla_vision` is imported from THIS worktree (inserted first); every
# CONSTANT below is still read as text. The distinction matters: the
# install-tree hazard bites on `mongla_control`, which has an installed copy
# from the main workspace, and would have been silently answered from a
# different commit.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

ROOT = Path(__file__).resolve().parents[2]
BARS = ROOT.parent / '.claude' / 'context' / 'measured-bars.md'


def _const(rel, name):
    """A module-level `NAME = <number>` assignment, read as text."""
    src = (ROOT / rel).read_text()
    m = re.search(rf'^{name}\s*=\s*([0-9.]+)', src, re.M)
    assert m, f'{name} not found in {rel}'
    return float(m.group(1))


def _deck(name):
    src = (ROOT / 'mongla_manager' / 'mongla_manager'
           / 'vision_tunables.py').read_text()
    m = re.search(rf"'vision\.{name}':\s*([0-9.]+)", src)
    assert m, f'{name} not found in vision_tunables.py'
    return float(m.group(1))


MV = 'mongla_control/mongla_control/motion_vision.py'


# --------------------------------------------------------------------------- #
#  1. The conf floor  (measured-bars.md section 1)
# --------------------------------------------------------------------------- #
def test_every_profile_conf_is_inside_the_measured_range():
    """0.15 -> 0.10 was measured on 7 labelled held-out pairs: it buys
    +0.0..+3.0 points of recall and costs 0.0..-8.3 of precision, and every F1
    knee sits at 0.25-0.50 except the cross-venue gate at 0.05.

    So the shipped floor belongs BELOW the knee (a miss stalls the control
    loop; a false positive is partly filtered by lock_on / ctrl_conf / the
    distinct-frame gate) but not below what was measured at all."""
    from mongla_vision.detection.profiles import PROFILES
    for name, (settings, _why) in PROFILES.items():
        c = settings['conf']
        assert 0.08 <= c <= 0.15, (
            f'profile {name!r} conf {c} is outside the measured range '
            f'0.08..0.15 -- see measured-bars.md section 1')


# --------------------------------------------------------------------------- #
#  2. The lock ladder  (measured-bars.md section 2)
# --------------------------------------------------------------------------- #
GAP_P50, GAP_P90, GAP_P99 = 0.155, 0.651, 2.418   # 71 real gaps, native rate


def test_coast_covers_the_p90_of_real_detection_gaps():
    """THE bar this round measured. `coast_s` exists to ride out routine
    flicker, so it has to reach the p90 of the flicker actually observed --
    0.651 s over 71 native-rate gaps from five competition clips. At 0.80 it
    covers 91.5 % with 23 % of margin.

    Measured at NATIVE FRAME RATE deliberately: a sampled sweep cannot see a
    gap shorter than its own sample interval, and reported 307 of 307 gaps
    exceeding this rung -- plausible, and pure arithmetic."""
    assert _deck('coast_s') >= GAP_P90


def test_the_ladder_is_still_ordered():
    """Each rung must outlast the one below or the shorter one is unreachable
    -- authority would hit zero after loss was already declared, turning a
    dropout into a lurch instead of a glide."""
    fresh_zero = _const(MV, 'VISION_FRESH_ZERO_S')
    coast, grace = _deck('coast_s'), _deck('lost_grace_s')
    assert fresh_zero < coast < grace, (fresh_zero, coast, grace)


def test_freshness_zero_reaches_the_median_gap():
    """Below the p50 the translational command decays to neutral on gaps that
    happen constantly, which is a hull that stutters rather than glides."""
    assert _const(MV, 'VISION_FRESH_ZERO_S') >= GAP_P50


def test_track_buffer_covers_the_p99():
    """The last rung is what makes a re-acquisition the SAME id rather than a
    new one. p99 is 2.418 s and nothing in the archive exceeded 5.0 s."""
    src = (ROOT / 'mongla_vision' / 'config' / 'tracker.yaml').read_text()
    m = re.search(r'track_buffer\s*:\s*([0-9.]+)', src)
    if not m:
        pytest.skip('track_buffer not in tracker.yaml')
    assert float(m.group(1)) >= GAP_P99


# --------------------------------------------------------------------------- #
#  3. The bars file is real
# --------------------------------------------------------------------------- #
def test_the_bars_file_exists_and_carries_its_measurements():
    """A bars file that loses its evidence column is a list of magic numbers
    with extra steps."""
    assert BARS.exists(), f'{BARS} is missing'
    text = BARS.read_text()
    for anchor in ('0.651', '17', '0.0 %', 'track_buffer', 'slalom'):
        assert anchor in text, f'measured-bars.md no longer mentions {anchor!r}'
