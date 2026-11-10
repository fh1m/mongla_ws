"""Three claims that drifted for weeks, each held to saying where it came from.

A docs pass on 2026-11-07 found the same shape of error three times: a number
or a state that was true of something, written where it read as true of
something else.

- The water field of view, 46.7 deg (downward) and 53.6 deg (forward), is
  computed from the air calibration by Snell's law. Eighteen files called it
  measured in water. Nothing has been measured in water.
- The firmware mixes for eight thrusters. That matrix belongs to the old
  Dubomini vehicle; the Mongla hull has five in the CAD and seven when
  finished. Docs said "this hull" about all three.
- Six asks to the firmware team were written, recorded as ready, and never
  sent, for six weeks. A draft in this repo is not a request anyone can read.

Each check reads text, so each can be satisfied by a word. That is the point:
the word is the provenance, and leaving it out is the defect.
"""
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# Records of what was sent or found on a given day. They quote the state at the
# time, and rewriting them would falsify the record.
HISTORICAL = ('.claude/context/upstream/', '.claude/context/sota/',
              '.claude/context/measured-bars.md', '.claude/skills/')


def _tracked(*globs):
    out = subprocess.run(['git', 'ls-files', *globs], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    return [f for f in out.split() if not f.startswith(HISTORICAL)
            and (ROOT / f).is_file()]


def _unqualified(files, claim, qualifier, reach):
    """Lines matching `claim` with no `qualifier` within `reach` lines."""
    found = []
    for f in files:
        lines = (ROOT / f).read_text(errors='replace').splitlines()
        for i, line in enumerate(lines):
            if claim.search(line):
                near = ' '.join(lines[max(0, i - reach):i + reach + 1])
                if not qualifier.search(near):
                    found.append(f'{f}:{i + 1}: {line.strip()[:100]}')
    return found


WATER_FOV = re.compile(r'(?<![\d.])(46\.7|53\.6)\d?\s*(°|deg)')
COMPUTED = re.compile(r'snell|computed|by refraction|derived|predict'
                      r'|not (yet )?(checked|measured)', re.I)


def test_a_water_field_of_view_says_it_is_computed():
    files = [f for f in _tracked('*.md', '*.html', '*.py', '*.yaml', '*.js')
             if f != 'test/test_claims_say_where_they_came_from.py']
    bad = _unqualified(files, WATER_FOV, COMPUTED, reach=3)
    assert not bad, ('a water FOV stated without saying it is Snell from the '
                     'air calibration (first-pool-verification.md §9 is the '
                     'measurement that would change this):\n' + '\n'.join(bad))


EIGHT = re.compile(r'8 ?(×|x) ?T200|eight (vectored )?T200|eight-thruster'
                   r'|8-thruster|vectored[-_ ]?8\b', re.I)
WHOSE = re.compile(r'dubomini|firmware|board|mixer|old(er)? vehicle', re.I)


def test_an_eight_thruster_figure_names_whose_it_is():
    bad = _unqualified(_tracked('*.md', '*.html'), EIGHT, WHOSE, reach=2)
    assert not bad, ('an eight-thruster figure that does not say it is the '
                     "board's mixer or the old Dubomini vehicle; the Mongla "
                     'hull has 5 in the CAD and 7 when finished '
                     '(vehicle-spec.md, "The body"):\n' + '\n'.join(bad))


UPSTREAM = ROOT / '.claude/context/upstream'

# Written and deliberately held back, each with its reason. Empty on purpose:
# J sat here seven weeks waiting for an owner, which is how an ask goes stale.
HELD: dict = {}


def _asks():
    return sorted(p for p in UPSTREAM.glob('pr-*.md'))


def test_there_are_asks_to_check():
    assert len(_asks()) >= 20


@pytest.mark.parametrize('ask', _asks(), ids=lambda p: p.name)
def test_every_written_ask_is_filed_or_held_with_a_reason(ask):
    letter = ask.name.split('-')[1]
    if letter in HELD:
        return
    table = (UPSTREAM / 'README.md').read_text()
    row = re.search(r'^\| \*{0,2}' + letter.upper() + r'\*{0,2} \|[^\n]*'
                    r'https://github\.com/RakibulIslam1/srot-control-board/'
                    r'(pull|issues)/\d+', table, re.M)
    assert row, (f'{ask.name} is written but has no row with a live URL in '
                 'the "filed as" table of upstream/README.md. File it, or add '
                 'it to HELD with the reason it waits.')
    text = ask.read_text()
    assert not re.search(r'NOT SENT|UNSENT', text), (
        f'{ask.name} is filed but still says it was not sent')
