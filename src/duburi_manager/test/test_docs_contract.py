"""The documentation contract: what must be true of the docs, not of the prose.

⛔ WHY THIS EXISTS. Three defects this repo actually shipped, none of which any
test could see:

  1. `.claude/context` accumulated SEVEN dead links (`esp32c3_bno085.md`,
     `anchor-system.md`, five pointing outside the repo). A reader following one
     learns nothing; an agent following one wastes a turn.
  2. Every package went un-documented -- seven directories of Python and no page
     saying what any of them was for.
  3. The entry points described a vehicle we stopped flying. `CLAUDE.md` still
     stated a firmware safety floor of 2 when the code had required 10 since the
     revision that INVERTED YAW, and the published site had a Pixhawk badge and
     zero occurrences of the board we actually fly.

These are read as TEXT, never imported: the docs are the artefact under test.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]          # .../duburi_ws
CTX = ROOT / '.claude' / 'context'
DOCS = ROOT / 'docs'

PACKAGES = sorted(p.name for p in (ROOT / 'src').iterdir()
                  if p.is_dir() and (p / 'package.xml').exists())


def _markdown():
    """Every doc a reader or an agent is pointed at."""
    out = [ROOT / 'README.md', ROOT / 'CLAUDE.md']
    out += sorted(CTX.glob('**/*.md'))
    out += sorted(DOCS.glob('*.md'))
    return [p for p in out if p.is_file()]


# ── 1. every package is documented, from both directions ────────────────────

def test_every_package_has_a_page_and_a_stub():
    assert PACKAGES, 'no packages found -- the sweep would pass vacuously'
    missing = []
    for pkg in PACKAGES:
        if not (CTX / 'packages' / pkg / 'README.md').is_file():
            missing.append(f'.claude/context/packages/{pkg}/README.md')
        if not (ROOT / 'src' / pkg / 'README.md').is_file():
            missing.append(f'src/{pkg}/README.md')
    assert not missing, 'undocumented packages:\n  ' + '\n  '.join(missing)


def test_the_package_index_lists_every_package():
    index = (CTX / 'packages' / 'README.md').read_text()
    absent = [p for p in PACKAGES if p not in index]
    assert not absent, f'packages/README.md does not mention: {absent}'


# ── 2. every relative link resolves ─────────────────────────────────────────

_LINK = re.compile(r'\]\(([^)]+)\)')


def _targets(doc):
    for raw in _LINK.findall(doc.read_text()):
        href = raw.split()[0].strip()                 # drop a title after the url
        if href.startswith(('http://', 'https://', 'mailto:', '#')):
            continue
        yield href.split('#', 1)[0]


def test_every_relative_markdown_link_resolves():
    bad = []
    for doc in _markdown():
        for href in _targets(doc):
            if not href:
                continue
            target = (doc.parent / href).resolve()
            if ROOT not in target.parents and target != ROOT:
                continue          # a sibling repository; not ours to verify
            if target.exists():
                continue
            # a published page links its sibling by the name Jekyll will serve
            if target.suffix == '.html' and target.with_suffix('.md').exists():
                continue
            bad.append(f'{doc.relative_to(ROOT)} -> {href}')
    assert not bad, 'dead links:\n  ' + '\n  '.join(bad)


# ── 3. the entry points describe the vehicle we fly ─────────────────────────

# Tokens that only ever appear when a document is presenting the OLD platform as
# the current one. A historical sentence uses the past tense and does not need
# these exact strings; every one of them was live in these files before the
# 2026-09-19 pass.
RETIRED = ('Jetson Orin Nano', 'ArduSub 4.x', 'Pixhawk 2.4.8', 'BlueOS')

ENTRY_POINTS = ('README.md', 'CLAUDE.md', 'docs/index.html', 'docs/_config.yml')


@pytest.mark.parametrize('name', ENTRY_POINTS)
def test_the_entry_points_do_not_present_the_retired_platform(name):
    text = (ROOT / name).read_text()
    hits = [tok for tok in RETIRED if tok in text]
    assert not hits, (
        f'{name} presents the retired platform ({hits}). The Pixhawk + Jetson '
        f'stack lives on the `pixhawk` branch; the live path is the SROT board '
        f'and the Pi. One page may describe the old path: '
        f'.claude/context/legacy-pixhawk-and-sitl.md')


def test_the_legacy_page_exists_so_the_simulator_is_not_undocumented():
    """The pixhawk backend and ArduSub SITL are still live code. Retiring the
    documentation without this page would leave `sim/` unexplained."""
    page = CTX / 'legacy-pixhawk-and-sitl.md'
    assert page.is_file(), 'the legacy page was deleted; sim/ has no documentation'
    text = page.read_text()
    assert 'SITL' in text and 'pixhawk' in text


# ── 4. the two flagship documents ───────────────────────────────────────────

def test_the_shift_and_the_capability_map_are_published():
    for name in ('the-shift.md', 'capability-map.md'):
        assert (DOCS / name).is_file(), f'docs/{name} is missing'
        assert (CTX / name).is_file(), f'.claude/context/{name} pointer is missing'


def test_the_shift_credits_the_firmware_lead_and_names_all_four_repositories():
    text = (DOCS / 'the-shift.md').read_text()
    assert 'Rakibul Islam' in text, 'the firmware development team lead is not credited'
    for repo in ('srot-control-board', 'srot-ground-station', 'srot-esc-flasher', 'duburi_ws'):
        assert repo in text, f'{repo} is not named'
    for codename in ('Hengla', 'Bondor', 'Mongla'):
        assert codename in text, f'{codename} is not named'


def test_the_capability_map_states_a_verification_state_on_every_row():
    """A capability map that quietly drops the honest states is marketing."""
    text = (DOCS / 'capability-map.md').read_text()
    for state in ('WATER', 'BENCH', 'BUILT', 'BLOCKED'):
        assert state in text, f'the {state} state is not used'
    assert 'has not been in water' in text, (
        'the map no longer says this platform has never flown -- if that changed, '
        'the rows should say WATER and this test should be updated deliberately')
