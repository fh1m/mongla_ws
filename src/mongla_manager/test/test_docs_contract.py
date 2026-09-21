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
import pathlib
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]          # .../mongla_ws
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
        f'.claude/context/platform/legacy-pixhawk-and-sitl.md')


def test_the_legacy_page_exists_so_the_simulator_is_not_undocumented():
    """The pixhawk backend and ArduSub SITL are still live code. Retiring the
    documentation without this page would leave `sim/` unexplained."""
    found = list(CTX.rglob('legacy-pixhawk-and-sitl.md'))
    assert found, 'the legacy page was deleted; sim/ has no documentation'
    page = found[0]
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
    for repo in ('srot-control-board', 'srot-ground-station', 'srot-esc-flasher', 'mongla_ws'):
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


# ── 5. the rename holds, and the record stays straight ──────────────────────

# The project was renamed from the vehicle's name to its own in September 2026,
# when the author left the university that owned those vehicles. The word may
# appear in history -- a defect record, a competition result -- but never as a
# live identifier, and never as a claim of current affiliation.
RETIRED_NAME = 'duburi'

# Filenames that live on the FIRMWARE team's repositories. We reference them; we
# cannot rename them -- that would take a pull request on someone else's repo.
FOREIGN = ('TASKS_FROM_DUBURI_WS', 'DUBURI_WS_INTEGRATION', 'INTEGRATION_DUBURI_WS',
           'DUBURI_WS_PR', 'bracuduburi')

CODE_ROOTS = ('src', 'sim/src', 'scripts', 'tools')


# Only source-shaped files, and never the recorded datasets (gigabytes of frames).
SOURCE_SUFFIXES = {'.py', '.sh', '.md', '.yaml', '.yml', '.xml', '.cfg', '.json',
                   '.html', '.css', '.js', '.jsx', '.sdf', '.urdf', '.rviz',
                   '.action', '.msg', '.in', '.dsv', '.rules', '.pkla', '.txt'}
SKIP_DIRS = {'__pycache__', '.pytest_cache', 'node_modules', 'datasets',
             'build', 'install', 'log', 'models', 'meshes'}


def _source_files():
    for root in CODE_ROOTS:
        base = ROOT / root
        if not base.is_dir():
            continue
        for f in base.rglob('*'):
            if not f.is_file() or f.suffix.lower() not in SOURCE_SUFFIXES:
                continue
            if any(part in SKIP_DIRS for part in f.parts):
                continue
            yield f


def test_no_live_code_carries_the_retired_project_name():
    bad = []
    for f in _source_files():
            try:
                text = f.read_text()
            except (UnicodeDecodeError, OSError):
                continue
            if f.name == pathlib.Path(__file__).name:
                continue                      # this file names the retired word to ban it
            for i, line in enumerate(text.splitlines(), 1):
                probe = line
                for foreign in FOREIGN:
                    probe = probe.replace(foreign, '')
                if RETIRED_NAME in probe.lower():
                    bad.append(f'{f.relative_to(ROOT)}:{i}: {line.strip()[:80]}')
    assert not bad, (
        'the retired project name is live in code again:\n  ' + '\n  '.join(bad[:20]))


def test_no_path_carries_the_retired_project_name():
    bad = [str(f.relative_to(ROOT)) for f in _source_files()
           if RETIRED_NAME in f.name.lower()]
    assert not bad, 'paths still named for the retired project:\n  ' + '\n  '.join(bad[:20])


def test_authorship_is_stated_once_and_consistently():
    authors = ROOT / 'AUTHORS.md'
    assert authors.is_file(), 'AUTHORS.md is missing'
    text = authors.read_text()
    assert 'Muhammad Fahim Faisal' in text, 'the author is not named'
    assert 'Rakibul Islam' in text, 'the firmware co-author is not credited'

    import re
    names = set()
    for pkg in (ROOT / 'src').glob('*/package.xml'):
        names |= set(re.findall(r'<maintainer[^>]*>([^<]+)</maintainer>', pkg.read_text()))
    assert names == {'Muhammad Fahim Faisal'}, f'package maintainers disagree: {sorted(names)}'


def test_nothing_claims_a_current_institutional_affiliation():
    """Past tense is allowed and wanted -- the competition record is real. A
    PRESENT-tense claim of belonging is what must not exist."""
    claims = ('BRACU', 'BRAC University Mongla', "BRAC University's")
    bad = []
    for name in ENTRY_POINTS + ('AUTHORS.md',):
        text = (ROOT / name).read_text()
        for c in claims:
            if c in text:
                bad.append(f'{name}: {c}')
    assert not bad, 'a current affiliation is claimed: ' + ', '.join(bad)


# ── the byte figure on the site is the frame the code sends ─────────────────
def test_site_wire_frame_is_the_frame_the_code_sends():
    """docs/index.html draws `move_forward` as its 44 real bytes. Those bytes are
    generated by tools/wire_frame.py from `srot_fc._build_params` and the
    protocol constants. If the verb table or a wire constant changes and nobody
    re-runs the tool, the site shows a frame the vehicle no longer sends -- a
    plausible number standing in for the real one. `--check` re-derives the
    figure and compares it with the page."""
    import subprocess
    import sys
    pytest.importorskip('pymavlink')
    r = subprocess.run([sys.executable, str(ROOT / 'tools' / 'wire_frame.py'), '--check'],
                       capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, r.stdout + r.stderr


def test_site_ledger_strip_is_counted_from_the_ledger():
    """The retractions section states how many ledger entries took a result
    back, and draws every entry. Both are counted from measured-bars.md by
    tools/ledger_strip.py. The hand-written sentence it replaced said "more
    than half" when the file supported about a fifth -- so the count is now
    generated, and this fails the moment the page and the file disagree."""
    import subprocess
    import sys
    r = subprocess.run([sys.executable, str(ROOT / 'tools' / 'ledger_strip.py'), '--check'],
                       capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, r.stdout + r.stderr


def test_every_css_token_the_site_uses_is_defined():
    """A `var(--x)` naming a token that does not exist is not an error in CSS:
    the declaration is silently dropped. That is how the two Markdown pages lost
    their monospace tables and their code background -- a palette rename left
    `--red`, `--mono` and `--hull` behind in the layout, and nothing noticed.
    Tokens set inline by a generated figure (`style="--w:.."`) count as defined."""
    token = re.compile(r'(--[a-z0-9-]+)\s*:')
    files = [DOCS / 'assets' / 'site.css', DOCS / 'index.html',
             DOCS / '_layouts' / 'default.html']
    defined = set()
    for f in files:
        defined |= set(token.findall(f.read_text(encoding='utf-8')))
    missing = {}
    for f in files:
        used = set(re.findall(r'var\((--[a-z0-9-]+)', f.read_text(encoding='utf-8')))
        if used - defined:
            missing[f.name] = sorted(used - defined)
    assert not missing, f'undefined CSS tokens: {missing}'
