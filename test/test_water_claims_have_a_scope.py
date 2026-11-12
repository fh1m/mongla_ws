"""A water claim names the platform it is about, and only evidence upgrades one (#87).

Two failures, both shipped. The README said "This vehicle has never been in
water", which collapsed the whole lineage into the current platform: the stack
this repository began as competed at RoboSub 2026, and the author's vision
stack at RoboSub 2025. And a deliberately false line, "The current SROT and
Kutum configuration completed end-to-end water trials in 2026", passed every
required check (#87, reproduced 2026-11-09).

So every time a vehicle in the lineage was in water is one event in
`docs/evidence/water.json`, with its platform, what ran and the evidence. A
status belongs to the platform it was earned on. This file checks the registry
with a real validator, keeps `lineage.md` generated from it, and reads every
public page: a sentence saying something was never in water must name which
platform, and a sentence putting the current platform in water fails unless
the registry holds a CURRENT_WATER event with a recording behind it.

Text-level, like the other claim guards: it catches the sentence shapes that
shipped and the ones planted below, not every way English can say a thing.
"""
import datetime as dt
import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / 'docs/evidence/water.json'
sys.path.insert(0, str(ROOT / 'tools' / 'ci'))
import gen_lineage  # noqa: E402

STATUSES = {'HISTORICAL_WATER', 'RECORDED_WATER_REPLAY', 'CURRENT_WATER', 'NOT_YET'}
KINDS = {'path', 'commit', 'upstream', 'archive', 'recording'}
FORMAT = {'year': '%Y', 'month': '%Y-%m', 'day': '%Y-%m-%d'}

# Records of a day: they quote the state at the time (as in
# test_claims_say_where_they_came_from.py).
HISTORICAL = ('.claude/context/upstream/', '.claude/context/sota/',
              '.claude/context/measured-bars.md', '.claude/skills/')
# A sentence naming one of these is scoped to the current platform.
SCOPE = ('SROT', 'Kutum', 'Mongla hull', 'current platform')
_NEVER = re.compile(r'never (?:been )?(?:in|under) ?water|in water\W{0,3}never'
                    r'|never swum|\b0 dives\b', re.I)
_IN_WATER = re.compile(r'water trials?|in-water test|(?:tested|validated|proven) '
                       r'(?:in|under) ?water|\bswam\b|\bdived\b|been in water'
                       r'|completed\b.{0,60}\bin water', re.I)
_HEDGE = re.compile(r"\b(?:never|not|no|yet|unverified|pending|until|before|"
                    r"once|must|will|would|first|blocked|has to)\b|n't", re.I)
_CURRENT = ('Kutum', 'current platform', 'Mongla hull', 'current SROT',
            'current configuration')


# ── the registry ────────────────────────────────────────────────────────────
def _tracked() -> set[str]:
    out = subprocess.run(['git', 'ls-files'], cwd=ROOT, check=True,
                         capture_output=True, text=True).stdout
    return set(out.split())


def validate(reg: dict, tracked: set[str]) -> list[str]:
    """Every way the registry can be wrong, as messages; [] when sound."""
    errs, seen = [], set()
    cur = reg.get('current_platform')
    if cur not in reg.get('platforms', {}):
        errs.append(f'current_platform {cur!r} is not a listed platform')
    for ev in reg.get('events', []):
        i = ev.get('id', '?')
        if i in seen:
            errs.append(f'{i}: duplicate id')
        seen.add(i)
        if ev.get('status') not in STATUSES:
            errs.append(f'{i}: status {ev.get("status")!r} is not one of {sorted(STATUSES)}')
        if ev.get('platform') not in reg.get('platforms', {}):
            errs.append(f'{i}: platform {ev.get("platform")!r} is not listed')
        fmt = FORMAT.get(ev.get('date_precision'))
        try:
            dt.datetime.strptime(str(ev.get('date')), fmt or '')
        except (ValueError, TypeError):
            errs.append(f'{i}: date {ev.get("date")!r} does not match precision '
                        f'{ev.get("date_precision")!r}')
        if not ev.get('evidence'):
            errs.append(f'{i}: no evidence')
        for e in ev.get('evidence', []):
            kind, ref = e.get('kind'), str(e.get('ref', ''))
            if kind not in KINDS:
                errs.append(f'{i}: evidence kind {kind!r}')
            elif kind in ('path', 'recording') and ref not in tracked:
                errs.append(f'{i}: {ref!r} is not a tracked file')
            elif kind == 'commit' and not re.fullmatch(r'[0-9a-f]{7,40}', ref):
                errs.append(f'{i}: {ref!r} is not a commit id')
            if not e.get('note'):
                errs.append(f'{i}: evidence {ref!r} says nothing about itself')
        on_current = ev.get('platform') == cur
        if ev.get('status') in ('CURRENT_WATER', 'NOT_YET') and not on_current:
            errs.append(f'{i}: {ev["status"]} on {ev.get("platform")!r}, which is '
                        f'not the current platform')
        if on_current and ev.get('status') in ('HISTORICAL_WATER',
                                               'RECORDED_WATER_REPLAY'):
            errs.append(f'{i}: the current platform cannot have a past')
        if ev.get('status') == 'CURRENT_WATER' and not any(
                e.get('kind') == 'recording' for e in ev.get('evidence', [])):
            errs.append(f'{i}: CURRENT_WATER needs a recording in this repository')
    return errs


def _registry():
    return json.loads(REGISTRY.read_text())


def _current_water(reg) -> bool:
    return any(e['status'] == 'CURRENT_WATER' for e in reg['events'])


def test_the_registry_is_sound():
    assert validate(_registry(), _tracked()) == []


@pytest.mark.parametrize('break_it, needle', [
    (lambda r: r['events'].append(dict(r['events'][0])), 'duplicate id'),
    (lambda r: r['events'][0].update(status='WATER'), 'is not one of'),
    (lambda r: r['events'][0].update(date='2025-13'), 'does not match'),
    (lambda r: r['events'][0]['evidence'].append(
        {'kind': 'path', 'ref': 'docs/nope.md', 'note': 'x'}), 'not a tracked file'),
    (lambda r: r['events'][0].update(platform=r['current_platform']),
     'cannot have a past'),
    (lambda r: r['events'][-1].update(status='CURRENT_WATER'),
     'needs a recording'),
    (lambda r: r['events'][0].update(evidence=[]), 'no evidence'),
])
def test_the_validator_refuses_each_broken_registry(break_it, needle):
    reg = _registry()
    break_it(reg)
    assert any(needle in e for e in validate(reg, _tracked()))


def test_the_lineage_page_is_generated_from_the_registry():
    page = (ROOT / 'docs/evidence/lineage.md').read_text()
    assert page == gen_lineage.render(_registry()), 'run tools/ci/gen_lineage.py'


def test_the_front_pages_state_the_status_the_registry_derives():
    """The README footnote and the site's "In water" gauge are written by
    gen_lineage from the registry, not checked against it by hand."""
    for path, text in gen_lineage.front_pages(_registry()).items():
        assert path.read_text(encoding='utf-8') == text, (
            f'{path.name}: run tools/ci/gen_lineage.py')


def test_a_first_dive_in_the_registry_changes_both_front_pages():
    reg = _registry()
    assert gen_lineage.status(reg)['dives'] == 0
    reg['events'].append({
        'id': 'W-TEST', 'platform': reg['current_platform'],
        'status': 'CURRENT_WATER', 'date': '2099-01-02'})
    st = gen_lineage.status(reg)
    assert st['dives'] == 1 and 'never' not in st['sentence']
    assert '1 recorded dive, the latest on 2099-01-02 (W-TEST)' in st['sentence']
    pages = gen_lineage.front_pages(reg)
    assert '<b>1 dive</b>' in pages[gen_lineage.SITE]
    assert 'has never been in water; its predecessors' \
        not in pages[gen_lineage.README]
    assert st['sentence'] in pages[gen_lineage.README]


def test_a_status_sentence_reworded_by_hand_is_refused(tmp_path, monkeypatch):
    """A sentence the generator cannot find is one it no longer writes."""
    copy = tmp_path / 'README.md'
    copy.write_text(gen_lineage.README.read_text(encoding='utf-8').replace(
        '<sub>The current platform,', '<sub>Our vehicle,'), encoding='utf-8')
    monkeypatch.setattr(gen_lineage, 'README', copy)
    monkeypatch.setattr(gen_lineage, 'ROOT', tmp_path)
    with pytest.raises(SystemExit, match='found it 0 times'):
        gen_lineage.front_pages(_registry())


def test_the_record_keeps_the_predecessors_and_the_current_platform_honest():
    reg = _registry()
    by = {e['id']: e for e in reg['events']}
    assert by['W-RS26']['status'] == 'HISTORICAL_WATER'
    assert by['W-RS25']['status'] == 'HISTORICAL_WATER'
    assert by['W-DUBOMINI26']['status'] == 'HISTORICAL_WATER'
    cur = [e for e in reg['events'] if e['platform'] == reg['current_platform']]
    assert [e['status'] for e in cur] == ['NOT_YET'], \
        'CURRENT_WATER is earned by a recording, never by an edit'


# ── the public pages ────────────────────────────────────────────────────────
def _surfaces() -> dict[str, str]:
    names = [f for f in _tracked()
             if (f.endswith('.md') or f == 'docs/index.html')
             and not f.startswith(HISTORICAL)]
    return {f: (ROOT / f).read_text(errors='replace') for f in sorted(names)
            if (ROOT / f).is_file()}


def _sentences(text: str):
    text = re.sub(r'<[^>]+>', ' ', text)                # tags, for the site
    text = re.sub(r'\n\s*>?\s*', ' ', text)            # rejoin wrapped lines
    return re.split(r'(?<=[.!?])\s+', text)


def violations(files: dict[str, str], current_water: bool) -> list[str]:
    out = []
    for path, text in files.items():
        for s in _sentences(text):
            if _NEVER.search(s) and not any(k in s for k in SCOPE):
                out.append(f'{path}: unscoped "never in water": {s.strip()[:140]}')
            if (not current_water and _IN_WATER.search(s)
                    and any(k in s for k in _CURRENT) and not _HEDGE.search(s)):
                out.append(f'{path}: puts the current platform in water with no '
                           f'CURRENT_WATER event: {s.strip()[:140]}')
    return out


def test_every_public_page_scopes_its_water_claims():
    assert violations(_surfaces(), _current_water(_registry())) == []


FALSE_CURRENT = ('The current SROT and Kutum configuration completed end-to-end '
                 'water trials in 2026.')


@pytest.mark.parametrize('planted', [
    FALSE_CURRENT,                                        # #87's own reproduction
    'This vehicle has never been in water.',             # the README that shipped
    'In water: 0 dives.',
    'The Kutum stack was validated in water last week.',
])
def test_a_planted_claim_fails(planted):
    readme = (ROOT / 'README.md').read_text()
    files = {'README.md': readme + '\n\n' + planted + '\n'}
    assert violations(files, current_water=False), planted


@pytest.mark.parametrize('honest', [
    'The current platform, this stack on the SROT board and Kutum in the Mongla '
    'hull, has never been in water.',
    'The stack this repository began as competed at RoboSub 2026.',
    'The SROT board has been in water, in the old Dubomini hull.',
    'Kutum has not yet been in water.',
])
def test_an_honest_claim_passes(honest):
    assert violations({'x.md': honest}, current_water=False) == []


def test_a_real_current_water_event_would_lift_the_upgrade_check():
    assert violations({'x.md': FALSE_CURRENT}, current_water=True) == []
