"""The weekly drift report finds each kind of drift, and only drift (#88 G3).

Every check is handed text and GitHub states it did not fetch, so each one is
shown firing on a planted defect and staying quiet on the honest version of
the same text. The workflow that runs it is checked for its shape: weekly,
one command, and only the permission it needs.
"""
import datetime as dt
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import backlog_drift as bd  # noqa: E402

TODAY = dt.date(2026, 11, 9)
FW = 'RakibulIslam1/srot-control-board'


def test_an_upstream_merge_is_reported_with_every_page_that_cites_it():
    docs = {
        'a.md': f'waits on https://github.com/{FW}/pull/39 to land',
        'b.md': 'the latch is upstream (srot-control-board#41)',
        'c.md': f'also https://github.com/{FW}/issues/41',
    }
    cited = bd.upstream_citations(docs)
    assert cited == {(FW, 39): ['a.md'], (FW, 41): ['b.md', 'c.md']}
    assert bd.upstream_moved(cited, {(FW, 39): 'OPEN', (FW, 41): 'OPEN'}) == []
    moved = bd.upstream_moved(cited, {(FW, 39): 'MERGED', (FW, 41): 'CLOSED'})
    assert len(moved) == 2 and 'MERGED' in moved[0] and '`a.md`' in moved[0]
    assert '`b.md`, `c.md`' in moved[1]


def test_an_upstream_item_that_cannot_be_read_is_reported_not_skipped():
    cited = bd.upstream_citations({'a.md': 'srot-control-board#7'})
    assert 'could not be read' in bd.upstream_moved(cited, {(FW, 7): None})[0]


def test_our_own_issue_links_are_not_upstream():
    docs = {'a.md': 'see https://github.com/fh1m/mongla_ws/issues/80 and #80'}
    assert bd.upstream_citations(docs) == {}


BUGS = """\
### B-1: one `READ 2026-11-01` **FIXED 2026-11-02**

Body. GitHub #10.

### B-2: two `READ 2026-11-01`

Body. GitHub #11.

### B-3: three `READ 2026-11-01` **FIXED 2026-11-02 (host)**

Body. GitHub #12.
"""


def test_bugs_and_issues_that_agree_report_nothing():
    assert bd.bugs_vs_issues(BUGS, {10: 'CLOSED', 11: 'OPEN', 12: 'OPEN'}) == []


def test_bugs_and_issues_that_disagree_are_each_reported():
    out = bd.bugs_vs_issues(BUGS, {10: 'OPEN', 11: 'CLOSED', 12: 'CLOSED'})
    assert len(out) == 2
    assert out[0].startswith('B-1 is FIXED but #10 is still open')
    assert out[1].startswith('B-2 is not FIXED but #11 is closed')


def test_a_scoped_fix_waits_on_someone_else_and_is_not_drift():
    """`(host)`: our half is done; the issue may be open or closed."""
    for state in ('OPEN', 'CLOSED'):
        assert bd.bugs_vs_issues(BUGS, {10: 'CLOSED', 11: 'OPEN', 12: state}) == []


def test_a_bug_naming_an_issue_that_does_not_exist_is_reported():
    out = bd.bugs_vs_issues(BUGS, {11: 'OPEN', 12: 'OPEN'})
    assert out == ['B-1 names GitHub #10, which does not exist']


ROADMAP = """\
| G1 | depth loop | OPEN (last read 2026-09-15) | bench |
| G2 | thrusters | OPEN (hardware) | blocks |
| G3 | firmware | OPEN (re-checked 2026-11-08) | gh |
| C1 | done | DONE 2026-09-01 | x |
"""


def test_only_a_dated_open_row_past_a_week_is_stale():
    out = bd.roadmap_stale(ROADMAP, TODAY)
    assert len(out) == 1 and out[0].startswith('`G1`') and '55 days' in out[0]


def test_the_threshold_is_the_roadmaps_own_week():
    assert bd.STALE_DAYS == 7
    text = (ROOT / '.claude/context/ROADMAP.md').read_text()
    assert 'older than a' in text and 'week' in text


def test_an_unlabelled_open_issue_is_untriaged():
    issues = [
        {'number': 1, 'title': 'a', 'state': 'OPEN', 'labels': []},
        {'number': 2, 'title': 'b', 'state': 'OPEN', 'labels': [{'name': 'bug'}]},
        {'number': 3, 'title': 'c', 'state': 'CLOSED', 'labels': []},
    ]
    assert bd.untriaged(issues) == ['#1 a']


def test_the_report_names_every_section_even_when_empty():
    body = bd.render({'One': [], 'Two': ['x']}, TODAY)
    assert '## One (0)\n- none' in body and '## Two (1)\n- x' in body


def test_it_reads_the_real_ledgers_without_error():
    """The parsers on the files they will meet, with no network."""
    bugs = (ROOT / bd.BUGS).read_text()
    entries = bd.bug_entries(bugs)
    assert len(entries) > 20 and all(e[0].startswith('B') for e in entries)
    assert any(e[1] is None for e in entries), 'no scoped fix parsed'
    bd.roadmap_stale((ROOT / bd.ROADMAP).read_text(), TODAY)


def test_the_workflow_runs_weekly_with_issue_rights_only():
    wf = yaml.safe_load((ROOT / '.github/workflows/backlog-drift.yml').read_text())
    on = wf.get('on', wf.get(True))
    assert 'schedule' in on and 'workflow_dispatch' in on
    assert 'pull_request' not in on, 'a report, not a merge gate'
    assert wf['permissions'] == {'contents': 'read', 'issues': 'write'}
    steps = wf['jobs']['drift']['steps']
    runs = [s['run'] for s in steps if 'run' in s]
    assert runs == ['python3 tools/backlog_drift.py --write-issue']
