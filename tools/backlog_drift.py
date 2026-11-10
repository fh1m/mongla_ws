#!/usr/bin/env python3
"""Where the written backlog has drifted from what GitHub says is true (#88 G3).

    python3 tools/backlog_drift.py                 # print the report
    python3 tools/backlog_drift.py --write-issue   # upsert the one report issue

Run weekly by `.github/workflows/backlog-drift.yml`. GitHub is the source of
truth for issue and pull request state. This compares the files that restate
that state against it, and nothing else:

- an upstream issue or pull request the docs cite has merged or closed, so
  every page waiting on it is now describing the past;
- a BUGS.md entry and the GitHub issue it names disagree (FIXED here and still
  open there, or closed there and not FIXED here);
- a ROADMAP.md row says OPEN on a check older than the file's own rule of a
  week ("Re-grep before trusting any OPEN row older than a week");
- an open issue carries no label, so no triage has happened.

It reports, it does not fix and it does not fail a build: each finding needs
a person to decide whether the doc or the issue is wrong. The report is one
issue, labelled `drift-report`, rewritten in place each run, closed when clean
and reopened when not, so it never multiplies.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUGS = '.claude/context/BUGS.md'
ROADMAP = '.claude/context/ROADMAP.md'
REPORT_TITLE = 'Backlog drift report'
REPORT_LABEL = 'drift-report'
STALE_DAYS = 7          # ROADMAP.md: "an OPEN row older than a week"

# The sibling repositories, by link or by the `srot-control-board#41`
# shorthand the docs also use. The shorthand names no owner; every sibling
# lives under the firmware lead's account.
UPSTREAM_OWNER = 'RakibulIslam1'
_UPSTREAM = re.compile(
    r'github\.com/(?P<repo>[A-Za-z0-9_.-]+/srot-[A-Za-z0-9_.-]+)/(?:pull|issues)/(?P<n>\d+)'
    r'|(?<![/\w-])(?P<short>srot-[a-z-]+)#(?P<sn>\d+)')
_BUG_HEAD = re.compile(r'^### (?P<id>B-?\d+)\b(?P<rest>.*)$', re.M)
_GH_REF = re.compile(r'GitHub #(\d+)')
_DATE = re.compile(r'(\d{4}-\d{2}-\d{2})')


# ── the checks: pure, so each one is tested on text it was handed ──────────
def upstream_citations(files: dict[str, str]) -> dict[tuple[str, int], list[str]]:
    """(repo, number) -> the files that cite that upstream issue or pull request."""
    out: dict[tuple[str, int], list[str]] = {}
    for path, text in sorted(files.items()):
        for m in _UPSTREAM.finditer(text):
            key = ((m['repo'], int(m['n'])) if m['repo'] else
                   (f'{UPSTREAM_OWNER}/{m["short"]}', int(m['sn'])))
            if path not in out.setdefault(key, []):
                out[key].append(path)
    return out


def upstream_moved(cited, states: dict[tuple[str, int], str | None]) -> list[str]:
    """Cited upstream pull requests that are no longer open, or unreadable.

    Unreadable is reported, not skipped: a check that cannot see is not a
    check that passed.
    """
    out = []
    for key in sorted(cited):
        state = states.get(key)
        repo, n = key
        files = ', '.join(f'`{f}`' for f in cited[key])
        if state in ('MERGED', 'CLOSED'):
            out.append(f'{repo}#{n} is **{state}**; cited by {files}')
        elif state is None:
            out.append(f'{repo}#{n} could not be read; cited by {files}')
    return out


def bug_entries(text: str) -> list[tuple[str, bool | None, list[int]]]:
    """(id, fixed, [GitHub issue numbers]) for each BUGS.md entry.

    `fixed` is None for a scoped fix, `**FIXED 2026-11-09 (host)**`: our half
    is done and the rest waits elsewhere, so the issue may rightly be open or
    closed and neither is drift.
    """
    heads = list(_BUG_HEAD.finditer(text))
    out = []
    for i, h in enumerate(heads):
        end = heads[i + 1].start() if i + 1 < len(heads) else len(text)
        body = text[h.start():end]
        fixed = '**FIXED' in h['rest']
        if fixed and re.search(r'\*\*FIXED[^*]*\(', h['rest']):
            fixed = None
        refs = sorted({int(n) for n in _GH_REF.findall(body)})
        out.append((h['id'], fixed, refs))
    return out


def bugs_vs_issues(text: str, issues: dict[int, str]) -> list[str]:
    """BUGS.md entries whose status disagrees with the issue they name."""
    out = []
    for bug, fixed, refs in bug_entries(text):
        if fixed is None:
            continue
        for n in refs:
            state = issues.get(n)
            if state is None:
                out.append(f'{bug} names GitHub #{n}, which does not exist')
            elif fixed and state == 'OPEN':
                out.append(f'{bug} is FIXED but #{n} is still open: close it, or '
                           f'say in the issue what is left')
            elif not fixed and state == 'CLOSED':
                out.append(f'{bug} is not FIXED but #{n} is closed')
    return out


def roadmap_stale(text: str, today: dt.date, days: int = STALE_DAYS) -> list[str]:
    """Table rows that say OPEN on a check older than `days`."""
    out = []
    for line in text.splitlines():
        if not line.startswith('|') or 'OPEN' not in line:
            continue
        cells = [c.strip() for c in line.strip('|').split('|')]
        status = next((c for c in cells if 'OPEN' in c), '')
        dates = [dt.date.fromisoformat(d) for d in _DATE.findall(line)
                 if _valid(d)]
        if not dates:
            continue                    # undated: nothing to compare against
        last = max(dates)
        age = (today - last).days
        if age > days:
            out.append(f'`{cells[0]}` says {status[:60]!r}, last dated '
                       f'{last} ({age} days)')
    return out


def untriaged(issues: list[dict]) -> list[str]:
    """Open issues with no label."""
    return [f'#{i["number"]} {i["title"][:80]}' for i in issues
            if i['state'] == 'OPEN' and not i.get('labels')]


def _valid(d: str) -> bool:
    try:
        dt.date.fromisoformat(d)
        return True
    except ValueError:
        return False


def render(sections: dict[str, list[str]], today: dt.date) -> str:
    lines = [f'Generated {today} by `tools/backlog_drift.py` (#88). GitHub is the '
             f'source of truth; each line is a written claim that disagrees with '
             f'it. Fix the doc or the issue, never this report.', '']
    for title, items in sections.items():
        lines.append(f'## {title} ({len(items)})')
        lines += [f'- {x}' for x in items] or ['- none']
        lines.append('')
    return '\n'.join(lines)


# ── the facts, from GitHub ──────────────────────────────────────────────────
def _gh(*args) -> str:
    return subprocess.run(['gh', *args], check=True, capture_output=True,
                          text=True).stdout


def _our_issues() -> list[dict]:
    raw = _gh('issue', 'list', '--state', 'all', '--limit', '1000',
              '--json', 'number,state,title,labels')
    return json.loads(raw)


def _upstream_state(repo: str, n: int) -> str | None:
    """OPEN, CLOSED or MERGED for an issue or a pull request; None if unreadable."""
    try:
        d = json.loads(_gh('api', f'repos/{repo}/issues/{n}'))
    except (subprocess.CalledProcessError, ValueError):
        return None
    if (d.get('pull_request') or {}).get('merged_at'):
        return 'MERGED'
    return str(d.get('state', '')).upper() or None


def _tracked_markdown(root: Path) -> dict[str, str]:
    names = subprocess.run(['git', 'ls-files', '*.md'], cwd=root, check=True,
                           capture_output=True, text=True).stdout.split()
    return {n: (root / n).read_text(errors='replace') for n in names
            if (root / n).is_file()}


def report(root: Path, today: dt.date) -> tuple[str, int]:
    docs = _tracked_markdown(root)
    issues = _our_issues()
    states = {i['number']: i['state'] for i in issues}
    cited = upstream_citations(docs)
    upstream = {key: _upstream_state(*key) for key in cited}
    sections = {
        'Upstream issues and pull requests that moved': upstream_moved(cited, upstream),
        'BUGS.md against GitHub issues': bugs_vs_issues(docs.get(BUGS, ''), states),
        'ROADMAP.md OPEN rows past their re-check': roadmap_stale(
            docs.get(ROADMAP, ''), today),
        'Open issues with no label': untriaged(issues),
    }
    return render(sections, today), sum(len(v) for v in sections.values())


def write_issue(body: str, n_findings: int) -> None:
    """Rewrite the one report issue in place; close it when clean."""
    _gh('label', 'create', REPORT_LABEL, '--force', '--color', 'BFD4F2',
        '--description', 'The weekly backlog drift report (tools/backlog_drift.py)')
    found = json.loads(_gh('issue', 'list', '--state', 'all', '--label',
                           REPORT_LABEL, '--json', 'number,state,title'))
    mine = [i for i in found if i['title'] == REPORT_TITLE]
    if not mine:
        if n_findings:
            _gh('issue', 'create', '--title', REPORT_TITLE, '--label',
                REPORT_LABEL, '--body', body)
        return
    n, state = str(mine[0]['number']), mine[0]['state']
    _gh('issue', 'edit', n, '--body', body)
    if n_findings and state == 'CLOSED':
        _gh('issue', 'reopen', n)
    elif not n_findings and state == 'OPEN':
        _gh('issue', 'close', n, '--comment', 'Clean this run.')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    ap.add_argument('--write-issue', action='store_true',
                    help='upsert the report issue instead of only printing it')
    args = ap.parse_args(argv)
    today = dt.date.today()
    body, n = report(ROOT, today)
    print(body)
    if args.write_issue:
        write_issue(body, n)
    return 0


if __name__ == '__main__':
    sys.exit(main())
