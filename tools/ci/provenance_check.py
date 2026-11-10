#!/usr/bin/env python3
"""Decide the provenance check from a pull request body.

    PR_BODY="$(cat body.md)" PR_AUTHOR=login python3 tools/ci/provenance_check.py

The body arrives in the environment, never as shell source: it is written by
whoever opens the pull request. Exit 0 passes, 1 fails; the verdict word is
also written to $GITHUB_OUTPUT as `status=` when that is set.

A body passes in one of two ways, each on lines of its own and outside code
fences and quotes:

* the declaration line, exactly (not in an HTML comment either)

      Provenance: 100% human-written

* a `## AI Provenance` heading followed by its table: the header
  `| # | Author / Handle | Source URL | What came from this person |`, the
  separator, and at least one row numbered from 1 with every cell filled.

A pull request opened by Dependabot passes as `bot`: it bumps declared
dependencies and carries no authored code. The author comes from the event
(`PR_AUTHOR`), not the body, and GitHub reserves `[bot]` logins for apps, so
a person cannot claim it.

The old check passed any body that contained the phrase anywhere, so
"this PR is NOT 100% human-written" passed as exempt (#97). Both forms are
what the author declares; neither is verified against how the code was made.
"""
from __future__ import annotations

import os
import re
import sys

DECLARATION = 'Provenance: 100% human-written'
HEADING = re.compile(r'##\s+AI Provenance\s*')
HEADER = ['#', 'Author / Handle', 'Source URL', 'What came from this person']


def _plain_lines(body: str) -> list[str]:
    """The body's lines outside ``` / ~~~ fences and HTML comments, with
    quoted lines dropped: what a reader sees as the author's own text."""
    body = re.sub(r'<!--.*?(-->|$)', '', body.replace('\r\n', '\n'), flags=re.S)
    out, fence = [], None
    for raw in body.split('\n'):
        line = raw.strip()
        marker = line[:3]
        if marker in ('```', '~~~'):
            if fence is None:
                fence = marker
            elif marker == fence:
                fence = None
            continue
        if fence is None and not line.startswith('>'):
            out.append(line)
    return out


def _cells(line: str):
    if not (line.startswith('|') and line.endswith('|') and len(line) > 1):
        return None
    return [c.strip() for c in line[1:-1].split('|')]


def _table_ok(lines: list[str]) -> bool:
    rows = []
    for line in lines:
        if line.startswith('#'):
            break
        cells = _cells(line)
        if cells is not None:
            rows.append(cells)
        elif rows:
            break                       # the table ended
    if len(rows) < 3 or rows[0] != HEADER:
        return False
    if not all(re.fullmatch(r':?-{3,}:?', c) for c in rows[1]) or len(rows[1]) != 4:
        return False
    data = rows[2:]
    return all(len(r) == 4 and all(r) for r in data) and \
        [r[0] for r in data] == [str(k) for k in range(1, len(data) + 1)]


BOTS = frozenset({'dependabot[bot]'})


def verdict(body: str, author: str = '') -> str:
    """'bot', 'exempt', 'pass', 'empty', 'missing' or 'contradictory'."""
    if author in BOTS:
        return 'bot'
    lines = _plain_lines(body or '')
    declared = DECLARATION in lines
    at = [i for i, line in enumerate(lines) if HEADING.fullmatch(line)]
    if declared and at:
        return 'contradictory'
    if declared:
        return 'exempt'
    if not at:
        return 'missing'
    return 'pass' if _table_ok(lines[at[0] + 1:]) else 'empty'


MESSAGES = {
    'bot': 'Opened by Dependabot: a dependency bump, no authored code.',
    'exempt': 'Declared 100% human-written.',
    'pass': 'AI Provenance section found and populated.',
    'empty': 'AI Provenance heading found, but its table is missing, malformed or empty.',
    'missing': 'No AI Provenance section and no human-written declaration.',
    'contradictory': 'Both a human-written declaration and an AI Provenance section.',
}


def main() -> int:
    status = verdict(os.environ.get('PR_BODY', ''), os.environ.get('PR_AUTHOR', ''))
    out = os.environ.get('GITHUB_OUTPUT')
    if out:
        with open(out, 'a', encoding='utf-8') as f:
            f.write(f'status={status}\n')
    ok = status in ('bot', 'exempt', 'pass')
    print(('ok ' if ok else 'FAIL ') + MESSAGES[status])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
