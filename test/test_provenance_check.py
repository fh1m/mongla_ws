"""The provenance check decides from a declaration, not a substring (#97).

The required check used to pass any body containing "100% human-written",
so "this PR is NOT 100% human-written" passed as exempt. These cases run the
real tool, the way the workflow does: the body in the environment.
"""
from __future__ import annotations

import importlib.util
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools/provenance_check.py'
_spec = importlib.util.spec_from_file_location('provenance_check', TOOL)
pc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pc)

TABLE = ('## AI Provenance\n\n'
         '| # | Author / Handle | Source URL | What came from this person |\n'
         '|---|---|---|---|\n'
         '| 1 | fh1m | https://github.com/fh1m/mongla_ws/issues/97 | The audit |\n')


@pytest.mark.parametrize('body, want', [
    # The bypass from #97, and its relatives.
    ('This PR is NOT 100% human-written; please check AI provenance.', 'missing'),
    ('Provenance: not 100% human-written', 'missing'),
    ('> Provenance: 100% human-written', 'missing'),
    ('```\nProvenance: 100% human-written\n```', 'missing'),
    ('Text `Provenance: 100% human-written` inline', 'missing'),
    ('<!--\nProvenance: 100% human-written\n-->', 'missing'),
    ('<!-- unclosed\nProvenance: 100% human-written\n', 'missing'),
    # A real declaration, on its own line.
    ('<!-- note -->\nProvenance: 100% human-written', 'exempt'),
    ('Fixes a typo.\n\nProvenance: 100% human-written\n', 'exempt'),
    # The table.
    ('Summary.\n\n' + TABLE, 'pass'),
    (TABLE.replace('| 1 |', '| 1 |', 1) + '| 2 | RakibulIslam1 | https://x | Board |\n', 'pass'),
    ('## AI Provenance\n\nnone\n', 'empty'),
    (TABLE.replace('| 1 | fh1m |', '| 1 |  |'), 'empty'),
    (TABLE.replace('| 1 |', '| 3 |'), 'empty'),
    (TABLE.split('|---')[0], 'empty'),
    ('```\n' + TABLE + '```\n', 'missing'),
    ('## AI Provenance\n\n## Next\n' + TABLE.split('\n', 2)[2], 'empty'),
    # Both at once is not a declaration of either.
    ('Provenance: 100% human-written\n\n' + TABLE, 'contradictory'),
    ('', 'missing'),
])
def test_the_verdict(body, want):
    assert pc.verdict(body) == want


@pytest.mark.parametrize('body, code, status', [
    ('This PR is NOT 100% human-written.', 1, 'missing'),
    ('Provenance: 100% human-written', 0, 'exempt'),
    (TABLE, 0, 'pass'),
    # Shell metacharacters stay data.
    ('"; touch /tmp/pwned; echo "\n$(id)\n`id`', 1, 'missing'),
])
def test_the_tool_as_the_workflow_runs_it(tmp_path, body, code, status):
    out = tmp_path / 'out'
    env = dict(os.environ, PR_BODY=body, GITHUB_OUTPUT=str(out))
    r = subprocess.run([sys.executable, str(TOOL)], env=env, capture_output=True, text=True)
    assert r.returncode == code, r.stdout
    assert out.read_text() == f'status={status}\n'


def test_a_wiki_edit_pr_passes_the_check():
    """The wiki workflow opens PRs into main; their body must pass the same
    required check. Run its real body-building block."""
    text = (ROOT / '.github/workflows/wiki.yml').read_text()
    block = re.search(r'\n( +)\{\n(.*?)\n\1\} > "\$RUNNER_TEMP/body"', text, re.S)
    assert block, 'the body block in wiki.yml moved'
    script = '{\n' + block.group(2) + '\n}'
    for guards_ok in ('true', 'false'):
        env = dict(os.environ, WHO='fh1m', PAGES='Roadmap, Bugs', GUARDS_OK=guards_ok,
                   GITHUB_REPOSITORY='fh1m/mongla_ws', RUNNER_TEMP='/nonexistent')
        body = subprocess.run(['bash', '-c', script], env=env, capture_output=True,
                              text=True, check=False).stdout
        assert pc.verdict(body) == 'pass', body


def test_the_unfilled_template_fails_and_a_filled_one_passes():
    template = (ROOT / '.github/PULL_REQUEST_TEMPLATE.md').read_text()
    assert pc.verdict(template) == 'empty'
    filled = template.replace('| 1 |                |            |                           |',
                              '| 1 | fh1m | https://github.com/fh1m | The idea |', 1)
    filled = filled.replace('| 2 |                |            |                           |\n', '')
    assert pc.verdict(filled) == 'pass'


@pytest.mark.parametrize('author, want', [
    ('dependabot[bot]', 'bot'),
    ('dependabot', 'missing'),          # a person's login, not the app
    ('Dependabot[bot]', 'missing'),
    ('fh1m', 'missing'),
    ('', 'missing'),
])
def test_only_the_dependabot_app_is_exempt_by_author(author, want):
    """The author comes from the event, not the body, and GitHub reserves
    `[bot]` logins for apps; a body claiming to be Dependabot proves nothing."""
    assert pc.verdict('Bumps numpy. dependabot[bot]', author) == want


def test_the_workflow_passes_the_author_from_the_event():
    import yaml
    doc = yaml.load((ROOT / '.github/workflows/poc-check.yml').read_text(),
                    Loader=yaml.BaseLoader)
    step = next(s for s in doc['jobs']['poc-check']['steps']
                if s.get('run', '').strip() == 'python3 tools/provenance_check.py')
    assert step['env']['PR_AUTHOR'] == '${{ github.event.pull_request.user.login }}'
