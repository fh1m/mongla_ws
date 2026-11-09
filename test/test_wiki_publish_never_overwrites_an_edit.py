"""Publishing the wiki never erases a human edit main does not have (#68).

The publisher used to start a fresh wiki history from main and force-push it.
An edit saved while its import was still queued, or still in review, was
replaced before anyone could merge it. Now the publisher runs
`wiki_build.py check` on the pages people changed since its last publish,
refuses while any of them differs from main, builds on top of the wiki's own
history, and pushes without force, so a page saved mid-run rejects the push.
"""
import importlib.util
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WIKI_YML = ROOT / '.github/workflows/wiki.yml'
_spec = importlib.util.spec_from_file_location('wiki_build', ROOT / 'tools/wiki_build.py')
wb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wb)


def _check(wiki: Path, *pages: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(ROOT / 'tools/wiki_build.py'), 'check',
                           str(wiki), *pages], capture_output=True, text=True)


def _a_page(wiki: Path) -> Path:
    return sorted(p for p in wiki.glob('*.md') if not p.name.startswith('_'))[0]


def _edit(page: Path) -> None:
    """A person's edit, in the body: text after the generated footer is
    discarded by design, so an edit there is not one."""
    head, rest = page.read_text().split('\n', 1)
    page.write_text(f'{head}\nA sentence a person added.\n{rest}')


def test_a_freshly_built_wiki_has_nothing_pending(tmp_path):
    wb.build(tmp_path)
    r = _check(tmp_path)
    assert r.returncode == 0, r.stdout


def test_an_edit_main_does_not_have_blocks_the_publish(tmp_path):
    """The race: a person saves a page; the publisher runs before the import
    is merged. `check` must name that page and fail, and must not write."""
    wb.build(tmp_path)
    page = _a_page(tmp_path)
    _edit(page)
    before = subprocess.run(['git', 'status', '--porcelain'], cwd=ROOT,
                            capture_output=True, text=True).stdout
    r = _check(tmp_path, page.name)
    assert r.returncode == 1
    assert r.stdout.strip(), 'the pending file is named'
    after = subprocess.run(['git', 'status', '--porcelain'], cwd=ROOT,
                           capture_output=True, text=True).stdout
    assert before == after, 'check wrote into the repository'


def test_only_the_pages_people_touched_are_judged(tmp_path):
    """A page nobody edited since the last publish never blocks it -- a
    document deleted from the book leaves such a page behind."""
    wb.build(tmp_path)
    edited = _a_page(tmp_path)
    _edit(edited)
    others = [p.name for p in tmp_path.glob('*.md') if p != edited]
    assert _check(tmp_path, *others).returncode == 0
    assert _check(tmp_path, edited.name).returncode == 1


def _publish_steps():
    doc = yaml.load(WIKI_YML.read_text(), Loader=yaml.BaseLoader)
    return doc['jobs']['publish']['steps']


def test_the_publisher_checks_before_it_builds_and_never_forces():
    steps = _publish_steps()
    names = [s.get('name', '') for s in steps]
    runs = '\n'.join(s.get('run', '') for s in steps)
    check = next(i for i, n in enumerate(names) if 'Refuse to publish' in n)
    build = next(i for i, n in enumerate(names) if n == 'Build the pages')
    fetch = next(i for i, n in enumerate(names) if n == 'Fetch the wiki')
    assert fetch < check < build
    assert 'wiki_build.py check' in steps[check]['run']
    assert '--force' not in runs and 'push -f' not in runs, 'a forced push erases edits'
    assert 'git init' not in runs, 'a fresh history replaces the wiki wholesale'
