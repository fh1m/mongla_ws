"""The README and the site show the test counts of the last full run, exactly.

They were typed by hand in six places until 2026-11-06. Now the root
`conftest.py` writes `docs/stats.json` at the end of every full run and
`tools/published_counts.py` copies it into the pages; this fails if any page
was edited by hand, or if a pattern stopped matching so a count would freeze.
"""
import importlib.util
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('published_counts',
                                               ROOT / 'tools' / 'published_counts.py')
pc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pc)


def test_every_page_shows_the_last_full_run():
    assert pc.apply(pc.load(), check=True) == 0


def test_every_published_count_has_exactly_one_home_on_its_page():
    s = pc.load()
    for page, pairs in pc.edits(s).items():
        text = page.read_text(encoding='utf-8')
        for pat, _ in pairs:
            assert len(re.findall(pat, text, flags=re.S)) == 1, (page.name, pat)


def test_the_stats_add_up():
    s = pc.load()
    assert set(s['suites']) == set(pc.SUITE_NAMES)
    for k in ('passed', 'failed', 'skipped', 'xfailed'):
        assert s[k] == sum(c.get(k, 0) for c in s['suites'].values()), k
    assert s['collected'] == sum(s[k] for k in ('passed', 'failed', 'skipped',
                                                'xfailed', 'xpassed'))


def test_a_hand_edited_count_is_caught(tmp_path, monkeypatch):
    """Injection: change one published number and the check must fail."""
    s = pc.load()
    fake = tmp_path / 'README.md'
    fake.write_text(pc.README.read_text().replace(
        f'{pc.num(s["passed"])} passed, ', f'{pc.num(s["passed"] + 1)} passed, ', 1))
    monkeypatch.setattr(pc, 'README', fake)
    assert pc.apply(s, check=True) == 1


def test_a_failing_run_is_published_red():
    s = json.loads(json.dumps(pc.load()))
    s['failed'] = 2
    assert pc._badge(s).endswith('-red') and '2%20failing' in pc._badge(s)
