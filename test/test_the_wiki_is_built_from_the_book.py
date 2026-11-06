"""The wiki and the repo hold the same docs, in both directions, and every link lands.

Docs are read and edited in the GitHub wiki. `tools/wiki_build.py build` turns
the repo's files into wiki pages on every push to main; `pull` turns a wiki edit
back into its file, which is then committed (`.github/workflows/wiki.yml`).
Two things would quietly lose work: a page whose round trip changes it (an
edit to one line would rewrite others), and a link that lands on no page (a
404 on GitHub that nothing here sees).
"""
import importlib.util
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('wiki_build', ROOT / 'tools' / 'wiki_build.py')
wb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(wb)


def _build(tmp_path):
    assert wb.build(tmp_path) == 0
    return {p.stem: p.read_text(encoding='utf-8') for p in tmp_path.glob('*.md')}


def test_every_page_comes_back_as_exactly_its_file():
    names = wb.pages()
    changed = [p.relative_to(ROOT).as_posix() for p in names
               if not wb._is_stub(p, names)
               and wb.unrender(wb.render(p, names), p, names) != p.read_text(encoding='utf-8')]
    assert not changed, changed


def test_every_link_between_pages_lands_on_a_page(tmp_path):
    wiki = _build(tmp_path)
    dangling = [(name, t) for name, text in wiki.items()
                for t in re.findall(r'\]\(([^)\s]+)\)', text)
                if not re.match(r'^[a-z][a-z0-9+.-]*:|^#', t)
                and t.split('#')[0] not in wiki]
    assert not dangling, dangling[:10]


def test_every_chapter_of_the_book_is_a_page(tmp_path):
    wiki = _build(tmp_path)
    manifest = json.loads((tmp_path / wb.MANIFEST).read_text())
    book = (ROOT / '.claude/context/README.md').read_text(encoding='utf-8')
    chapters = {(wb.BOOK.parent / c).resolve().relative_to(ROOT).as_posix()
                for c in re.findall(r'^\d+\. \[[^\]]+\]\(([^)#]+)', book, flags=re.M)}
    assert chapters and chapters <= set(manifest.values()), chapters - set(manifest.values())
    assert set(manifest) <= set(wiki)


def test_home_sidebar_footer_and_every_page_says_it_can_be_edited(tmp_path):
    wiki = _build(tmp_path)
    assert {'Home', '_Sidebar', '_Footer'} <= set(wiki)
    unsigned = [n for n, t in wiki.items()
                if not n.startswith('_') and 'Edit this page here in the wiki' not in t]
    assert not unsigned, unsigned


def test_a_wiki_edit_lands_in_its_file_and_nowhere_else(tmp_path, monkeypatch):
    wb.build(tmp_path)
    manifest = json.loads((tmp_path / wb.MANIFEST).read_text())
    name = next(n for n, rel in manifest.items() if rel.endswith('ROADMAP.md'))
    page = tmp_path / f'{name}.md'
    page.write_text(page.read_text().replace('\n', '\nA line typed in the wiki.\n', 1))
    bugs = wb.pages()[ROOT / '.claude/context/BUGS.md']
    (tmp_path / 'A-new-page.md').write_text(f'# A new page\n\nSee [the bugs]({bugs}).\n')
    written = {}
    monkeypatch.setattr(Path, 'write_text',
                        lambda self, text, **kw: written.__setitem__(self, text))
    monkeypatch.setattr(Path, 'mkdir', lambda self, **kw: None)
    changed = wb.pull(tmp_path)
    assert sorted(changed) == ['.claude/context/ROADMAP.md', '.claude/context/wiki/A-new-page.md']
    roadmap = written[ROOT / '.claude/context/ROADMAP.md']
    assert 'A line typed in the wiki.' in roadmap and 'wiki:generated' not in roadmap
    assert '](../BUGS.md)' in written[ROOT / '.claude/context/wiki/A-new-page.md']


def test_no_published_page_names_this_machine(tmp_path):
    """The site once published paths from the dev box. The wiki must not.
    (`/home/user/` is the docs' placeholder, not a machine.)"""
    wiki = _build(tmp_path)
    hits = [n for n, t in wiki.items() if re.search(r'/home/(?!user/)[a-z_][a-z0-9_-]*/', t)]
    assert not hits, hits


def test_a_link_inside_a_code_fence_is_left_alone():
    names = {(wb.BOOK.parent / 'ROADMAP.md'): 'ROADMAP'}
    text = '```\n[x](ROADMAP.md)\n```\n[y](ROADMAP.md)\n'
    out = wb._map_links(text, lambda t: wb._to_wiki(wb.BOOK, t, names))
    assert '[x](ROADMAP.md)' in out and '[y](ROADMAP)' in out
