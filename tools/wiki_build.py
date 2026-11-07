#!/usr/bin/env python3
"""The GitHub wiki is where Mongla's docs are read and edited. This keeps it and the repo one.

The arrangement. People read, organise and edit the docs in the wiki. Each
document also lives in this repo, as the file the wiki page was built from,
because the guards that keep the docs true run here, against files, in the
same commit as the code they describe: `test_docs_contract.py` (every page
reachable from the book, every link lands), `test_doc_drift.py` (no doc states
a constant the code no longer holds), the retired-stack guard. Two directions,
both automatic (`.github/workflows/wiki.yml`):

    repo -> wiki   on every push to main:   python3 tools/wiki_build.py build OUT
    wiki -> repo   on every wiki edit:      python3 tools/wiki_build.py pull WIKI

`pull` turns each edited page back into its file -- wiki links back into
relative paths, the generated footer off -- the doc guards run, and the change
is committed to main under the editor's name. `test_the_wiki_is_built_from_the_book.py`
proves the round trip loses nothing: every page, built and pulled, is its file.

What the wiki holds
  * Home        the front page, `.claude/context/wiki/Home.md`: what Mongla is, in pictures
  * the book    the contents page, `.claude/context/README.md`, every chapter in order
  * _Sidebar    the book, part by part, on every page
  * _Footer     where the code, the site and the defect register are
  * one page    per document the book reaches, titled by its own first heading
  * .mongla-pages.json   which file each page came from (what `pull` reads)

A page added in the wiki lands in `.claude/context/wiki/`; link it from Home
(the book) or the doc guard fails, because a page nobody reaches is lost.
Deleting a page in the wiki does not delete its file (GitHub's wiki event does
not report deletions): remove the file in the repo, and the page goes with it.

Standalone: no ROS, no third-party packages.
"""
from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BOOK = ROOT / '.claude' / 'context' / 'README.md'
NEW_PAGES = ROOT / '.claude' / 'context' / 'wiki'
HOME = NEW_PAGES / 'Home.md'
REPO_URL = 'https://github.com/fh1m/mongla_ws'
SITE_URL = 'https://fh1m.github.io/mongla_ws/'
BLOB = f'{REPO_URL}/blob/main'
MANIFEST = '.mongla-pages.json'

# A markdown link or image: [text](target) / ![alt](target). The target stops at
# whitespace or ')' -- the docs use no titles or spaces inside link targets.
_LINK = re.compile(r'(!?\[[^\]\n]*\]\()([^)\s]+)(\))')
_FENCE = re.compile(r'^\s*(```|~~~)')
_IMAGE = {'.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp'}

# Generated text is fenced by this, so `pull` can take it off again exactly.
_FOOT = '\n\n<!-- wiki:generated -->\n'
# The site's pages open with Jekyll front matter, which the wiki would print as
# text. It is taken off for the wiki and put back from the file on `pull`.
_FRONT_MATTER = re.compile(r'\A---\n.*?\n---\n\n?', re.S)
# A block only the site can run (the capability map's live tally is JavaScript)
# is fenced in the file; the wiki gets a hidden marker in its place, and `pull`
# puts the block back from the file.
_SITE_ONLY = re.compile(r'^<!-- site-only:.*?^<!-- /site-only -->\n', re.S | re.M)
_SITE_MARK = '<!-- site-only block {}: kept in the repository file -->\n'


# --------------------------------------------------------------------------- #
#  Which documents are pages, and what each page is called                     #
# --------------------------------------------------------------------------- #
def _title(path: Path) -> str:
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('# '):
            return line[2:].strip()
    return path.stem


def _slug(title: str) -> str:
    """A wiki page name from a title. The wiki shows its dashes as spaces.

    'SROT board integration — how the host drives Hengla' -> 'SROT-board-integration':
    the part before the first dash or colon, when it can stand alone. A bare label
    ('PR K', 'Ask M', 'Pool') says nothing, so those keep the rest of the title;
    the name stops at a word, by 50 characters.
    """
    clean = title.replace('`', '')
    head = re.split(r'\s+[—–-]+\s+|:\s', clean, maxsplit=1)[0]
    bare = len(head) < 6 or re.fullmatch(r'\w+ \w{1,2}', head)
    short = clean if bare else head
    # No parentheses (a link target ends at ')'), no commas (they read as noise).
    short = re.sub(r'[^\w +.&\'-]', ' ', short)
    words, out = short.split(), ''
    for w in words:
        if len(out) + len(w) + 1 > 50 and out:
            break
        out = f'{out}-{w}' if out else w
    return re.sub(r'-+', '-', out).strip('-.')


def _path_slug(path: Path) -> str:
    parts = list(path.relative_to(ROOT).with_suffix('').parts)
    if parts[:2] == ['.claude', 'context']:
        parts = parts[2:]
    if parts and parts[-1] == 'README':
        parts = parts[:-1]
    return '-'.join(parts)


def _publishable(path: Path) -> bool:
    """The book, and the site's long-form docs; no README that sits beside code."""
    rel = path.relative_to(ROOT).as_posix()
    return rel.startswith('.claude/context/') or (rel.startswith('docs/') and rel.count('/') == 1)


def _reached() -> list[Path]:
    """Every markdown file the book reaches by following links, book first."""
    seen, todo = [], [BOOK, HOME]
    while todo:
        path = todo.pop(0)
        if path in seen:
            continue
        seen.append(path)
        for hit in _linked_files(path):
            if hit and hit.suffix == '.md' and hit.is_file() and _publishable(hit):
                todo.append(hit)
    return seen


def _stub_target(path: Path, reached: list[Path]) -> Path | None:
    """`.claude/context/the-shift.md` only says "it lives at docs/the-shift.md".

    Such a stub is not a page of its own: links to it go to the docs copy."""
    twin = ROOT / 'docs' / path.name
    if path.parent == BOOK.parent and twin in reached:
        return twin
    return None


def pages() -> dict[Path, str]:
    """{file: wiki page name} for every page, stubs aliased to the page they name."""
    reached = _reached()
    names: dict[Path, str] = {}
    taken: dict[str, Path] = {}
    for path in reached:
        if _stub_target(path, reached):
            continue
        name = 'Home' if path == HOME else _slug(_title(path))
        if path != HOME and (not name or name in taken
                             or name.lower() in ('home', '_sidebar', '_footer')):
            name = _path_slug(path)          # two "README"-titled pages, say
        if name in taken:
            raise SystemExit(f'two documents map to the wiki page {name!r}: '
                             f'{taken[name]} and {path}')
        names[path], taken[name] = name, path
    for path in reached:
        twin = _stub_target(path, reached)
        if twin:
            names[path] = names[twin]
    return names


def _is_stub(path: Path, names: dict[Path, str]) -> bool:
    return any(other != path and n == names[path] and other.parent.name == 'docs'
               for other, n in names.items())


# --------------------------------------------------------------------------- #
#  Links                                                                       #
# --------------------------------------------------------------------------- #
def _outside_fences(text: str):
    """(line, is_outside_a_code_fence) for every line."""
    fenced = False
    for line in text.splitlines(keepends=True):
        if _FENCE.match(line):
            fenced = not fenced
            yield line, False
            continue
        yield line, not fenced


def _linked_files(path: Path):
    for line, live in _outside_fences(path.read_text(encoding='utf-8')):
        if live:
            for m in _LINK.finditer(line):
                yield _resolve(path, m.group(2))[0]


def _resolve(src: Path, target: str) -> tuple[Path | None, str]:
    """The repo file a link points at, and its anchor. (None, ...) if not a repo file."""
    if re.match(r'^[a-z][a-z0-9+.-]*:', target) or target.startswith('#'):
        return None, ''
    file, _, anchor = target.partition('#')
    if not file:
        return None, anchor
    hit = Path(os.path.normpath(src.parent / file))
    try:
        hit.relative_to(ROOT)
    except ValueError:
        return None, anchor
    return hit, anchor


def _map_links(text: str, fn) -> str:
    out = []
    for line, live in _outside_fences(text):
        out.append(_LINK.sub(lambda m: m.group(1) + fn(m.group(2)) + m.group(3), line)
                   if live else line)
    return ''.join(out)


def _site_page(src: Path) -> bool:
    """`docs/*.md` is also served by the site, so its links are written for the
    site: `capability-map.html`, and absolute GitHub URLs for everything else."""
    return src.parent == ROOT / 'docs'


def _to_wiki(src: Path, target: str, names: dict[Path, str]) -> str:
    if target.startswith(BLOB + '/'):
        file, _, anchor = target[len(BLOB) + 1:].partition('#')
        hit = ROOT / file
        return names[hit] + (f'#{anchor}' if anchor else '') if hit in names else target
    hit, anchor = _resolve(src, target)
    if hit is None:
        return target
    tail = f'#{anchor}' if anchor else ''
    if _site_page(src) and hit.suffix == '.html' and hit.with_suffix('.md') in names:
        hit = hit.with_suffix('.md')
    if hit in names:
        return names[hit] + tail
    if _site_page(src):
        return SITE_URL + hit.relative_to(ROOT / 'docs').as_posix() + tail
    rel = hit.relative_to(ROOT).as_posix() + ('/' if target.split('#')[0].endswith('/') else '')
    if hit.suffix.lower() in _IMAGE:
        return f'{BLOB}/{rel}?raw=true'
    return f'{BLOB}/{rel}{tail}'


def _from_wiki(src: Path, target: str, by_name: dict[str, Path]) -> str:
    file, _, anchor = target.partition('#')
    tail = f'#{anchor}' if anchor else ''
    if file and file in by_name:
        dest = by_name[file]
        if not _site_page(src):
            return os.path.relpath(dest, src.parent) + tail
        if dest.parent == ROOT / 'docs':
            return dest.with_suffix('.html').name + tail
        return f'{BLOB}/{dest.relative_to(ROOT).as_posix()}{tail}'
    if _site_page(src) and file.startswith(SITE_URL):
        return file[len(SITE_URL):] + tail
    if not _site_page(src) and file.startswith(BLOB + '/'):
        rel = file[len(BLOB) + 1:].removesuffix('?raw=true')
        trail = '/' if rel.endswith('/') else ''
        return os.path.relpath(ROOT / rel, src.parent) + trail + tail
    return target


# --------------------------------------------------------------------------- #
#  The two directions                                                          #
# --------------------------------------------------------------------------- #
def render(path: Path, names: dict[Path, str]) -> str:
    """repo file -> wiki page."""
    text = _FRONT_MATTER.sub('', path.read_text(encoding='utf-8'), count=1)
    n = iter(range(1_000))
    text = _SITE_ONLY.sub(lambda m: _SITE_MARK.format(next(n)), text)
    body = _map_links(text, lambda t: _to_wiki(path, t, names))
    rel = path.relative_to(ROOT).as_posix()
    foot = (f'{_FOOT}---\n*Edit this page here in the wiki: the edit is committed to '
            f'[`{rel}`]({BLOB}/{rel}), where the doc tests check it.*\n')
    return body + foot


def unrender(page: str, path: Path, names: dict[Path, str]) -> str:
    """wiki page -> repo file: the inverse of `render`."""
    page = page.split(_FOOT, 1)[0]
    if not page.endswith('\n'):
        page += '\n'
    by_name = {n: p for p, n in names.items() if not _is_stub(p, names)}
    # A link the file already has keeps the exact text it was written with, so
    # an edit to one paragraph does not rewrite every link on the page.
    known, front = {}, ''
    if path.is_file():
        old = path.read_text(encoding='utf-8')
        _map_links(old, lambda t: known.setdefault(_to_wiki(path, t, names), t))
        hit = _FRONT_MATTER.match(old)
        front = hit.group(0) if hit else ''
        blocks = _SITE_ONLY.findall(old)
        page = re.sub(r'^' + re.escape(_SITE_MARK).replace(r'\{\}', r'(\d+)'),
                      lambda m: blocks[int(m.group(1))], page, flags=re.M)
    return front + _map_links(page, lambda t: known.get(t) or _from_wiki(path, t, by_name))


def sidebar(names: dict[Path, str]) -> str:
    """The book, part by part, each part folded so the sidebar stays short."""
    out = [f'### [Mongla](Home) · [the book]({names[BOOK]})', '']
    open_part = False
    for line in BOOK.read_text(encoding='utf-8').splitlines():
        if line.startswith('## '):
            if open_part:
                out += ['', '</details>', '']
            out += [f'<details><summary><b>{line[3:].strip()}</b></summary>', '']
            open_part = True
        elif re.match(r'^\d+\. \[', line) and open_part:
            out.append(_map_links(re.sub(r'^\d+\. ', '- ', line),
                                  lambda t: _to_wiki(BOOK, t, names)))
    if open_part:
        out += ['', '</details>']
    return '\n'.join(out) + '\n'


def footer(names: dict[Path, str]) -> str:
    ctx = BOOK.parent
    return (f'[Home](Home) · [The book]({names[BOOK]}) · [Code]({REPO_URL}) · [Site]({SITE_URL}) · '
            f'[Defects]({names[ctx / "BUGS.md"]}) · [Roadmap]({names[ctx / "ROADMAP.md"]}) · '
            f'Pages are edited here and committed to the repository.\n')


def build(out: Path) -> int:
    names = pages()
    out.mkdir(parents=True, exist_ok=True)
    manifest = {}
    for path, name in names.items():
        if _is_stub(path, names):
            continue
        (out / f'{name}.md').write_text(render(path, names), encoding='utf-8')
        manifest[name] = path.relative_to(ROOT).as_posix()
    (out / '_Sidebar.md').write_text(sidebar(names), encoding='utf-8')
    (out / '_Footer.md').write_text(footer(names), encoding='utf-8')
    (out / MANIFEST).write_text(json.dumps(manifest, indent=1, sort_keys=True) + '\n',
                                encoding='utf-8')
    print(f'{len(manifest)} pages -> {out}')
    return 0


def pull(wiki: Path) -> list[str]:
    """Write every wiki page back to its file. Returns the files that changed."""
    names = pages()
    # The manifest says which file each page was built from. A wiki that was
    # never built (its first page made by hand) has none: use the repo's names.
    manifest = {n: p.relative_to(ROOT).as_posix() for p, n in names.items()
                if not _is_stub(p, names)}
    if (wiki / MANIFEST).is_file():
        manifest.update(json.loads((wiki / MANIFEST).read_text(encoding='utf-8')))
    changed = []
    for page in sorted(wiki.glob('*.md')):
        name = page.stem
        if name in ('_Sidebar', '_Footer'):
            continue                          # generated from the book, never edited
        rel = manifest.get(name)
        path = ROOT / rel if rel else NEW_PAGES / f'{name}.md'
        names.setdefault(path, name)
        text = unrender(page.read_text(encoding='utf-8'), path, names)
        if not path.is_file() or path.read_text(encoding='utf-8') != text:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding='utf-8')
            changed.append(path.relative_to(ROOT).as_posix())
    return changed


def main(argv: list[str]) -> int:
    if len(argv) != 2 or argv[0] not in ('build', 'pull'):
        print(__doc__, file=sys.stderr)
        return 2
    if argv[0] == 'build':
        return build(Path(argv[1]))
    for rel in pull(Path(argv[1])):
        print(rel)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
