"""The vehicle's own files, read only: the missions and the deck.

An operator planning a run needs to see the mission it will run and the deck
it will start with (bringup.yaml, the course, the calibration) without a
shell on Kutum. Batighor serves them as text, and nothing else:

* only the shelves named here, each a directory on Kutum;
* a path is resolved and must stay inside its shelf, so `..` and a symlink
  out of it are refused;
* no name starting with `.` anywhere in the path (keys, caches, editors'
  swap files), only text suffixes, and at most MAX_BYTES a file;
* nothing is ever written: the server refuses every method but GET.

Batighor binds every interface with no login (owner ruling, bead
batighor_8 F2), so what a shelf holds is visible to the network it is on.
"""
from __future__ import annotations

import os
from pathlib import Path

def _missions() -> Path:
    """The directory the mission runner loads from (MONGLA_MISSIONS_DIR, else
    the workspace's source tree), asked of the runner itself: `~/missions` is
    only mirrored into it by build_mongla.sh, and on Kutum it was empty while
    the runner held every mission (2026-11-25)."""
    try:
        from mongla_planner.missions import _find_src_missions_dir
        found = _find_src_missions_dir()
        if found is not None:
            return found
        import mongla_planner.missions as m
        return Path(m.__file__).parent
    except ImportError:
        return Path.home() / 'missions'


SHELVES = {
    'missions': _missions(),
    'deck': Path.home() / 'mongla' / 'deck',
}
TEXT = {'.py', '.yaml', '.yml', '.json', '.md', '.txt', '.toml', '.cfg', '.csv', '.sh'}
MAX_BYTES = 256 * 1024
MAX_FILES = 2000
MAX_DEPTH = 5


def _visible(rel: Path) -> bool:
    return bool(rel.parts) and not any(p.startswith('.') for p in rel.parts) \
        and rel.suffix.lower() in TEXT


def listing(shelves: dict | None = None) -> dict:
    """{shelf: [{path, size, mtime}]}, newest first; a missing shelf is []."""
    out = {}
    for name, root in (shelves or SHELVES).items():
        files = []
        if root.is_dir():
            base = root.resolve()
            for dirpath, dirs, names in os.walk(base):
                depth = len(Path(dirpath).relative_to(base).parts)
                dirs[:] = [] if depth >= MAX_DEPTH else sorted(d for d in dirs if not d.startswith('.'))
                for n in names:
                    p = Path(dirpath) / n
                    rel = p.relative_to(base)
                    if not _visible(rel) or not p.is_file() or len(files) >= MAX_FILES:
                        continue
                    if base not in p.resolve().parents:       # a link out of the shelf
                        continue
                    st = p.stat()
                    files.append({'path': rel.as_posix(), 'size': st.st_size, 'mtime': st.st_mtime})
        out[name] = sorted(files, key=lambda f: -f['mtime'])
    return out


def read(shelf: str, rel: str, shelves: dict | None = None) -> tuple[bytes | None, str]:
    """(text, '') or (None, why). Every refusal names its reason."""
    root = (shelves or SHELVES).get(shelf)
    if root is None:
        return None, f'no shelf {shelf!r}'
    if not root.is_dir():
        return None, f'{shelf} is not on this machine ({root})'
    base = root.resolve()
    want = (base / rel).resolve()
    if base != want and base not in want.parents:
        return None, 'outside the shelf'
    inside = want.relative_to(base)
    if not _visible(inside):
        return None, 'not a text file this viewer shows'
    if not want.is_file():
        return None, 'no such file'
    if want.stat().st_size > MAX_BYTES:
        return None, f'larger than {MAX_BYTES // 1024} KB'
    return want.read_bytes(), ''
