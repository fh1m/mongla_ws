"""Put the pinned Lichtblick web build on this machine.

    ros2 run mongla_batighor fetch                  # download, check, unpack
    ros2 run mongla_batighor fetch --from FILE.tgz  # a tarball already here

The tarball must match the hash in LICHTBLICK.json, or nothing is written.
Source maps are left out (130 of 173 MB, and only a debugger reads them),
and every text asset gets a gzip copy beside it so a weak link loads about a
quarter of the bytes.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import shutil
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path

from mongla_batighor.pin import HOME, pin, web_dir

_COMPRESS = ('.js', '.wasm', '.css', '.html', '.json', '.svg', '.ttf')


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def unpack(tarball: Path, dest: Path, expected: str) -> int:
    """Check, unpack and compress. Returns the number of files written."""
    got = sha256(tarball)
    if got != expected:
        raise ValueError(f'{tarball}: sha256 {got}, expected {expected}; '
                         'nothing written')
    tmp = Path(tempfile.mkdtemp(prefix='.batighor-', dir=dest.parent))
    n = 0
    try:
        with tarfile.open(tarball) as tar:
            for m in tar.getmembers():
                name = Path(m.name)
                if name.is_absolute() or '..' in name.parts:
                    raise ValueError(f'{tarball}: unsafe path {m.name!r}')
                if not m.isfile() or name.suffix == '.map':
                    continue
                out = tmp / name
                out.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(m) as src, open(out, 'wb') as dst:
                    shutil.copyfileobj(src, dst)
                n += 1
                if name.suffix in _COMPRESS and m.size > 1024:
                    with open(out, 'rb') as src, gzip.open(
                            str(out) + '.gz', 'wb', compresslevel=9) as dst:
                        shutil.copyfileobj(src, dst)
        if not (tmp / 'index.html').exists():
            raise ValueError(f'{tarball}: no index.html; not a web build')
        if dest.exists():
            shutil.rmtree(dest)
        tmp.rename(dest)
    finally:
        if tmp.exists():
            shutil.rmtree(tmp)
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--from', dest='src', type=Path,
                    help='a tarball already on this machine')
    ap.add_argument('--home', type=Path, default=HOME)
    a, _ = ap.parse_known_args(argv)
    p = pin()
    dest = web_dir(p['version'], a.home)
    if (dest / 'index.html').exists() and a.src is None:
        print(f'ok  Lichtblick {p["version"]} already at {dest}')
        return 0
    dest.parent.mkdir(parents=True, exist_ok=True)
    src = a.src
    if src is None:
        src = dest.parent / f'lichtblick-web-{p["version"]}.tar.gz'
        print(f'downloading {p["url"]}')
        urllib.request.urlretrieve(p['url'], src)
    try:
        n = unpack(src, dest, p['sha256'])
    except ValueError as exc:
        print(f'FAIL {exc}', file=sys.stderr)
        return 1
    finally:
        if a.src is None and src.exists():
            src.unlink()
    print(f'ok  Lichtblick {p["version"]}: {n} files at {dest}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
