#!/usr/bin/env python3
"""Open Batighor in the Lichtblick desktop app: Kutum's panels, look and views, connected.

The browser gets Batighor from Kutum on every load. The desktop app reads
nothing from Kutum by itself: it loads extensions from
`~/.lichtblick-suite/extensions/<publisher>.<name>-<version>/` and opens a
layout from a URL. So this, on the operator's machine:

  1. downloads every panel Kutum serves and unpacks the versions it does not
     have; an older version of one of ours moves to ~/attic/<date>/, never
     deleted;
  2. writes every view Kutum serves into `~/.lichtblick-suite/layouts/`, so
     the app's layout menu offers all of them, as the browser's does;
  3. opens the app connected to Kutum's bridge, on one of Kutum's views,
     which carries the hull's URL on the host named here.

The app imports a layout file once per name and keeps its own copy, so a
view changed on Kutum reaches the menu only as a new name; the view this
opens is always fetched fresh.

    python3 tools/pool/batighor_desktop.py                    # mongla.local, the cockpit
    python3 tools/pool/batighor_desktop.py 192.168.0.103 --view localization
    python3 tools/pool/batighor_desktop.py --list             # the views Kutum serves

Standard library only: the operator's machine needs no ROS.
"""
from __future__ import annotations

import argparse
import base64
import datetime
import io
import json
import re
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path
from urllib.parse import quote

EXTENSIONS = Path.home() / '.lichtblick-suite' / 'extensions'
LAYOUTS = Path.home() / '.lichtblick-suite' / 'layouts'
OURS = 'mongla.batighor-'          # panels.py: publisher `mongla`, name `batighor-<panel>`


def get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=10) as r:
        return r.read()


def install(base: str, into: Path = EXTENSIONS) -> list[str]:
    """Unpack every extension Kutum serves; returns the directories it wrote."""
    listed = json.loads(get(f'{base}/api/workspaces/mongla/extensions'))['data']
    into.mkdir(parents=True, exist_ok=True)
    attic = Path.home() / 'attic' / datetime.date.today().isoformat() / 'lichtblick-extensions'
    wrote = []
    for e in listed:
        z = zipfile.ZipFile(io.BytesIO(get(f'{base}/api/extensions/{quote(e["id"])}/download')))
        pkg = json.loads(z.read('package.json'))
        stem = f'{pkg["publisher"].lower()}.{pkg["name"]}'
        if not stem.startswith(OURS):
            raise SystemExit(f'ERROR: {stem} is not a Batighor panel; refusing to install it')
        target = into / f'{stem}-{pkg["version"]}'
        for old in into.glob(f'{stem}-*'):
            if old != target:
                attic.mkdir(parents=True, exist_ok=True)
                shutil.move(str(old), attic / old.name)
        if target.is_dir():
            continue
        for name in ('package.json', 'dist/extension.js'):
            data = z.read(name)
            if name.endswith('.js'):
                data = embed_fonts(data, base)
            (target / name).parent.mkdir(parents=True, exist_ok=True)
            (target / name).write_bytes(data)
        wrote.append(target.name)
    return wrote


def install_views(base: str, views: list[str], into: Path = LAYOUTS) -> list[str]:
    """Each view as Kutum serves it for this host (the hull's URL absolute)."""
    into.mkdir(parents=True, exist_ok=True)
    attic = Path.home() / 'attic' / datetime.date.today().isoformat() / 'lichtblick-layouts'
    wrote = []
    for name in views:
        data = get(f'{base}/layouts/{quote(name)}.json')
        json.loads(data)                       # a page that is not a layout stops here
        target = into / f'{name}.json'
        if target.is_file() and target.read_bytes() == data:
            continue
        if target.is_file():
            attic.mkdir(parents=True, exist_ok=True)
            shutil.move(str(target), attic / target.name)
        target.write_bytes(data)
        wrote.append(name)
    return wrote


def embed_fonts(code: bytes, base: str) -> bytes:
    """The theme's fonts as data: URIs in the panel's code.

    On Kutum they are root paths; the desktop app's page is a file and its
    policy is `font-src 'self' data:`, which blocks a font from Kutum.
    """
    def inline(m: re.Match) -> bytes:
        font = get(f'{base}/theme/fonts/{m.group(1).decode()}')
        return b'data:font/woff2;base64,' + base64.b64encode(font)
    return re.sub(rb'/theme/fonts/([a-z0-9-]+\.woff2)', inline, code)


def link(host: str, port: int, bridge: int, view: str) -> str:
    ds = quote(f'ws://{host}:{bridge}', safe='')
    layout = quote(f'http://{host}:{port}/layouts/{view}.json', safe='')
    return f'lichtblick://open?ds=foxglove-websocket&ds.url={ds}&layoutUrl={layout}'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Open Batighor in the Lichtblick desktop app.')
    ap.add_argument('host', nargs='?', default='mongla.local', help="Kutum's name or address")
    ap.add_argument('--port', type=int, default=8000, help="Batighor's web port")
    ap.add_argument('--bridge', type=int, default=8765, help="foxglove_bridge's port")
    ap.add_argument('--view', default='mongla_layout', help='the layout to open')
    ap.add_argument('--list', action='store_true', help="print Kutum's views and exit")
    ap.add_argument('--no-launch', action='store_true', help='install the panels only')
    a = ap.parse_args(argv)
    base = f'http://{a.host}:{a.port}'
    try:
        listed = json.loads(get(f'{base}/api/workspaces/mongla/layouts'))['data']
        views = [v['name'] for v in listed]
    except OSError as exc:
        print(f'ERROR: Batighor is not answering at {base}: {exc}', file=sys.stderr)
        return 1
    if a.list:
        print('\n'.join(views))
        return 0
    if a.view not in views:
        print(f'ERROR: Kutum has no view {a.view!r}; it has: {", ".join(views)}', file=sys.stderr)
        return 1
    wrote = install(base)
    print(f'ok  panels: {", ".join(wrote) if wrote else "already current"}')
    laid = install_views(base, views)
    print(f'ok  views: {", ".join(laid) if laid else "already current"}')
    if a.no_launch:
        return 0
    app = shutil.which('lichtblick')
    if app is None:
        print('ERROR: the lichtblick desktop app is not on PATH', file=sys.stderr)
        return 1
    url = link(a.host, a.port, a.bridge, a.view)
    subprocess.Popen([app, url], start_new_session=True,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'ok  opened {a.view} on {a.host}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
