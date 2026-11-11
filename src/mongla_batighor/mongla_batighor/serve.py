"""Serve Lichtblick and our layouts to every operator's browser.

    ros2 run mongla_batighor serve --port 8000 --bridge-port 8765

`http://mongla.local:8000/` answers with the app already connected to this
machine's foxglove_bridge and opened on a layout (`/?layout=board_health` for
another one). The address in the link is the one the browser used, so the
same link works over the tether and over Wi-Fi.

Static files only: the app renders in the browser, so a second operator costs
Kutum the bytes and nothing else. Files whose name carries a content hash are
cached for good; `index.html` is never cached, so a new build is seen at once.

`/api/` answers the part of Lichtblick's workspace API it reads, so our panels
(`panels/`, see `panels.py`) load in every browser with nothing installed. It
is read only: a layout or extension saved from a browser stays in that browser.
"""
from __future__ import annotations

import argparse
import gzip
import json
import mimetypes
import re
import sys
from datetime import datetime, timezone
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlsplit

from mongla_batighor import panels
from mongla_batighor.pin import pin, share, web_dir

_NAME = re.compile(r'^[A-Za-z0-9_-]+$')
_HASHED = re.compile(r'\.[0-9a-f]{16,}\.')
mimetypes.add_type('application/wasm', '.wasm')


def app_link(host: str, bridge_port: int, layout: str) -> str:
    """Where `/` sends a browser: the app, its data source and our layout.

    `host` is the Host header, port included. The layout is named, not sent:
    the app reads every layout from `/api/` as the workspace's, and `layout=`
    selects one by name during its own start-up. A `layoutUrl` raced that
    start-up: its copy replaced the one already selected, the app fell back to
    the first layout, and a second visit opened the wrong one.
    """
    name = host.rsplit(':', 1)[0] if host.count(':') == 1 else host
    ws = f'ws://{name}:{bridge_port}'
    return (f'/app/?ds=foxglove-websocket&ds.url={quote(ws, safe="")}'
            f'&workspace={panels.WORKSPACE}&layout={quote(layout, safe="")}')


class Handler(BaseHTTPRequestHandler):
    server_version = 'batighor'

    def __init__(self, *a, web: Path, layouts: Path, bridge_port: int,
                 default_layout: str, exts: dict, bundle, **kw):
        self.web, self.layouts = web, layouts
        self.exts, self.bundle = exts, bundle      # see make_server
        self.bridge_port, self.default_layout = bridge_port, default_layout
        super().__init__(*a, **kw)

    def log_message(self, fmt, *args):                     # quiet per request
        pass

    def do_GET(self):                                       # noqa: N802
        url = urlsplit(self.path)
        if url.path == '/':
            layout = parse_qs(url.query).get('layout', [self.default_layout])[0]
            if not _NAME.match(layout) or not (self.layouts / f'{layout}.json').exists():
                return self._text(HTTPStatus.NOT_FOUND, f'no layout {layout!r}\n')
            self.send_response(HTTPStatus.FOUND)
            self.send_header('Location', app_link(self.headers.get('Host', 'localhost'),
                                                  self.bridge_port, layout))
            self.end_headers()
            return
        if url.path.startswith('/layouts/'):
            name = url.path[len('/layouts/'):].removesuffix('.json')
            if not _NAME.match(name):
                return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
            return self._file(self.layouts / f'{name}.json', cache=False)
        if url.path.startswith('/api/'):
            return self._api(url.path[len('/api/'):])
        if url.path == '/app' or url.path.startswith('/app/'):
            if not (self.web / 'index.html').exists():
                return self._text(HTTPStatus.SERVICE_UNAVAILABLE,
                                  f'Lichtblick is not installed at {self.web}.\n'
                                  # The newline on its own: the command
                                  # linter reads this line as an operator's.
                                  'Run: ros2 run mongla_batighor fetch'
                                  '\n')
            rel = url.path[len('/app/'):] or 'index.html'
            if self.bundle and rel == 'index.html':
                html = (self.web / 'index.html').read_bytes()
                return self._bytes(html.replace(self.bundle[0].encode(), self.bundle[1].encode()),
                                   'text/html', cache=False)
            if self.bundle and rel == self.bundle[1]:
                return self._bytes(self.bundle[2], 'text/javascript', cache=True,
                                   gz=self.bundle[3])
            target = (self.web / rel).resolve()
            if self.web.resolve() not in target.parents and target != self.web.resolve():
                return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
            return self._file(target, cache=bool(_HASHED.search(target.name)))
        return self._text(HTTPStatus.NOT_FOUND, 'not found\n')

    def _api(self, rel: str):
        """Lichtblick's workspace API, read only: our extensions and no layouts.

        Its client unwraps `{data, timestamp, path}`; a download is the bare
        .foxe. Shapes: lichtblick 1.30.0 `api/extensions/ExtensionsAPI.ts`.
        """
        ws = f'workspaces/{panels.WORKSPACE}/'
        if rel == ws + 'extensions':
            return self._json(rel, [e for e, _ in self.exts.values()])
        if rel == ws + 'layouts':
            return self._json(rel, self._org_layouts())
        if rel == 'profile/favorite-layouts':           # api/layouts/LayoutFavoritesAPI.ts
            return self._json(rel, {'favoriteLayouts': []})
        parts = rel.split('/')
        if parts[0] == 'extensions' and len(parts) in (2, 3) and parts[1] in self.exts:
            entry, data = self.exts[parts[1]]
            if len(parts) == 2:
                return self._json(rel, entry)
            if parts[2] == 'download':
                return self._bytes(data, 'application/zip', cache=False)
        return self._text(HTTPStatus.NOT_FOUND, 'not found\n')

    def _org_layouts(self) -> list:
        """Our layouts as the organisation's: every browser gets Kutum's copy.

        Shape: lichtblick 1.30.0 `api/layouts/types.ts` `LayoutApiData`. The
        date is the file's, so an edited layout reaches a browser that has
        the old one.
        """
        out = []
        for f in sorted(self.layouts.glob('*.json')):
            when = datetime.fromtimestamp(f.stat().st_mtime, timezone.utc).isoformat()
            out.append({'id': f.stem, 'layoutId': f.stem, 'name': f.stem,
                        'data': json.loads(f.read_text()), 'workspace': panels.WORKSPACE,
                        'permission': 'ORG_READ', 'from': 'kutum',
                        'createdAt': when, 'updatedAt': when})
        return out

    def _refuse(self):
        self._text(HTTPStatus.FORBIDDEN, 'Kutum serves layouts and panels read only\n')

    do_POST = do_PUT = do_DELETE = _refuse                  # noqa: N815

    def _json(self, path: str, data):
        body = json.dumps({'data': data, 'path': path,
                           'timestamp': '1970-01-01T00:00:00Z'}).encode()
        return self._bytes(body, 'application/json', cache=False)

    def _file(self, path: Path, cache: bool):
        if not path.is_file():
            return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
        gz = Path(str(path) + '.gz')
        return self._bytes(path, mimetypes.guess_type(path.name)[0]
                           or 'application/octet-stream', cache,
                           gz=gz if gz.is_file() else None)

    def _bytes(self, body, ctype: str, cache: bool, gz=None):
        """Send `body`, or `gz` to a browser that takes gzip; each is bytes or a file."""
        use_gz = gz is not None and 'gzip' in self.headers.get('Accept-Encoding', '')
        body = gz if use_gz else body
        if isinstance(body, Path):
            body = body.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', ctype)
        if use_gz:
            self.send_header('Content-Encoding', 'gzip')
        self.send_header('Vary', 'Accept-Encoding')
        self.send_header('Cache-Control', 'public, max-age=31536000, immutable'
                         if cache else 'no-cache')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, status, text: str):
        body = text.encode()
        self.send_response(status)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def make_server(port: int, web: Path, layouts: Path, bridge_port: int,
                default_layout: str = 'mongla_layout',
                panels_dir: Path | None = None) -> ThreadingHTTPServer:
    """The server; built once, so a panel or the bundle is read at start.

    A Lichtblick whose API switch has moved is served without our panels, and
    says so: the views still work, and the deck still drives.
    """
    exts = panels.extensions(panels_dir) if panels_dir and panels_dir.is_dir() else {}
    try:
        bundle = panels.api_bundle(web)
    except ValueError as exc:
        print(f'[batighor] WARNING: no panels from Kutum: {exc}', file=sys.stderr, flush=True)
        bundle = None
    if bundle:
        bundle = (*bundle, gzip.compress(bundle[2], compresslevel=9))
    return ThreadingHTTPServer(('0.0.0.0', port), partial(
        Handler, web=web, layouts=layouts, bridge_port=bridge_port,
        default_layout=default_layout, exts=exts, bundle=bundle))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--bridge-port', type=int, default=8765)
    ap.add_argument('--layout', default='mongla_layout')
    a, _ = ap.parse_known_args(argv)          # launch adds --ros-args
    root = share()
    web = web_dir(pin(root)['version'])
    srv = make_server(a.port, web, root / 'layouts', a.bridge_port, a.layout,
                      panels_dir=root / 'panels')
    state = 'ok' if (web / 'index.html').exists() else \
        'WARNING: not installed, run `ros2 run mongla_batighor fetch`'
    print(f'[batighor] http://0.0.0.0:{a.port}/  Lichtblick at {web}: {state}', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
