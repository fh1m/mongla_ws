"""Serve Lichtblick and our layouts to every operator's browser.

    ros2 run mongla_batighor serve --port 8000 --bridge-port 8765

`http://mongla.local:8000/` answers with the app already connected to this
machine's foxglove_bridge and opened on a layout (`/?layout=board_health` for
another one). The address in the link is the one the browser used, so the
same link works over the tether and over Wi-Fi.

Static files only: the app renders in the browser, so a second operator costs
Kutum the bytes and nothing else. Files whose name carries a content hash are
cached for good; `index.html` is never cached, so a new build is seen at once.
"""
from __future__ import annotations

import argparse
import mimetypes
import re
import sys
from functools import partial
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlsplit

from mongla_batighor.pin import pin, share, web_dir

_NAME = re.compile(r'^[A-Za-z0-9_-]+$')
_HASHED = re.compile(r'\.[0-9a-f]{16,}\.')
# The Host header becomes the bridge address the app connects to. Any value but
# a host name or address with an optional port is refused: a forged Host would
# point a browser's helm at another machine's bridge.
_HOST = re.compile(r'^(?:[A-Za-z0-9](?:[A-Za-z0-9.-]{0,251}[A-Za-z0-9])?|\[[0-9A-Fa-f:.]+\])'
                   r'(?::[0-9]{1,5})?$')
mimetypes.add_type('application/wasm', '.wasm')


def app_link(host: str, bridge_port: int, layout: str) -> str:
    """Where `/` sends a browser: the app, its data source and our layout.

    `host` is the Host header, port included. The layout URL must be absolute:
    Lichtblick parses it with `new URL()`, which refuses a path and shows
    "Invalid layout URL" over the default layout.
    """
    name = host.rsplit(':', 1)[0] if host.count(':') == 1 else host
    ws = f'ws://{name}:{bridge_port}'
    return (f'/app/?ds=foxglove-websocket&ds.url={quote(ws, safe="")}'
            f'&layoutUrl={quote(f"http://{host}/layouts/{layout}.json", safe="")}')


class Handler(BaseHTTPRequestHandler):
    server_version = 'batighor'

    def __init__(self, *a, web: Path, layouts: Path, bridge_port: int,
                 default_layout: str, **kw):
        self.web, self.layouts = web, layouts
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
            host = self.headers.get('Host', 'localhost')
            if not _HOST.match(host):
                return self._text(HTTPStatus.BAD_REQUEST, 'bad Host header\n')
            self.send_response(HTTPStatus.FOUND)
            self.send_header('Location', app_link(host, self.bridge_port, layout))
            self.end_headers()
            return
        if url.path.startswith('/layouts/'):
            name = url.path[len('/layouts/'):].removesuffix('.json')
            if not _NAME.match(name):
                return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
            return self._file(self.layouts / f'{name}.json', cache=False)
        if url.path == '/app' or url.path.startswith('/app/'):
            if not (self.web / 'index.html').exists():
                return self._text(HTTPStatus.SERVICE_UNAVAILABLE,
                                  f'Lichtblick is not installed at {self.web}.\n'
                                  # The newline on its own: the command
                                  # linter reads this line as an operator's.
                                  'Run: ros2 run mongla_batighor fetch'
                                  '\n')
            rel = url.path[len('/app/'):] or 'index.html'
            target = (self.web / rel).resolve()
            if self.web.resolve() not in target.parents and target != self.web.resolve():
                return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
            return self._file(target, cache=bool(_HASHED.search(target.name)))
        return self._text(HTTPStatus.NOT_FOUND, 'not found\n')

    def _file(self, path: Path, cache: bool):
        if not path.is_file():
            return self._text(HTTPStatus.NOT_FOUND, 'not found\n')
        gz = Path(str(path) + '.gz')
        use_gz = gz.is_file() and 'gzip' in self.headers.get('Accept-Encoding', '')
        body = (gz if use_gz else path).read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', mimetypes.guess_type(path.name)[0]
                         or 'application/octet-stream')
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
                default_layout: str = 'mongla_layout') -> ThreadingHTTPServer:
    return ThreadingHTTPServer(('0.0.0.0', port), partial(
        Handler, web=web, layouts=layouts, bridge_port=bridge_port,
        default_layout=default_layout))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--bridge-port', type=int, default=8765)
    ap.add_argument('--layout', default='mongla_layout')
    a, _ = ap.parse_known_args(argv)          # launch adds --ros-args
    root = share()
    web = web_dir(pin(root)['version'])
    srv = make_server(a.port, web, root / 'layouts', a.bridge_port, a.layout)
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
