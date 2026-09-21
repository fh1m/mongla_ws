#!/usr/bin/env python3
"""The README's figures, captured from the site's own components.

The README had too much prose and too few pictures. The site already draws the
most informative figures this project has -- every one of them from real
measurements -- so rather than redraw them (and have two copies drift), this
tool photographs them: it serves docs/, drives headless Chrome over the
DevTools protocol, and captures each component by CSS selector at 2x.

Reduced motion is emulated so every animation is at its resting pose and every
scroll-reveal is already revealed; that is also the honest still frame, since
it is the one the site shows readers who ask for no motion.

    python3 tools/readme_figures.py            # writes docs/imgs/readme/*.webp
"""
from __future__ import annotations

import asyncio
import base64
import http.server
import io
import json
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from functools import partial
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / 'docs'
OUT = DOCS / 'imgs' / 'readme'
WIDTH = 1500

# (file stem, css selector, js to run first so the figure shows the right state)
FIGURES = [
    ('numbers',     '#surface .bento', ''),
    ('wire-frame',  '.wire', ''),
    ('stop-chip',   '#frame .stop[data-n="03"]', ''),
    ('stop-lock',   '#frame .stop[data-n="05"]', ''),
    ('colour-loss', '.colour', ''),
    ('lock-ladder', '.rungs2', ''),
    ('ledger',      '.core', ''),
    ('retractions', '.retracts', ''),
    ('cores',       '.cores', ''),
    ('pid-lab',     '#pid .lab',
     "document.querySelectorAll('[data-pid]').forEach(b => "
     "{ if (b.textContent.trim() === 'add I') b.click(); });"),
    ('flow-lab',    '#flow .lab',
     "document.querySelectorAll('[data-flow]').forEach(b => "
     "{ if (b.textContent.trim() === 'height off by 20 %') b.click(); });"),
    ('blockers',    '.board', ''),
]


def free_port() -> int:
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def serve(port: int) -> http.server.ThreadingHTTPServer:
    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *_a, **_k):
            pass
    httpd = http.server.ThreadingHTTPServer(('127.0.0.1', port),
                                            partial(Quiet, directory=str(DOCS)))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


async def capture(ws_url: str, page_url: str) -> dict[str, bytes]:
    import websockets
    out: dict[str, bytes] = {}
    async with websockets.connect(ws_url, max_size=64 * 1024 * 1024) as ws:
        seq = 0

        async def cmd(method, **params):
            nonlocal seq
            seq += 1
            me = seq
            await ws.send(json.dumps({'id': me, 'method': method, 'params': params}))
            while True:
                msg = json.loads(await ws.recv())
                if msg.get('id') == me:
                    if 'error' in msg:
                        raise RuntimeError(f'{method}: {msg["error"]}')
                    return msg.get('result', {})

        await cmd('Page.enable')
        await cmd('Emulation.setDeviceMetricsOverride', width=WIDTH, height=1000,
                  deviceScaleFactor=2, mobile=False)
        await cmd('Emulation.setEmulatedMedia',
                  features=[{'name': 'prefers-reduced-motion', 'value': 'reduce'}])
        await cmd('Page.navigate', url=page_url)
        await asyncio.sleep(3.0)
        await cmd('Runtime.evaluate', expression='document.fonts.ready', awaitPromise=True)
        # reveal everything the page reveals on scroll
        await cmd('Runtime.evaluate', expression=(
            "document.querySelectorAll('.rise,.pair').forEach(e => e.classList.add('in'));"
            # a stop is drawn at full brightness only while it is the one in view
            "document.querySelectorAll('.stop').forEach(e => e.classList.add('on'));"
            # page chrome that is fixed to the viewport would bleed into every clip
            "document.querySelectorAll('.dock,#grain').forEach(e => e.style.display = 'none');"))

        for stem, selector, prep in FIGURES:
            if prep:
                await cmd('Runtime.evaluate', expression=prep)
            # Chrome cannot rasterise a clip much past ~16 000 px down the page,
            # and this page is far taller than that -- captures there came back
            # shifted and CUT (the wire figure lost its key table). So each
            # figure's own section is shown alone, near the top, and every other
            # section is hidden for the length of the capture.
            ok = (await cmd('Runtime.evaluate', returnByValue=True, expression=(
                f"(() => {{ const e = document.querySelector({json.dumps(selector)});"
                f"  if (!e) return false;"
                f"  const keep = e.closest('section') || e;"
                f"  document.querySelectorAll('section,footer,header').forEach(n => {{"
                f"    n.style.display = (n === keep || n.contains(keep)) ? '' : 'none'; }});"
                # lazy images above the figure load DURING the wait and push it
                # down; that is what cut the wire figure. Load them now.
                f"  keep.querySelectorAll('img[loading=lazy]').forEach(i => i.loading = 'eager');"
                f"  scrollTo(0, 0); return true; }})()"
            )))['result'].get('value')
            if not ok:
                print(f'missing: {selector}', file=sys.stderr)
                continue
            await cmd('Runtime.evaluate', awaitPromise=True, expression=(
                "Promise.all([...document.images].filter(i => !i.complete && i.offsetParent)"
                ".map(i => new Promise(r => { i.onload = i.onerror = r; })))"))
            await asyncio.sleep(0.8)
            # measured LAST, after every image above it has taken its height;
            # a journey stop is drawn at full strength only while it is the one
            # in view, so that is set here too
            rect = (await cmd('Runtime.evaluate', returnByValue=True, expression=(
                f"(() => {{ document.querySelectorAll('.stop').forEach(s => s.classList.add('on'));"
                f"  const r = document.querySelector({json.dumps(selector)}).getBoundingClientRect();"
                f"  return {{x: r.left + scrollX, y: r.top + scrollY, w: r.width, h: r.height}}; }})()"
            )))['result'].get('value')
            await asyncio.sleep(0.3)
            pad = 12
            shot = await cmd('Page.captureScreenshot', format='png', captureBeyondViewport=True,
                             clip={'x': max(0, rect['x'] - pad), 'y': max(0, rect['y'] - pad),
                                   'width': rect['w'] + 2 * pad, 'height': rect['h'] + 2 * pad,
                                   'scale': 1})
            out[stem] = base64.b64decode(shot['data'])
    return out


def main() -> int:
    chrome = shutil.which('google-chrome-stable') or shutil.which('google-chrome')
    if not chrome:
        print('google-chrome not found', file=sys.stderr)
        return 2
    from PIL import Image
    port, dbg = free_port(), free_port()
    httpd = serve(port)
    with tempfile.TemporaryDirectory() as prof:
        proc = subprocess.Popen([chrome, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                                 f'--remote-debugging-port={dbg}', f'--user-data-dir={prof}',
                                 'about:blank'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            ws = None
            for _ in range(50):
                try:
                    pages = json.load(urllib.request.urlopen(f'http://127.0.0.1:{dbg}/json'))
                    ws = next(p['webSocketDebuggerUrl'] for p in pages if p['type'] == 'page')
                    break
                except Exception:
                    time.sleep(0.2)
            if not ws:
                print('chrome did not come up', file=sys.stderr)
                return 2
            shots = asyncio.run(capture(ws, f'http://127.0.0.1:{port}/index.html'))
        finally:
            proc.terminate()
            httpd.shutdown()
    OUT.mkdir(parents=True, exist_ok=True)
    for stem, png in shots.items():
        im = Image.open(io.BytesIO(png)).convert('RGB')
        path = OUT / f'{stem}.webp'
        im.save(path, 'WEBP', quality=88, method=6)
        print(f'{path.relative_to(ROOT)}  {im.size[0]}x{im.size[1]}  {path.stat().st_size // 1024} KB')
    return 0 if len(shots) == len(FIGURES) else 1


if __name__ == '__main__':
    sys.exit(main())
