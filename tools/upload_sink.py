#!/usr/bin/env python3
"""Tiny sink for tools/hull_render_rig.html: accepts "<name>|<dataURL>" POSTs
on 127.0.0.1:8778 and writes each frame as a PNG. A 2800px dataURL is far too
large to hand back through a browser-automation return value, so the rig posts
it here instead. Binds loopback only.
"""
import http.server, base64, pathlib, re
OUT = pathlib.Path('/tmp/claude-1000/renders'); OUT.mkdir(exist_ok=True)
class H(http.server.BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers['Content-Length'])
        body = self.rfile.read(n).decode()
        name, _, data = body.partition('|')
        # A plain file name only: the name arrives in the request body, and a
        # '../' in it would write outside OUT (CodeQL py/path-injection).
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,99}', name) or '..' in name:
            self.send_response(400); self.end_headers(); return
        raw = base64.b64decode(data.split(',',1)[1])
        (OUT/f'{name}.png').write_bytes(raw)
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin','*')
        self.end_headers(); self.wfile.write(b'ok')
        print(f'{name}.png {len(raw)//1024} KB', flush=True)
    def do_OPTIONS(self):
        self.send_response(200)
        for h,v in (('Access-Control-Allow-Origin','*'),('Access-Control-Allow-Headers','*'),('Access-Control-Allow-Methods','POST,OPTIONS')):
            self.send_header(h,v)
        self.end_headers()
    def log_message(self,*a): pass
http.server.HTTPServer(('127.0.0.1',8778),H).serve_forever()
