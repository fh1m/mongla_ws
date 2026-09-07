#!/usr/bin/env python3
"""Guided camera calibration in a browser: live view, coverage, tilt spread.

⛔ WHY GUIDED AND NOT JUST "GRAB 25 FRAMES". A calibration capture fails in a
way that LOOKS LIKE SUCCESS. Too little tilt leaves focal length and distance
confounded, and the fit then returns a LOW reprojection RMS with a WRONG
focal length -- that is exactly how the downward camera produced fx = 835.7,
969.9 and 1011.2 on three consecutive attempts, each with a confident-looking
metric (`fov_solve.py`'s own docstring). OpenCV's docs say the same of
printed targets.

So the two things that actually make the system well-conditioned are driven
explicitly, and the operator is told which one is missing:

  * TILT SPREAD -- the board seen from genuinely different angles, not 25
    fronto-parallel views. This is the one that decides whether the answer is
    right, and the one a person naturally under-does.
  * FRAME COVERAGE -- corners and edges as well as the middle, because that
    is where the distortion coefficients get their only evidence.

Both are shown live, as bars that must fill, so "am I done" is answerable
without reading a number afterwards.

It also refuses to auto-capture near-duplicate poses: 25 views of the same
pose is one view with 24 witnesses, and it inflates confidence without adding
information.

⛔ ThreadingHTTPServer, NOT HTTPServer. An MJPEG handler never returns, so on
a single-threaded server it starves every other route -- which silently broke
a second endpoint during the FOV calibration round and looked like a hung
page.

Usage (stop the vision launch first -- it holds the cameras):
    python3 tools/fov_calibrate_web.py --device 3 --out ~/fantech_cal
then open the URL it prints, and run `fov_solve.py` on the folder when the
bars are full. The page prints the exact command.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

FIND = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Coverage is scored on a 3x3 grid of where the board's CENTRE landed, and
# tilt on the board's apparent aspect (a fronto-parallel board is square-ish;
# a tilted one is not). Both are crude on purpose -- they are guidance, and
# `fov_solve` does the real conditioning check afterwards.
GRID = 3
TILT_BINS = (0.00, 0.12, 0.25, 0.40, 1.00)


class State:
    def __init__(self, cols, rows, want, outdir):
        self.lock = threading.Lock()
        self.cols, self.rows, self.want = cols, rows, want
        self.outdir = outdir
        self.frame = None                  # latest annotated JPEG
        self.kept = 0
        self.cover = np.zeros((GRID, GRID), int)
        self.tilt = np.zeros(len(TILT_BINS) - 1, int)
        self.poses = []                    # (cx, cy, tilt) of accepted views
        self.msg = 'starting...'
        self.found = False
        self.paused = False
        self.err = None
        self.solve = 'idle'        # idle | running | done | failed
        self.solve_out = ''


def tilt_of(corners, cols, rows):
    """Crude foreshortening: how far the board is from fronto-parallel.

    Compares the two diagonals of the detected quad. A square-on board has
    equal diagonals; tilt makes them differ. Scale-free, so it does not care
    how far away the board is -- which is what we want, since distance is a
    different axis of variety.
    """
    c = corners.reshape(-1, 2)
    tl, tr = c[0], c[cols - 1]
    bl, br = c[-cols], c[-1]
    d1 = np.linalg.norm(br - tl)
    d2 = np.linalg.norm(bl - tr)
    if max(d1, d2) < 1e-6:
        return 0.0
    return float(abs(d1 - d2) / max(d1, d2))


def novel(poses, cx, cy, t, w, h):
    """Reject a pose too close to one already captured.

    25 views of the same pose is ONE view with 24 witnesses: it lowers the
    apparent RMS and adds no information, which is the failure this whole
    file exists to prevent.
    """
    for (px, py, pt) in poses:
        near = (abs(px - cx) < 0.12 * w and abs(py - cy) < 0.12 * h
                and abs(pt - t) < 0.06)
        if near:
            return False
    return True


def capture_loop(st, dev, width, height):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        with st.lock:
            st.err = (f'cannot open camera {dev}. The vision launch holds the '
                      f'cameras -- stop it first (Ctrl-C in its terminal).')
        return
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    last = 0.0
    while True:
        ok, f = cap.read()
        if not ok:
            time.sleep(0.02)
            continue
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (st.cols, st.rows), FIND)
        vis = f.copy()
        msg = 'show the board'
        if found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRIT)
            cv2.drawChessboardCorners(vis, (st.cols, st.rows), corners, True)
            c = corners.reshape(-1, 2)
            cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
            t = tilt_of(corners, st.cols, st.rows)
            gx = min(GRID - 1, int(cx / w * GRID))
            gy = min(GRID - 1, int(cy / h * GRID))
            tb = int(np.digitize(t, TILT_BINS) - 1)
            tb = max(0, min(len(st.tilt) - 1, tb))
            with st.lock:
                fresh = novel(st.poses, cx, cy, t, w, h)
                done = st.kept >= st.want
                paused = st.paused
            if done:
                msg = 'enough views -- run the solve command below'
            elif paused:
                msg = 'paused'
            elif not fresh:
                msg = 'too close to a view already taken -- move or tilt more'
            elif time.time() - last < 0.8:
                msg = 'hold still...'
            else:
                fn = os.path.join(st.outdir, f'cal_{st.kept:03d}.png')
                cv2.imwrite(fn, f)
                last = time.time()
                with st.lock:
                    st.kept += 1
                    st.cover[gy, gx] += 1
                    st.tilt[tb] += 1
                    st.poses.append((cx, cy, t))
                msg = f'captured {st.kept}/{st.want}'
        # Draw the coverage grid so the operator sees WHERE to go next.
        with st.lock:
            cover = st.cover.copy()
        for iy in range(GRID):
            for ix in range(GRID):
                x0, y0 = ix * w // GRID, iy * h // GRID
                x1, y1 = (ix + 1) * w // GRID, (iy + 1) * h // GRID
                col = (60, 190, 60) if cover[iy, ix] else (70, 70, 200)
                cv2.rectangle(vis, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), col, 2)
        small = cv2.resize(vis, (960, int(960 * h / w)))
        ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with st.lock:
                st.frame = jpg.tobytes()
                st.msg = msg
                st.found = found


def run_solve(st, argv, dest):
    """Run fov_solve as a subprocess and keep its output for the page.

    A subprocess rather than an import: `fov_solve` is a script with its own
    argv handling, and shelling out is both the leaner call and the same
    command the operator would have typed -- so the page cannot drift from
    the documented path.
    """
    with st.lock:
        st.solve, st.solve_out = 'running', 'solving...'
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=900)
        out = (r.stdout or '') + (r.stderr or '')
        ok = r.returncode == 0 and os.path.exists(dest)
        with st.lock:
            st.solve = 'done' if ok else 'failed'
            st.solve_out = out[-4000:]
    except Exception as exc:
        with st.lock:
            st.solve, st.solve_out = 'failed', f'{exc}'


PAGE = """<!doctype html><meta charset=utf-8>
<title>Camera calibration</title>
<style>
 :root{--bg:#0e1116;--fg:#e6edf3;--dim:#8b949e;--ok:#3fb950;--no:#f85149;
       --line:#21262d;--acc:#58a6ff}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
 header{padding:14px 20px;border-bottom:1px solid var(--line);
        display:flex;gap:18px;align-items:baseline}
 h1{font-size:15px;margin:0;letter-spacing:.08em;text-transform:uppercase}
 .sub{color:var(--dim);font-size:12px}
 main{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:0;
      align-items:start}
 .vid{padding:16px}
 img{width:100%;display:block;border:1px solid var(--line);border-radius:4px}
 aside{padding:16px;border-left:1px solid var(--line);min-height:70vh}
 .msg{padding:10px 12px;border-radius:4px;background:#161b22;
      border-left:3px solid var(--acc);margin-bottom:18px;min-height:42px}
 h2{font-size:11px;letter-spacing:.12em;text-transform:uppercase;
    color:var(--dim);margin:18px 0 8px}
 .bar{height:8px;background:#161b22;border-radius:4px;overflow:hidden;
      margin:6px 0 2px}
 .bar i{display:block;height:100%;background:var(--ok);width:0}
 .row{display:flex;justify-content:space-between;font-size:12px;
      color:var(--dim)}
 .tilt{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:6px}
 .tilt div{background:#161b22;border-radius:3px;padding:6px 0;text-align:center;
           font-size:11px;color:var(--dim)}
 .tilt div.on{background:#132d1a;color:var(--ok)}
 code{display:block;background:#161b22;padding:10px;border-radius:4px;
      font-size:12px;color:var(--acc);margin-top:8px;word-break:break-all}
 .warn{color:var(--no)}
 button{width:100%;padding:10px;border:0;border-radius:4px;background:var(--acc);
        color:#04121f;font:inherit;font-weight:700;cursor:pointer}
 button:disabled{background:#21262d;color:var(--dim);cursor:not-allowed}
 pre{background:#161b22;padding:10px;border-radius:4px;font-size:11px;
     max-height:38vh;overflow:auto;white-space:pre-wrap;color:var(--fg);
     margin-top:8px}
</style>
<header>
  <h1>Calibration</h1>
  <span class=sub id=hint>tilt spread decides whether the answer is RIGHT</span>
</header>
<main>
  <div class=vid><img src="/video"></div>
  <aside>
    <div class=msg id=msg>...</div>
    <h2>Views</h2>
    <div class=bar><i id=kb></i></div>
    <div class=row><span id=kept>0</span><span id=want></span></div>
    <h2>Frame coverage</h2>
    <div class=bar><i id=cb></i></div>
    <div class=row><span id=cov>0/9 cells</span>
      <span class=sub>edges carry the distortion</span></div>
    <h2>Tilt variety</h2>
    <div class=tilt id=tilt></div>
    <div class=row style="margin-top:6px"><span class=sub id=tiltmsg></span></div>
    <h2>Solve</h2>
    <button id=go>Run calibration</button>
    <div id=sv class=sub style="margin-top:8px"></div>
    <pre id=out></pre>
  </aside>
</main>
<script>
async function tick(){
 try{
  const r=await fetch('/status'); const s=await r.json();
  document.getElementById('msg').textContent = s.err ? s.err : s.msg;
  document.getElementById('msg').className = 'msg'+(s.err?' warn':'');
  document.getElementById('kept').textContent = s.kept+' captured';
  document.getElementById('want').textContent = 'need '+s.want;
  document.getElementById('kb').style.width = (100*s.kept/s.want)+'%';
  const cells=s.cover.flat().filter(v=>v>0).length;
  document.getElementById('cov').textContent = cells+'/9 cells';
  document.getElementById('cb').style.width = (100*cells/9)+'%';
  const t=document.getElementById('tilt'); t.innerHTML='';
  const names=['flat','slight','good','steep'];
  s.tilt.forEach((v,i)=>{const d=document.createElement('div');
    d.textContent=names[i]+' '+v; if(v>0)d.className='on'; t.appendChild(d);});
  const empty=s.tilt.filter(v=>v===0).length;
  document.getElementById('tiltmsg').textContent = empty
    ? 'still missing '+empty+' tilt band(s) -- angle the board more'
    : 'all tilt bands covered';
  const b=document.getElementById('go');
  b.disabled = s.solve==='running';
  b.textContent = s.solve==='running' ? 'solving...'
      : s.solve==='done' ? 'Re-run calibration' : 'Run calibration';
  document.getElementById('sv').textContent =
      s.solve==='done'   ? 'installed -- rebuild and the launch picks it up'
    : s.solve==='failed' ? 'solve failed, see below'
    : s.kept>=s.want ? 'ready' : 'you can solve early, but fill the bars first';
  document.getElementById('out').textContent = s.solve_out||'';
 }catch(e){}
}
document.getElementById('go').onclick=async()=>{
  await fetch('/solve',{method:'POST'}); tick();};
setInterval(tick,500); tick();
</script>
"""


def make_handler(st, solve_argv, solve_dest):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == '/':
                body = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/status':
                with st.lock:
                    s = json.dumps({
                        'kept': st.kept, 'want': st.want, 'msg': st.msg,
                        'cover': st.cover.tolist(), 'tilt': st.tilt.tolist(),
                        'err': st.err, 'solve': st.solve,
                        'solve_out': st.solve_out}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(s)))
                self.end_headers()
                self.wfile.write(s)
            elif self.path == '/video':
                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                try:
                    while True:
                        with st.lock:
                            f = st.frame
                        if f:
                            self.wfile.write(b'--f\r\nContent-Type: image/jpeg'
                                             b'\r\nContent-Length: '
                                             + str(len(f)).encode()
                                             + b'\r\n\r\n' + f + b'\r\n')
                        time.sleep(0.05)
                except Exception:
                    pass
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path != '/solve':
                self.send_response(404)
                self.end_headers()
                return
            with st.lock:
                busy = st.solve == 'running'
            if not busy:
                threading.Thread(target=run_solve,
                                 args=(st, solve_argv, solve_dest),
                                 daemon=True).start()
            self.send_response(204)
            self.end_headers()
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=3)
    ap.add_argument('--out', default=os.path.expanduser('~/fantech_cal'))
    ap.add_argument('--grid', default='8x6',
                    help='INNER corners, must match the board AND fov_solve')
    ap.add_argument('--square', type=float, default=0.025)
    ap.add_argument('--views', type=int, default=25)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--applies-to', default='pi_forward',
                    help='camera PROFILE this calibration describes')
    ap.add_argument('--install', default=None,
                    help='where the finished calibration is written; defaults '
                         'to the package source tree so it is committable')
    a = ap.parse_args()

    cols, rows = (int(x) for x in a.grid.lower().split('x'))
    os.makedirs(a.out, exist_ok=True)
    st = State(cols, rows, a.views, a.out)
    threading.Thread(target=capture_loop,
                     args=(st, a.device, a.width, a.height),
                     daemon=True).start()

    here = os.path.dirname(os.path.abspath(__file__))
    install = a.install or os.path.join(
        here, '..', 'src', 'duburi_vision', 'config', 'calibration')
    install = os.path.abspath(install)
    os.makedirs(install, exist_ok=True)
    solve_dest = os.path.join(install,
                              f'{a.applies_to}_{a.width}x{a.height}.json')
    solve_argv = [sys.executable, os.path.join(here, 'fov_solve.py'), a.out,
                  '--grid', a.grid, '--square', str(a.square),
                  '--applies-to', a.applies_to, '--install', install]
    # ThreadingHTTPServer: the MJPEG handler never returns, so a
    # single-threaded server would starve /status and the page would look hung.
    srv = ThreadingHTTPServer(('0.0.0.0', a.port),
                              make_handler(st, solve_argv, solve_dest))
    print(f'board {cols}x{rows} inner corners, {a.views} views -> {a.out}')
    print(f'open  http://<this-host>:{a.port}/   (or http://localhost:{a.port}/)')
    print(f'solve installs -> {solve_dest}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
