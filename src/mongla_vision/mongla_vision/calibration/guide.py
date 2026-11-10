#!/usr/bin/env python3
"""Step-by-step guided camera calibration in a browser.

Why a script of poses and not a freeform capture. A calibration fails in a
way that LOOKS LIKE SUCCESS: too little tilt leaves focal length and distance
confounded, and the fit then returns a LOW reprojection RMS with a WRONG
focal length. That is how this project's downward camera produced fx = 835.7,
969.9 and 1011.2 on three consecutive tries, each with a confident metric.

A freeform version of this tool was tried first, showing coverage and tilt as
bars for the operator to fill. Watched live it reached **9/9 coverage with 15
of 17 views FLAT** -- the operator naturally holds a board square-on, and the
one axis that decides correctness is the one that goes unfilled. Bars report
the problem; they do not prevent it.

So the capture is a SCRIPT. Each step names one pose -- a frame region and a
tilt -- checks the live detection against it, and only advances when it is
actually met. Coverage and tilt spread are then guaranteed by construction,
and the operator never has to work out what is missing.

Layout: streaming and detection run on separate threads, and detection works
on a downscale. Inline full-resolution detection made the video unusable
(with no board in frame the detector searches exhaustively before failing --
142 s per call at 1280x720 on random noise, ~112 ms at 480 px on a real
scene). The saved frame is full resolution and `fov_solve` re-detects it
there, so the downscale costs nothing.

ThreadingHTTPServer, not HTTPServer: an MJPEG handler never returns and would
starve every other route.

The files:
    guide.py           the HTTP server, restarts, `main`
    guide_files.py     camera discovery, the calibration and capture directories, the library
    guide_capture.py   the pose script, the capture/detect threads, `assess`
    guide_page.py      the page served at `/`
    solver.py          the fit; also run as a subprocess by the Solve button

Usage (stop the vision launch first -- it holds the cameras):
    python3 tools/pool/fov_calibrate_web.py --device 3 --out ~/fantech_cal
"""
import argparse
import glob
import re
import signal
import socket
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def _sibling(stem):
    """A module beside this file, loaded by path.

    `tools/pool/fov_calibrate_web.py` runs this file without a sourced ROS or an
    installed package, where a relative import has no parent to resolve
    against. Loading the sibling by path keeps that entry point ROS-free.
    """
    import importlib.util
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        f'mongla_calib_{stem}', os.path.join(here, f'{stem}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# One implementation, two callers: `solver.py` is also what the Solve button
# runs as a subprocess, so the naming rule and the geometry exist once.
try:
    from . import guide_capture as capture          # the installed package
    from . import guide_files as files
    from . import guide_page, solver
except ImportError:                                  # run as a file (tools/)
    capture = _sibling('guide_capture')
    files = _sibling('guide_files')
    guide_page = _sibling('guide_page')
    solver = _sibling('solver')
PAGE = guide_page.PAGE
camera_identity, list_cameras = files.camera_identity, files.list_cameras
default_calibration_dir, capture_dir = files.default_calibration_dir, files.capture_dir
library, apply_calibration = files.library, files.apply_calibration


def run_solve(st, argv, dest):
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
            st.solve, st.solve_out = 'failed', str(exc)


def make_handler(st, solve_argv, solve_dest, cal_dir, restart):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self):
            with st.lock:
                i, n = st.i, len(st.steps)
                done = i >= n
                ty, tx, tb = st.steps[min(i, n - 1)]
                cell, band = st.cell, st.band
                d = {
                    'i': i, 'n': n, 'done': done,
                    'want_cell': capture.CELL_NAME[ty][tx], 'want_tilt': capture.TILT_NAME[tb],
                    'cell': capture.CELL_NAME[cell[0]][cell[1]] if cell else None,
                    'band': capture.TILT_NAME[band] if band is not None else None,
                    'ok_cell': bool(cell == (ty, tx)),
                    'ok_tilt': bool(band == tb),
                    'hold': st.hold, 'msg': st.msg, 'err': st.err,
                    'sharp': st.sharp, 'sharp_max': capture.MAX_SHARPNESS_PX,
                    'undistort': st.undistort, 'medium': st.medium,
                    'grid': f'{st.cols}x{st.rows}',
                    'square': st.square,
                    'have_result': os.path.exists(solve_dest),
                    'ere': st.ere, 'ere_pred': st.ere_pred,
                    'ere_bar': capture.ERE_BAR_PX, 'n_boards': st.n_boards,
                    'fx': st.fx, 'fy': st.fy, 'cx': st.cx, 'cy': st.cy,
                    'fx_sd': st.fx_sd, 'hfov_air': st.hfov_air,
                    'hfov_water': st.hfov_water,
                    'suggest': (f'{capture.CELL_NAME[st.suggest[0][0]][st.suggest[0][1]]}'
                                f' at ~{st.suggest[1]} deg'
                                if st.suggest else None),
                    'fps': round(st.fps, 1), 'det_ms': round(st.det_ms, 1),
                    'solve': st.solve, 'solve_out': st.solve_out,
                }
            return json.dumps(d).encode()

        def do_GET(self):
            if self.path == '/':
                b = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/calibration.json':
                # Download the installed calibration. A calibration is
                # expensive to make and trivial to lose; it should be one
                # click to keep a copy off the vehicle.
                try:
                    b = open(solve_dest, 'rb').read()
                except Exception:
                    self.send_response(404); self.end_headers(); return
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Disposition',
                                 'attachment; filename="'
                                 + os.path.basename(solve_dest) + '"')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/library':
                b = json.dumps({'library': library(cal_dir),
                                'dir': cal_dir}).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/cameras':
                b = json.dumps({
                    'cameras': list_cameras(cal_dir),
                    'active': st.device,
                }).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/status':
                b = self._json()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == '/video':
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                try:
                    while True:
                        with st.lock:
                            f = st.frame
                        if f:
                            self.wfile.write(
                                b'--f\r\nContent-Type: image/jpeg\r\n'
                                b'Content-Length: ' + str(len(f)).encode()
                                + b'\r\n\r\n' + f + b'\r\n')
                        time.sleep(0.02)
                except Exception:
                    pass
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path.startswith('/switch'):
                # Re-exec rather than retarget the threads. The capture
                # thread owns an open V4L2 handle and the detector owns an
                # anchor; unwinding both live is a race for no gain. The
                # process replaces itself with the same argv bar the camera,
                # which reuses the reaper, the resume, and every check that
                # runs at startup -- one code path, not two.
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                dev = q.get('device', [None])[0]
                prof = q.get('applies_to', [None])[0]
                med = q.get('medium', [None])[0]
                self.send_response(204)
                self.end_headers()
                # A medium change alone is a valid switch -- the camera does
                # not move when the hull goes in the water, the refraction
                # does.
                if dev is not None or med is not None:
                    threading.Thread(
                        target=restart,
                        kwargs={'device': dev, 'applies_to': prof,
                                'medium': med},
                        daemon=True).start()
                return
            if self.path.startswith('/apply'):
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                name, err = apply_calibration(
                    cal_dir, q.get('file', [''])[0],
                    q.get('device', ['/dev/video0'])[0],
                    q.get('applies_to', ['pi_forward'])[0])
                with st.lock:
                    st.msg = (
                        f'applied -> {name}. The RUNNING vision node still '
                        f'holds the old intrinsics -- colcon build and '
                        f'relaunch before trusting a bearing.'
                        if name else f'apply refused: {err}')
                self.send_response(204); self.end_headers(); return
            if self.path == '/undistort':
                with st.lock:
                    st.undistort = not st.undistort
                self.send_response(204); self.end_headers(); return
            if self.path == '/undo':
                # Drop the LAST capture. A view you know was bad -- a hand in
                # frame, a wobble -- is worth deleting on the spot; leaving it
                # in and hoping RANSAC eats it is how a quiet bias gets in.
                with st.lock:
                    fs_ = sorted(glob.glob(os.path.join(st.outdir,
                                                        'cal_*.png')))
                    if fs_:
                        os.remove(fs_[-1])
                        st.i = max(0, st.i - 1)
                        st.assessed_n = -1        # force a refit
                        st.msg = f'deleted {os.path.basename(fs_[-1])}'
                self.send_response(204); self.end_headers(); return
            if self.path == '/reset':
                with st.lock:
                    for f_ in glob.glob(os.path.join(st.outdir, 'cal_*.png')):
                        os.remove(f_)
                    st.i = 0
                    st.ere = st.ere_pred = st.K = None
                    st.assessed_n = -1
                    st.n_boards = 0
                    st.msg = 'reset -- all captures deleted'
                self.send_response(204); self.end_headers(); return
            if self.path.startswith('/board'):
                # The board can differ on the day. A tool that hardcodes the
                # grid is a tool that cannot be used with the board you
                # actually brought -- and a WRONG grid detects nothing while
                # looking like a lighting problem.
                from urllib.parse import urlparse, parse_qs
                q = parse_qs(urlparse(self.path).query)
                self.send_response(204); self.end_headers()
                threading.Thread(
                    target=restart,
                    kwargs={'device': None,
                            'grid': q.get('grid', [None])[0],
                            'square': q.get('square', [None])[0]},
                    daemon=True).start()
                return
            if self.path == '/skip':
                with st.lock:
                    if st.i < len(st.steps):
                        st.i += 1
                        st.msg = 'skipped -- that pose is missing from the set'
                self.send_response(204)
                self.end_headers()
            elif self.path == '/solve':
                with st.lock:
                    busy = st.solve == 'running'
                if not busy:
                    threading.Thread(target=run_solve,
                                     args=(st, solve_argv, solve_dest),
                                     daemon=True).start()
                self.send_response(204)
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()
    return H


def restart_argv(argv, cur_profile, cur_medium, device=None,
                 applies_to=None, grid=None, square=None, medium=None):
    """The argv a re-exec should run with. Pure, so it can be driven.

    `--out` is always rewritten, from BOTH the profile and the medium.
    An earlier version rewrote it only when the profile changed, so a
    medium-only switch re-execed pointing at the previous capture folder,
    `resume()` picked up the AIR frames, and Solve installed them as
    `..._water.json` -- a filename saying water over pixels taken in air,
    which is the exact 1.44x confusion the medium flag exists to prevent.
    Nothing in the naming or file-content tests can see that, because the
    artifact is correct and the INPUT is wrong.
    """
    new = list(argv)

    def setarg(flag, val):
        if val is None:
            return
        if flag in new:
            new[new.index(flag) + 1] = str(val)
        else:
            new.extend([flag, str(val)])

    setarg('--device', device)
    setarg('--applies-to', applies_to)
    setarg('--grid', grid)
    setarg('--square', square)
    setarg('--medium', medium)
    setarg('--out', capture_dir(applies_to or cur_profile,
                                medium or cur_medium))
    return new


def reap_previous(port, wait_s=6.0):
    """Kill any EARLIER instance of this tool, then wait for it to let go.

    On competition ground you do not go PID hunting. A second `ros2 run
    mongla_vision calibrate` used to die on `OSError: [Errno 98] Address
    already in use`, and the fix -- find the process, work out that there are
    TWO of them, kill both -- is exactly the wrong thing to be doing beside a
    pool with a run slot ticking. Restarting a tool must just work.

    TWO processes per instance, which is the trap: `ros2 run` execs the node
    as a CHILD, so killing the wrapper leaves the child holding both the port
    and the camera. That is the same defect that put three duplicate copies
    of every A/B arm on this machine earlier today, met a third time.

    Found two ways, because either alone has a hole: whoever holds the PORT
    (catches an instance started under a different name), and whoever LOOKS
    like this tool (catches one that has not bound yet, mid-startup).

    Never kills anything in our OWN process group -- that includes the
    `ros2 run` wrapper that launched us, and killing it would take us with
    it.
    """
    me = os.getpid()
    try:
        my_pg = os.getpgid(0)
    except OSError:
        my_pg = None
    victims = set()

    out = subprocess.run(['ss', '-lntpH', f'sport = :{port}'],
                         capture_output=True, text=True)
    for m in re.finditer(r'pid=(\d+)', out.stdout or ''):
        victims.add(int(m.group(1)))

    for cl in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            pid = int(cl.split('/')[2])
            cmd = open(cl, 'rb').read().replace(b'\x00', b' ').decode(
                'utf-8', 'replace')
        except Exception:
            continue
        if any(k in cmd for k in ('mongla_vision/calibrate',
                                  'mongla_vision calibrate',
                                  'calibration/guide.py',
                                  'fov_calibrate_web')):
            victims.add(pid)

    kept = []
    for v in list(victims):
        if v == me:
            continue
        try:
            if my_pg is not None and os.getpgid(v) == my_pg:
                continue                       # our own wrapper -- leave it
        except OSError:
            pass
        kept.append(v)
    if not kept:
        return []

    for sig in (signal.SIGTERM, signal.SIGKILL):
        alive = []
        for v in kept:
            try:
                os.kill(v, sig)
                alive.append(v)
            except OSError:
                pass
        if not alive:
            break
        t0 = time.time()
        while time.time() - t0 < wait_s / 2:
            if not any(os.path.exists(f'/proc/{v}') for v in alive):
                break
            time.sleep(0.2)
        if not any(os.path.exists(f'/proc/{v}') for v in alive):
            break
    # The port lingers briefly in TIME_WAIT even after the holder is gone.
    t0 = time.time()
    while time.time() - t0 < wait_s:
        probe = socket.socket()
        try:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('0.0.0.0', port))
            probe.close()
            break
        except OSError:
            probe.close()
            time.sleep(0.3)
    return kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', type=int, default=3)
    # Derived from the profile AND the medium unless given explicitly --
    # see `capture_dir`.
    ap.add_argument('--out', default=None)
    ap.add_argument('--grid', default='8x6',
                    help='INNER corners; must match the board AND fov_solve')
    ap.add_argument('--square', type=float, default=0.025)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--port', type=int, default=8099)
    ap.add_argument('--detect-width', type=int, default=480)
    ap.add_argument('--stream-width', type=int, default=800)
    ap.add_argument('--exposure', type=int, default=50,
                    help='manual shutter in units of 0.1 ms. 0 leaves the '
                         'camera on auto, which measured a 200 ms shutter '
                         'here and smeared every hand movement.')
    ap.add_argument('--brightness', type=int, default=150)
    ap.add_argument('--applies-to', default='pi_forward')
    ap.add_argument('--medium', choices=('air', 'water'), default='air',
                    help='the medium the board is being viewed THROUGH. '
                         'Air and water differ by ~1.44x on this hull; the '
                         'medium is recorded in the file, shown on the page '
                         'and suffixed onto the filename so an in-water '
                         'calibration can never be mistaken for an in-air '
                         'one.')
    ap.add_argument('--install', default=None)
    ap.add_argument('--no-reap', action='store_true',
                    help='do NOT stop an earlier instance first. Only for '
                         'deliberately running two at once; the default is '
                         'to take over, because hunting a PID beside a pool '
                         'is the wrong job at the wrong moment.')
    a = ap.parse_args()

    if not a.no_reap:
        killed = reap_previous(a.port)
        if killed:
            print(f'stopped {len(killed)} earlier instance(s): '
                  f'{", ".join(str(k) for k in killed)}')

    cols, rows = (int(x) for x in a.grid.lower().split('x'))
    if a.out is None:
        a.out = capture_dir(a.applies_to, a.medium)
    os.makedirs(a.out, exist_ok=True)
    st = capture.State(cols, rows, a.out, capture.build_steps())
    st.square = a.square
    capture.resume(st)

    here = os.path.dirname(os.path.abspath(__file__))
    install = os.path.abspath(a.install or default_calibration_dir())
    os.makedirs(install, exist_ok=True)
    print(f'calibrations directory: {install}')
    st.medium = a.medium
    # Same rule the solver installs by -- imported, not restated, so the
    # Download button can never serve a different file than Solve wrote.
    dest = os.path.join(install, solver.calibration_filename(
        a.applies_to, a.width, a.height, a.medium))
    # `solver.py`, NOT `fov_solve.py`. The name changed when calibration
    # moved into the package and this path did not, so the Solve button would
    # have launched a file that does not exist -- a stale string surviving a
    # rename, which is the same defect class as the calibration filename that
    # outlived the camera it was named for.
    argv = [sys.executable, os.path.join(here, 'solver.py'), a.out,
            '--grid', a.grid, '--square', str(a.square),
            '--applies-to', a.applies_to, '--install', install,
            '--medium', a.medium,
            '--identity', json.dumps(camera_identity(f'/dev/video{a.device}'))]

    threading.Thread(target=capture.capture_loop,
                     args=(st, a.device, a.width, a.height, a.stream_width,
                           a.exposure, a.brightness),
                     daemon=True).start()
    threading.Thread(target=capture.detect_loop, args=(st, a.detect_width),
                     daemon=True).start()
    threading.Thread(target=capture.assess,
                     args=(st, cols, rows, a.square, (a.width, a.height),
                           solver),
                     daemon=True).start()

    st.device = a.device

    def restart(device=None, applies_to=None, grid=None, square=None,
                medium=None):
        """Re-exec on the chosen camera/medium. One startup path, not two."""
        new = restart_argv(sys.argv, a.applies_to, a.medium, device,
                           applies_to, grid, square, medium)
        time.sleep(0.4)
        # `sys.argv[0]` is a Python file either way -- a .py when run
        # directly, and the console-script wrapper under `ros2 run` -- so one
        # form covers both. The earlier version branched on the name and was
        # wrong for the wrapper, which is the case that actually ships.
        os.execv(sys.executable, [sys.executable] + new)

    srv = ThreadingHTTPServer(('0.0.0.0', a.port),
                              make_handler(st, argv, dest, install, restart))
    print(f'{len(st.steps)} guided poses -> {a.out}  (resuming at step '
          f'{st.i + 1})')
    # Never a hardcoded IP: the vehicle's address changes with the link,
    # and a printed URL that quietly stopped being true reads to an operator
    # as "the server did not start". The mDNS name is the stable one.
    hosts = [socket.gethostname().split('.')[0] + '.local']
    try:
        hosts += [ip for ip in subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True,
            timeout=2).stdout.split()
            if ':' not in ip and not ip.startswith(('127.', '169.254.'))
            and not (ip.split('.')[0] == '172'
                     and 16 <= int(ip.split('.')[1]) <= 31)]
    except Exception:
        pass
    print('open  ' + '   or   '.join(f'http://{h}:{a.port}/' for h in hosts))
    print(f'solve installs -> {dest}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
