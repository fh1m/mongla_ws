"""What a run leaves behind: vision evidence, the countdown, the scoreboard.

Mixed into `MonglaMission`.
"""
from __future__ import annotations
import json
import os
import subprocess
import sys
import time as _time

import rclpy

from .dsl_support import _write_ppm


_RUN_DIR_ENV     = 'MONGLA_RUN_DIR'
_DEFAULT_RUN_DIR = '~/mongla_runs'


def _run_dir() -> str:
    """Return the per-run artifact folder, creating it if absent."""
    base = os.path.expanduser(os.environ.get(_RUN_DIR_ENV) or _DEFAULT_RUN_DIR)
    os.makedirs(base, exist_ok=True)
    return base


def _git_sha() -> str:
    """Best-effort short git SHA of the workspace (which code ran this run).

    Returns '' if git/the repo is unavailable -- traceability is a nice-to-have,
    never a reason to raise on pool day.
    """
    try:
        out = subprocess.run(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, timeout=2.0)
        return out.stdout.strip() if out.returncode == 0 else ''
    except Exception:
        return ''


class ReportMixin:
    """Methods of `MonglaMission`; see `mongla_dsl.py` for the node."""

    def _record_vision(self, verb: str, target: str, camera: str, res) -> None:
        """Put WHERE and HOW a vision verb ended on its scoreboard row.

        The scorecard used to carry only cmd/success/elapsed/msg, and the vision
        server always reports success -- so a run that never saw the gate and
        one that centred it looked identical afterwards. Updates the verb's own
        row (the last `vision_<verb>`); adds one if the verb never reached the
        server.
        """
        import math as _math
        info = self.__dict__.get('_vinfo', {}).get(camera)
        evidence = {
            'target': str(target), 'camera': str(camera),
            'outcome': res.reason, 'saw_target': bool(res.saw_target),
            'x_px': None if _math.isnan(res.x_px) else round(res.x_px, 1),
            'y_px': None if _math.isnan(res.y_px) else round(res.y_px, 1),
            'fill': round(float(res.fill), 3), 'fired': res.fired,
            'model': list(info[0]) if info else None,
        }
        board = self.__dict__.setdefault('_scoreboard', [])
        for row in reversed(board):
            if row.get('cmd') == f'vision_{verb}' and 'vision' not in row:
                row['vision'] = evidence
                return
        board.append({'cmd': f'vision_{verb}', 'success': bool(res.ok),
                      'elapsed': round(float(res.elapsed_s), 2),
                      'msg': res.reason, 'vision': evidence})

    def save_evidence(self, camera: str | None = None, tag: str = 'evidence', *,
                      timeout: float = 2.0):
        """Save the next `image_debug` frame (boxes drawn) into the run folder.

        OPT-IN. Returns the file path, or None when no frame arrived. Written as
        binary PPM: no OpenCV or cv_bridge needed (both fail under numpy 2), and
        any viewer opens it. The path goes on the scoreboard so the scorecard row
        and the picture of how the task ended travel together.
        """
        import time as _t
        from sensor_msgs.msg import Image
        cam = camera or self.camera
        node = self.client.node
        got = {}
        sub = node.create_subscription(
            Image, f'/mongla/vision/{cam}/image_debug',
            lambda m: got.setdefault('img', m), 1)
        try:
            deadline = _t.monotonic() + float(timeout)
            while 'img' not in got and _t.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_subscription(sub)
        img = got.get('img')
        path = _write_ppm(img, _run_dir(), f'{tag}_{cam}') if img is not None else None
        self.__dict__.setdefault('_scoreboard', []).append(
            {'cmd': f'evidence:{tag}', 'success': path is not None,
             'elapsed': 0.0, 'msg': path or f'no image_debug frame from {cam}'})
        return path

    def countdown(self, seconds: int = 10, *,
                  message: str = "Wire removed  --  Mongla is now autonomous. Good luck."):
        """Print a tether-removal countdown and return."""
        width = 66
        border_h = '━' * width
        tl, tr, bl, br = '┏', '┓', '┗', '┛'
        vb = '┃'

        def _box(lines):
            print(f'{tl}{border_h}{tr}')
            for line in lines:
                pad = width - len(line)
                lp  = pad // 2
                rp  = pad - lp
                print(f'{vb}{" " * lp}{line}{" " * rp}{vb}')
            print(f'{bl}{border_h}{br}')

        print()
        _box([
            '',
            'TETHER REMOVAL WINDOW',
            '',
            'Disconnect the tether now.',
            f'Mission starts in {seconds} seconds.',
            '',
        ])
        print()

        for remaining in range(seconds, 0, -1):
            bar_total = 40
            filled    = int(bar_total * (seconds - remaining) / seconds)
            bar       = '█' * filled + '░' * (bar_total - filled)
            sys.stdout.write(f'\r  T-{remaining:3d}s  [{bar}]  ')
            sys.stdout.flush()
            _time.sleep(1)

        sys.stdout.write('\r' + ' ' * 60 + '\r')
        sys.stdout.flush()
        print()
        _box(['', message, ''])
        print()

    def log_scoreboard(self, *, json_path: str | None = None,
                       mission: str | None = None) -> None:
        """Print a structured per-verb mission summary and optionally write JSON.

        Called automatically by `mission.py` on exit (normal or exception).
        Can also be called manually at any point during a mission.

        Parameters
        ----------
        json_path : str | None
            If given, write the scoreboard JSON to this path in addition to
            printing to stdout.  Pass ``'auto'`` to write a timestamped
            ``<mission>_<ts>.json`` into the dedicated run folder (``_run_dir``,
            default ``~/mongla_runs``, override with ``MONGLA_RUN_DIR``).
        mission : str | None
            Mission name, recorded in the JSON and used in the ``'auto'``
            filename so a pool session's scorecards are traceable per run.

        Example output::

            ╔══════════════════════════════════════════════════════════════════╗
            ║  MISSION SCOREBOARD                         total: 47.3 s       ║
            ╠══════════╦══════════╦═══════╦═══════════════════════════════════╣
            ║  #  verb ║ success  ║  time ║  message                          ║
            ╠══════════╬══════════╬═══════╬═══════════════════════════════════╣
            ║  1  arm  ║        ║  2.1s ║  armed                            ║
            ║  ...                                                             ║
            ╚══════════════════════════════════════════════════════════════════╝
        """
        total_s = round(_time.monotonic() - self._mission_start, 1)
        width   = 68
        hb      = '═' * width

        def _row(n, entry):
            tick = 'ok' if entry['success'] else 'FAIL'
            cmd  = entry['cmd'][:18]
            t    = f"{entry['elapsed']:.1f}s"
            msg  = entry['msg'][:30]
            return f"  {n:>2d}  {cmd:<18s}  {tick}   {t:>5s}   {msg}"

        print(f'\n╔{hb}╗')
        print(f'║  MISSION SCOREBOARD{" " * (width - 20 - len(str(total_s)) - 12)}total: {total_s} s  ║')
        print(f'╠{hb}╣')
        print(f'║  {"#":>2s}  {"verb":<18s}  {"ok"}   {"time":>5s}   {"message":<30s}  ║')
        print(f'╠{hb}╣')
        for i, entry in enumerate(self._scoreboard, 1):
            row = _row(i, entry)
            pad = width - len(row)
            print(f'║{row}{" " * pad}║')
        print(f'╚{hb}╝\n')

        if json_path:
            payload = {
                'mission':       mission or '',
                'timestamp':     _time.strftime('%Y-%m-%dT%H:%M:%S'),
                'git_sha':       _git_sha(),
                'total_s':       total_s,
                'success_count': sum(1 for e in self._scoreboard if e['success']),
                'fail_count':    sum(1 for e in self._scoreboard if not e['success']),
                'phases':        self._scoreboard,
            }
            # Whole write is best-effort: log_scoreboard runs in mission.py's
            # finally, so a run-dir mkdir / write failure must never mask the
            # mission outcome. Resolve the auto path INSIDE the guard too
            # (_run_dir mkdir can raise on a read-only / full disk).
            try:
                if json_path == 'auto':
                    ts   = _time.strftime('%Y%m%d_%H%M%S')
                    name = (mission or 'mission').replace('/', '_')
                    json_path = os.path.join(_run_dir(), f'{name}_{ts}.json')
                with open(json_path, 'w') as fh:
                    json.dump(payload, fh, indent=2)
                self.log.info(f'[DSL  ] scoreboard → {json_path}')
            except OSError as exc:
                self.log.warning(f'[DSL  ] scoreboard write failed ({exc})')
