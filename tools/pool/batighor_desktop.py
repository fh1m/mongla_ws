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

The app imports a layout file once, by file name (its `from`), and keeps
its own copy; a later change to the file never reaches that copy. The
instruments panel keeps that copy in step instead: on every app start it
takes each view from Kutum's server (`syncLayouts` in
`panels/instruments`), keeping a view the operator edited (Revert in the
layout's menu takes Kutum's). So the views the app shows are Kutum's, not
this folder's, once it has connected to Kutum. A link
that names a layout saves a copy with no `from`, so the next start imports
the file again beside it: two of each in the menu. So no link names a
layout. Every run writes every view to the folder (a fresh app imports them
all), and starts the app with `--defaultLayout=<view>`, which selects by
name once the folder is imported: the cockpit, unless `--view` names
another. A view that changed is named, with how to refresh it. Read from
Lichtblick 1.27: `loadDefaultLayouts.ts`, `DesktopLayoutLoader.ts`,
`useLayoutTransfer.tsx` and `CurrentLayoutProvider/index.tsx`.

    python3 tools/pool/batighor_desktop.py                    # kutum.local, every view
    python3 tools/pool/batighor_desktop.py 192.168.0.103 --view localization
    python3 tools/pool/batighor_desktop.py --list             # the views Kutum serves
    python3 tools/pool/batighor_desktop.py --screen eDP-1 --vitals DP-2   # two screens, named
    python3 tools/pool/batighor_desktop.py --one-screen       # the cockpit alone, even on a Duo

Standard library only: the operator's machine needs no ROS.
"""
from __future__ import annotations

import argparse
import base64
import concurrent.futures
import datetime
import io
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
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
        # Complete means its code is there. A run that died between the two
        # files left a directory the app shows as "Unknown panel type", and
        # skipping on the directory alone kept it so (BUGS B-116).
        complete = (target / 'dist' / 'extension.js').is_file()
        for old in into.glob(f'{stem}-*'):
            if old != target or not complete:
                attic.mkdir(parents=True, exist_ok=True)
                shutil.move(str(old), attic / f'{old.name}-{time.time_ns()}')
        if complete:
            continue
        # Built beside it and renamed in whole, so a failure leaves nothing.
        stage = into / f'.{target.name}.part'
        shutil.rmtree(stage, ignore_errors=True)
        for name in ('package.json', 'dist/extension.js'):
            data = z.read(name)
            if name.endswith('.js'):
                data = embed_fonts(data, base)
            (stage / name).parent.mkdir(parents=True, exist_ok=True)
            (stage / name).write_bytes(data)
        stage.rename(target)
        wrote.append(target.name)
    return wrote


def install_views(base: str, views: list[str]) -> tuple[list[str], list[str]]:
    """Each view as Kutum serves it for this host (the hull's URL absolute).

    Returns (new, changed): a changed file is one the app may already hold
    an older copy of."""
    into = LAYOUTS
    into.mkdir(parents=True, exist_ok=True)
    attic = Path.home() / 'attic' / datetime.date.today().isoformat() / 'lichtblick-layouts'
    new, changed = [], []
    for name in views:
        data = get(f'{base}/layouts/{quote(name)}.json')
        json.loads(data)                       # a page that is not a layout stops here
        target = into / f'{name}.json'
        if target.is_file() and target.read_bytes() == data:
            continue
        if target.is_file():
            attic.mkdir(parents=True, exist_ok=True)
            shutil.move(str(target), attic / target.name)
            changed.append(name)
        else:
            new.append(name)
        target.write_bytes(data)
    return new, changed


def embed_fonts(code: bytes, base: str) -> bytes:
    """The theme's fonts as data: URIs in the panel's code.

    On Kutum they are root paths; the desktop app's page is a file and its
    policy is `font-src 'self' data:`, which blocks a font from Kutum.
    """
    def inline(m: re.Match) -> bytes:
        font = get(f'{base}/theme/fonts/{m.group(1).decode()}')
        return b'data:font/woff2;base64,' + base64.b64encode(font)
    return re.sub(rb'/theme/fonts/([a-z0-9-]+\.woff2)', inline, code)


# The density the app's sizes were drawn for, on a laptop read at arm's
# length. A 4K 15-inch panel is ~290 dpi; at device scale 1, the app's 12 px
# type is 1 mm tall on it (seen on the dev box 2026-11-12). At 2 the owner read
# the type as tiny (2026-11-24); 120 gives 2.5, 1536 px of layout, which the
# views are now laid out for (2026-11-21 had found them cramped there).
READ_DPI = 120

# The Vitals view opens in a window of its own, for a second screen. The app
# keeps one window per profile (a second launch hands its link to the first,
# and `--force-multiple-windows` opened the cockpit again, not the view it
# was given), so this window has its own profile; panels and layouts are
# shared from ~/.lichtblick-suite.
VITALS_VIEW = 'vitals'
VITALS_PROFILE = Path.home() / '.config' / 'Lichtblick-vitals'
# The screen under the main one, a quarter step smaller (2.25 on the Zenbook
# Duo): its panel is only 1100 px tall, and at 2.75 the type was too big for
# what Vitals must hold at a glance (owner, 2026-11-24). Its numbers are
# drawn large on purpose; the scale sets everything else.
VITALS_BOOST = -0.25


def hyprland(cmd: str) -> str:
    """One request to Hyprland's socket; '' where there is none."""
    import socket
    sig = os.environ.get('HYPRLAND_INSTANCE_SIGNATURE')
    run = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')) / 'hypr'
    socks = [run / sig / '.socket.sock'] if sig else sorted(run.glob('*/.socket.sock'))
    if not socks or not socks[0].exists():
        return ''
    with socket.socket(socket.AF_UNIX) as s:
        s.connect(str(socks[0]))
        s.sendall(cmd.encode())
        out = b''
        while chunk := s.recv(65536):
            out += chunk
    return out.decode()


def two_screens(monitors: list) -> tuple[str, str] | None:
    """(main, under) when a wide, short screen sits under another: the
    Zenbook Duo's ScreenPad (3840 x 1100, 3.5 wide to 1 tall) under its
    panel. None for an ordinary desk."""
    wide = [m for m in monitors if m.get('height') and m['width'] / m['height'] >= 3]
    for w in wide:
        above = [m for m in monitors if m is not w and m.get('x') == w.get('x')
                 and m.get('y', 0) < w.get('y', 0)]
        if above:
            return above[0]['name'], w['name']
    return None


# When the key is pressed, in seconds after the window shows: the page must be
# loaded first, and the app reloads once after start when Kutum's layouts
# changed (syncLayouts), which leaves full screen; a single press at 8 s left
# the window's buttons back on such a start (seen 2026-11-24).
PRESS_AT_S = (8, 20, 35)
# Focus, then full screen, is one step, and so is focus, then a press:
# Hyprland's fullscreen acts on the focused window, and a shortcut reaches an
# XWayland window only while it has focus. Two windows placed at once toggled
# one twice and the other never, and their presses landed nowhere (seen
# 2026-11-24). Only the waits between the presses overlap.
FOCUS = threading.Lock()


def place(pid: int, screen: str, wait_s: float = 30.0) -> bool:
    """Put the window of process `pid` on the workspace `screen` is showing,
    full screen, and press one key in it (Hyprland).

    The workspace the screen shows, not the monitor alone: a window rule had
    filed the cockpit on the OLED's workspace 7 while it showed 1 (seen
    2026-11-24). The key is Shift, which nothing acts on alone: a press is
    what lets the page go full screen and drop the window's own buttons
    (common.js), and it needs the page loaded first (PRESS_AT_S). Both dispatch dialects
    are tried: the Lua one this dev box's Hyprland takes, then the classic
    one. False when there is no Hyprland or the window never showed."""
    end = time.monotonic() + wait_s
    while time.monotonic() < end:
        try:
            clients = json.loads(hyprland('j/clients') or '[]')
            monitors = json.loads(hyprland('j/monitors') or '[]')
        except (OSError, ValueError):
            return False
        mine = [c for c in clients if c.get('pid') == pid]
        shown = [m['activeWorkspace']['id'] for m in monitors if m.get('name') == screen]
        if mine and shown:
            addr, ws = mine[0]['address'], shown[0]
            win = f'window = "address:{addr}"'
            with FOCUS:
                if hyprland(f'/dispatch hl.dsp.window.move({{ workspace = "{ws}", {win} }})').strip() == 'ok':
                    hyprland(f'/dispatch hl.dsp.focus({{ {win} }})')
                    if not mine[0].get('fullscreen'):
                        hyprland('/dispatch hl.dsp.window.fullscreen()')
                    focus = f'/dispatch hl.dsp.focus({{ {win} }})'
                    press = f'/dispatch hl.dsp.send_shortcut({{ mods = "", key = "Shift_L", {win} }})'
                else:
                    hyprland(f'/dispatch movetoworkspace {ws},address:{addr}')
                    hyprland(f'/dispatch focuswindow address:{addr}')
                    if not mine[0].get('fullscreen'):
                        hyprland('/dispatch fullscreen 0')
                    focus = f'/dispatch focuswindow address:{addr}'
                    press = f'/dispatch sendshortcut , Shift_L, address:{addr}'
            was = 0
            for at in PRESS_AT_S:
                time.sleep(at - was)
                was = at
                # A shortcut reaches this (XWayland) window only while it has
                # focus: placed together, the presses found it elsewhere.
                with FOCUS:
                    hyprland(focus)
                    hyprland(press)
            return True
        time.sleep(0.5)
    return False


def screen_scale(xrandr: str) -> float | None:
    """The scale for the first screen `xrandr --query` lists (the primary if
    marked), from its pixels and millimetres, to a quarter; None if unknown."""
    lines = [ln for ln in xrandr.splitlines() if ' connected' in ln]
    lines.sort(key=lambda ln: ' primary ' not in ln)
    for ln in lines[:1]:
        m = re.search(r'(\d+)x\d+\+\d+\+\d+.* (\d+)mm x \d+mm', ln)
        if m:
            return scale_for(int(m.group(1)), int(m.group(2)))
    return None


def scale_for(px: int, mm: int) -> float | None:
    """The scale for a screen `px` wide and `mm` wide, to a quarter; None if unknown."""
    if mm <= 0:
        return None
    return max(1.0, min(3.0, round(px / (mm / 25.4) / READ_DPI * 4) / 4))


def hyprland_scale(screen: str | None) -> float | None:
    """The scale for the screen named (else the focused one) from Hyprland's
    own sizes; None without Hyprland. Under Hyprland there is often no
    xrandr, and the app (through XWayland) is not scaled for it: on the
    Zenbook Duo it drew at 1, its type a millimetre tall (2026-11-24)."""
    try:
        mons = json.loads(hyprland('j/monitors') or '[]')
    except (OSError, ValueError):
        return None
    pick = [m for m in mons if m.get('name') == screen] or [m for m in mons if m.get('focused')] or mons
    return scale_for(pick[0].get('width', 0), pick[0].get('physicalWidth', 0)) if pick else None


def scale_flag(asked: float | None, screen: str | None = None, boost: float = 0.0) -> list[str]:
    """`--force-device-scale-factor` for the app, unless the desktop scales it."""
    if asked:
        return [f'--force-device-scale-factor={asked + boost:g}']
    if os.environ.get('GDK_SCALE'):
        return []
    found = hyprland_scale(screen)
    if found:
        found += boost
        return [f'--force-device-scale-factor={found:g}'] if found > 1 else []
    if not shutil.which('xrandr'):
        return []
    xrdb = subprocess.run(['xrdb', '-query'], capture_output=True, text=True).stdout \
        if shutil.which('xrdb') else ''
    dpi = re.search(r'Xft\.dpi:\s*(\d+)', xrdb)
    if dpi and int(dpi.group(1)) > 96:
        return []                      # the desktop already scales its apps
    found = screen_scale(subprocess.run(['xrandr', '--query'], capture_output=True, text=True).stdout)
    return [f'--force-device-scale-factor={found:g}'] if found and found > 1 else []


# A kernel GPU driver and the Vulkan driver (ICD) that serves it.
_VULKAN_ICD = {'nvidia': 'nvidia_icd', 'amdgpu': 'radeon_icd', 'i915': 'intel_icd',
               'xe': 'intel_icd'}
_ICD_DIRS = ('/usr/share/vulkan/icd.d', '/etc/vulkan/icd.d',
             '/usr/local/share/vulkan/icd.d', '~/.local/share/vulkan/icd.d')


def vulkan_flag(drm: Path = Path('/sys/class/drm'), icd_dirs=_ICD_DIRS) -> list[str]:
    """`--use-angle=vulkan` on Linux when a GPU in use has its Vulkan driver.

    Lichtblick builds its WebGL panels again on every tab switch. Through
    ANGLE's default OpenGL backend each shader compile waits, and a switch
    blocked the page for 1.5-4.9 s; on Vulkan, 70-390 ms (measured-bars 105).
    A blocked page also holds back a STOP press and the helm's heartbeat.
    Without a Vulkan driver the flag leaves no WebGL at all, so it is given
    only when the kernel's GPU driver has its ICD installed. Mesa installs
    ICDs for GPUs a machine does not have, so a file alone proves nothing;
    lavapipe, a CPU, has no kernel driver and never qualifies.
    """
    if not sys.platform.startswith('linux'):
        return []                     # ANGLE is on D3D (Windows) or Metal (macOS)
    drivers = set()
    for card in drm.glob('card[0-9]*'):
        try:
            drivers.add(Path(os.readlink(card / 'device' / 'driver')).name)
        except OSError:
            continue
    icds = {f.name.split('.')[0] for d in icd_dirs
            for f in Path(d).expanduser().glob('*.json')}
    return ['--use-angle=vulkan'] if any(_VULKAN_ICD.get(d) in icds for d in drivers) else []


def running() -> bool:
    """Whether the desktop app is open already (Linux and macOS; False if unknown)."""
    if shutil.which('pgrep') is None:
        return False
    return subprocess.run(['pgrep', '-xi', 'lichtblick'], capture_output=True).returncode == 0


def link(host: str, bridge: int) -> str:
    ds = quote(f'ws://{host}:{bridge}', safe='')
    return f'lichtblick://open?ds=foxglove-websocket&ds.url={ds}'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Open Batighor in the Lichtblick desktop app.')
    ap.add_argument('host', nargs='?', default='kutum.local', help="Kutum's name or address")
    ap.add_argument('--port', type=int, default=8000, help="Batighor's web port")
    ap.add_argument('--bridge', type=int, default=8765, help="foxglove_bridge's port")
    ap.add_argument('--view', help='open on this view (default: the cockpit, mongla_layout)')
    ap.add_argument('--scale', type=float, help='the app\'s scale (default: from the screen\'s density)')
    ap.add_argument('--list', action='store_true', help="print Kutum's views and exit")
    ap.add_argument('--no-launch', action='store_true', help='install the panels only')
    ap.add_argument('--one-screen', action='store_true',
                    help='one window even where a second screen is found')
    ap.add_argument('--vitals', metavar='SCREEN',
                    help='also open the Vitals view in its own window on this screen '
                         '(a Hyprland monitor name, e.g. DP-2 for the Zenbook Duo\'s ScreenPad)')
    ap.add_argument('--screen', metavar='SCREEN',
                    help='move the main window to this screen (Hyprland), e.g. eDP-1')
    ap.add_argument('--no-vulkan', action='store_true',
                    help="render through ANGLE's OpenGL backend even where Vulkan is found")
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
    if a.view and a.view not in views:
        print(f'ERROR: Kutum has no view {a.view!r}; it has: {", ".join(views)}', file=sys.stderr)
        return 1
    try:
        wrote = install(base)
        print(f'ok  panels: {", ".join(wrote) if wrote else "already current"}')
        new, changed = install_views(base, views)
    except OSError as exc:
        # Half a set is still a set the app can open; the operator reruns this.
        print(f'ERROR: Batighor stopped answering at {base} mid-install: {exc}; run this again',
              file=sys.stderr)
        return 1
    print(f'ok  views: {", ".join(new + changed) if new or changed else "already current"}')
    # Said whether or not this launches: with --no-launch it once returned
    # first, and a changed view went unmentioned while the app kept the old.
    # Deleting the app's copy does not help: the instruments panel takes
    # Kutum's again at the next start. It is the panel that refreshes it.
    for name in changed:
        print(f'ok  {name} changed on Kutum; the app takes it at its next start '
              f'(unless you edited it: Revert in the layout\'s menu)')
    if a.no_launch:
        return 0
    app = shutil.which('lichtblick')
    if app is None:
        print('ERROR: the lichtblick desktop app is not on PATH', file=sys.stderr)
        return 1
    # Two screens by default where Hyprland shows a screen under the main
    # one: the pictures and the pilot's tools above, Vitals below.
    if not a.one_screen and not a.vitals:
        try:
            found = two_screens(json.loads(hyprland('j/monitors') or '[]'))
        except (OSError, ValueError):
            found = None
        if found and VITALS_VIEW in views:
            a.screen = a.screen or found[0]
            a.vitals = found[1]
            print(f'ok  two screens: {a.screen} and {a.vitals} under it (--one-screen for one)')
    pilot = 'batighor_pilot'                   # the console, Pilot first (views.PILOT_CONSOLE)
    view = a.view or (pilot if a.vitals and pilot in views else 'mongla_layout')
    url = link(a.host, a.bridge)
    if running():
        # A second launch hands its link to the open app, which ignores flags.
        print(f'WARNING: the app is already open and keeps its view; pick {view} under '
              f'Layouts, or close the app and run this again')
    flags = scale_flag(a.scale, a.screen)
    gpu = [] if a.no_vulkan else vulkan_flag()
    main_app = subprocess.Popen([app, *flags, *gpu, f'--defaultLayout={view}', url],
                                start_new_session=True,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'ok  opened {view} on {a.host}' + (f' at scale {flags[0].split("=")[1]}' if flags else '')
          + (', drawing through Vulkan' if gpu else ''))
    placing = []
    if a.vitals:
        if VITALS_VIEW not in views:
            print(f'WARNING: Kutum serves no {VITALS_VIEW} view; update Kutum for the second screen')
        else:
            pad = subprocess.Popen([app, f'--user-data-dir={VITALS_PROFILE}',
                                    *scale_flag(a.scale, a.vitals, VITALS_BOOST), *gpu,
                                    f'--defaultLayout={VITALS_VIEW}', url], start_new_session=True,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f'ok  opened {VITALS_VIEW} in its own window')
            placing.append((pad.pid, a.vitals, f'the {VITALS_VIEW} window'))
    if a.screen:
        placing.append((main_app.pid, a.screen, 'the main window'))
    # Both at once: each waits for its page and presses three times.
    with concurrent.futures.ThreadPoolExecutor() as pool:
        done = list(pool.map(lambda w: place(w[0], w[1]), placing))
    for ok, (_, screen, what) in zip(done, placing):
        if not ok:
            print(f'WARNING: move {what} to {screen} by hand (placing it needs Hyprland)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
