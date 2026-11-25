"""The desktop app's layout menu holds each Batighor view once, and opens on the cockpit.

`tools/pool/batighor_desktop.py` puts Kutum's views where the Lichtblick
desktop app reads them. Lichtblick 1.27 imports a folder file once, by its
`from`; a link that names a layout saves a copy with no `from`, so the next
start imports the file again beside it. Seen on the desktop app 2026-11-12:
every view listed twice, and a fresh profile opening on `bins_downward`
(the first name) instead of the cockpit, because the link raced the import.
So no link names a layout, and the app is told which to select by name.
"""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
VIEWS = ['batighor', 'batighor_pilot', 'board_health', 'mongla_layout', 'thrust', 'vitals']


@pytest.fixture
def desk(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        '_desk', ROOT / 'tools' / 'pool' / 'batighor_desktop.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'LAYOUTS', tmp_path / '.lichtblick-suite' / 'layouts')
    monkeypatch.setattr(mod.Path, 'home', staticmethod(lambda: tmp_path))
    mod.served = {v: {'layout': v} for v in VIEWS}

    def get(url):
        if url.endswith('/api/workspaces/mongla/layouts'):
            return json.dumps({'data': [{'name': v} for v in mod.served]}).encode()
        return json.dumps(mod.served[url.rsplit('/', 1)[-1].removesuffix('.json')]).encode()
    monkeypatch.setattr(mod, 'get', get)
    monkeypatch.setattr(mod, 'install', lambda base: [])
    monkeypatch.setattr(mod.shutil, 'which', lambda _: '/bin/true')
    monkeypatch.setattr(mod, 'running', lambda: False)
    monkeypatch.setattr(mod, 'vulkan_flag', lambda: [])  # this machine's GPU is not the test's
    monkeypatch.setenv('GDK_SCALE', '1')        # the desktop scales: no flag of ours
    monkeypatch.setattr(mod, 'hyprland', lambda cmd: '')    # this machine's compositor is not the test's
    mod.launches, mod.placed = [], []

    class Proc:
        def __init__(self, argv, **_):
            mod.launches.append(argv)
            self.pid = 1000 + len(mod.launches)
    monkeypatch.setattr(mod.subprocess, 'Popen', Proc)
    monkeypatch.setattr(mod, 'place', lambda pid, screen: mod.placed.append((pid, screen)) or True)
    return mod


def test_a_fresh_app_opens_on_the_console_as_the_browser_does(desk):
    assert desk.main(['h']) == 0
    argv = desk.launches[0]
    assert '--defaultLayout=batighor' in argv
    assert sorted(p.stem for p in desk.LAYOUTS.glob('*.json')) == sorted(VIEWS)


def test_no_link_ever_names_a_layout(desk):
    """A layout saved from a link has no `from`; the folder copy comes back beside it."""
    for args in (['h'], ['h', '--view', 'thrust'], ['h']):
        assert desk.main(args) == 0
    assert all('layoutUrl' not in arg for argv in desk.launches for arg in argv)
    assert '--defaultLayout=thrust' in desk.launches[1]


def test_a_changed_view_is_named_with_how_to_refresh_it(desk, capsys):
    assert desk.main(['h']) == 0
    desk.served['thrust'] = {'layout': 'thrust', 'v': 2}
    capsys.readouterr()
    assert desk.main(['h']) == 0
    out = capsys.readouterr().out
    assert 'thrust changed on Kutum' in out and 'next start' in out
    assert json.loads((desk.LAYOUTS / 'thrust.json').read_text())['v'] == 2
    # Without launching too: --no-launch once returned before the warning,
    # and the app kept the old view with nothing said.
    desk.served['thrust'] = {'layout': 'thrust', 'v': 3}
    assert desk.main(['h', '--no-launch']) == 0
    assert 'thrust changed on Kutum' in capsys.readouterr().out


def test_vitals_opens_in_a_window_of_its_own_on_the_screen_named(desk, capsys, monkeypatch):
    """One window per profile: the second screen's window gets its own, or the
    app hands it to the cockpit's window and opens the cockpit again."""
    assert desk.main(['h', '--screen', 'eDP-1', '--vitals', 'DP-2']) == 0
    cockpit, vitals = desk.launches
    assert '--defaultLayout=batighor_pilot' in cockpit   # the keys and the state are on Vitals
    assert not any(a.startswith('--user-data-dir') for a in cockpit)
    assert '--defaultLayout=vitals' in vitals
    assert f'--user-data-dir={desk.VITALS_PROFILE}' in vitals
    assert desk.placed == [(1002, 'DP-2'), (1001, 'eDP-1')]
    # Found, not named: a screen under another, three times wider than tall.
    duo = [{'name': 'eDP-1', 'width': 3840, 'height': 2160, 'x': 0, 'y': 0, 'physicalWidth': 340},
           {'name': 'DP-2', 'width': 3840, 'height': 1100, 'x': 0, 'y': 1080, 'physicalWidth': 340},
           {'name': 'HDMI-A-1', 'width': 1920, 'height': 1080, 'x': 1920, 'y': 0, 'physicalWidth': 480}]
    monkeypatch.delenv('GDK_SCALE')                  # Hyprland's sizes set the scale
    assert desk.two_screens(duo) == ('eDP-1', 'DP-2')
    assert desk.two_screens(duo[::2]) is None
    desk.launches.clear(); desk.placed.clear()
    desk.hyprland = lambda cmd: json.dumps(duo)
    assert desk.main(['h']) == 0
    assert '--defaultLayout=batighor_pilot' in desk.launches[0]  # keys and state are on Vitals
    assert '--force-device-scale-factor=2.25' in desk.launches[1]  # Vitals, a quarter smaller
    assert '--defaultLayout=vitals' in desk.launches[1]
    assert sorted(s for _, s in desk.placed) == ['DP-2', 'eDP-1']
    desk.launches.clear()
    assert desk.main(['h', '--one-screen']) == 0
    assert len(desk.launches) == 1 and '--defaultLayout=batighor' in desk.launches[0]
    desk.hyprland = lambda cmd: ''
    monkeypatch.setenv('GDK_SCALE', '1')
    # A Kutum without the view says so and opens no second window.
    desk.launches.clear()
    del desk.served['vitals']
    capsys.readouterr()
    assert desk.main(['h', '--vitals', 'DP-2']) == 0
    assert len(desk.launches) == 1
    assert 'serves no vitals view' in capsys.readouterr().out


def test_a_placed_window_lands_where_its_screen_is_looking_and_is_pressed_past_a_reload(tmp_path, monkeypatch):
    """Seen 2026-11-24: a window rule filed the cockpit on the OLED's
    workspace 7 while the OLED showed 1, so the screen showed the desktop."""
    spec = importlib.util.spec_from_file_location('_desk4', ROOT / 'tools' / 'pool' / 'batighor_desktop.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sent = []

    def hypr(cmd):
        if cmd == 'j/clients':
            return json.dumps([{'pid': 7, 'address': '0xab', 'fullscreen': 0}])
        if cmd == 'j/monitors':
            return json.dumps([{'name': 'eDP-1', 'activeWorkspace': {'id': 1}},
                               {'name': 'DP-2', 'activeWorkspace': {'id': 90}}])
        sent.append(cmd)
        return 'ok'
    monkeypatch.setattr(mod, 'hyprland', hypr)
    monkeypatch.setattr(mod.time, 'sleep', lambda s: None)
    assert mod.place(7, 'eDP-1')
    assert 'workspace = "1"' in sent[0] and 'address:0xab' in sent[0]
    assert any('fullscreen' in c for c in sent)
    presses = [c for c in sent if 'Shift_L' in c]
    # Pressed again after the app's one reload, which leaves full screen.
    assert len(presses) == len(mod.PRESS_AT_S) >= 2 and max(mod.PRESS_AT_S) >= 20
    assert all('address:0xab' in c for c in presses)
    # No Hyprland: says so, places nothing.
    monkeypatch.setattr(mod, 'hyprland', lambda cmd: '')
    assert not mod.place(7, 'eDP-1', wait_s=0.01)


def test_a_dense_screen_starts_the_app_at_a_scale_it_can_be_read_at(desk):
    """A 4K 15-inch panel at device scale 1 drew 12 px type 1 mm tall; at 2
    the owner still read it as tiny (2026-11-24); 2.5 is the working scale."""
    laptop = ('HDMI-A-1 connected 1920x1080+7680+0 (normal) 480mm x 260mm\n'
              'eDP-1 connected primary 3840x2160+0+0 (normal) 340mm x 190mm\n')
    assert desk.screen_scale(laptop) == 2.5         # 287 dpi against READ_DPI 120
    assert desk.screen_scale('DP-1 connected 1920x1080+0+0 (normal) 527mm x 296mm') == 1.0
    assert desk.screen_scale('eDP-1 connected 1920x1080+0+0 (normal) 0mm x 0mm') is None
    # Under Hyprland, from its own sizes, per screen: no xrandr there.
    duo = json.dumps([{'name': 'eDP-1', 'width': 3840, 'physicalWidth': 340, 'focused': True},
                      {'name': 'HDMI-A-1', 'width': 1920, 'physicalWidth': 480}])
    desk.hyprland = lambda cmd: duo
    assert desk.hyprland_scale('eDP-1') == 2.5 and desk.hyprland_scale('HDMI-A-1') == 1.0
    assert desk.hyprland_scale(None) == 2.5                      # the focused screen
    desk.hyprland = lambda cmd: ''
    assert desk.hyprland_scale('eDP-1') is None
    assert desk.main(['h']) == 0
    assert not any(a.startswith('--force-device-scale-factor') for a in desk.launches[0])
    assert desk.main(['h', '--scale', '2']) == 0
    assert '--force-device-scale-factor=2' in desk.launches[1]


def _panel_zip():
    import io
    import zipfile
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w') as z:
        z.writestr('package.json', json.dumps(
            {'publisher': 'mongla', 'name': 'batighor-calibrate', 'version': '1.0.1'}))
        z.writestr('dist/extension.js', 'registerPanel({name: "Calibrate"})')
    return buf.getvalue()


def test_a_panel_is_whole_or_absent_and_a_half_one_is_repaired(tmp_path, monkeypatch):
    """Seen 2026-11-18: a run died fetching a font between package.json and
    extension.js, and every later run skipped the directory, so the calibrate
    view read "Unknown panel type" for good (B-116)."""
    spec = importlib.util.spec_from_file_location(
        '_desk2', ROOT / 'tools' / 'pool' / 'batighor_desktop.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod.Path, 'home', staticmethod(lambda: tmp_path))
    into = tmp_path / 'ext'
    listed = json.dumps({'data': [{'id': 'mongla.batighor-calibrate'}]}).encode()
    monkeypatch.setattr(mod, 'get', lambda url: listed if url.endswith('/extensions')
                        else _panel_zip())

    def dies(data, base):
        raise OSError('font 404')
    monkeypatch.setattr(mod, 'embed_fonts', dies)
    with pytest.raises(OSError):
        mod.install('http://k', into)
    assert not (into / 'mongla.batighor-calibrate-1.0.1').exists(), \
        'a failed install must leave no directory to be skipped later'

    half = into / 'mongla.batighor-calibrate-1.0.1'
    half.mkdir(parents=True)
    (half / 'package.json').write_text('{}')               # what the old code left
    monkeypatch.setattr(mod, 'embed_fonts', lambda data, base: data)
    assert mod.install('http://k', into) == ['mongla.batighor-calibrate-1.0.1']
    assert (half / 'dist' / 'extension.js').is_file()
    assert mod.install('http://k', into) == [], 'a whole panel is left alone'


def _gpu(tmp_path, drivers, icds):
    """A /sys/class/drm with one card per kernel driver, and an ICD folder."""
    drm, icd = tmp_path / 'drm', tmp_path / 'icd.d'
    icd.mkdir(parents=True)
    for i, d in enumerate(drivers):
        (tmp_path / 'drivers' / d).mkdir(parents=True, exist_ok=True)
        (drm / f'card{i}' / 'device').mkdir(parents=True)
        (drm / f'card{i}' / 'device' / 'driver').symlink_to(tmp_path / 'drivers' / d)
    for name in icds:
        (icd / name).write_text('{}')
    return drm, (str(icd),)


@pytest.mark.parametrize('drivers, icds, vulkan', [
    (['nvidia', 'i915'], ['nvidia_icd.json', 'intel_icd.x86_64.json'], True),  # this dev box
    (['i915'], ['intel_icd.x86_64.json'], True),
    # Mesa ships ICDs for GPUs the machine lacks; lavapipe is a CPU.
    (['i915'], ['radeon_icd.x86_64.json', 'lvp_icd.x86_64.json'], False),
    ([], ['lvp_icd.x86_64.json', 'nvidia_icd.json'], False),
    (['nvidia'], [], False),
])
def test_vulkan_only_where_a_gpu_in_use_has_its_driver(tmp_path, monkeypatch,
                                                       drivers, icds, vulkan):
    """Without a Vulkan driver `--use-angle=vulkan` leaves the app no WebGL at
    all (seen: every 3D and camera panel blank), so the flag needs both the
    kernel's GPU driver and its ICD."""
    spec = importlib.util.spec_from_file_location(
        '_desk_gpu', ROOT / 'tools' / 'pool' / 'batighor_desktop.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod.sys, 'platform', 'linux')
    drm, dirs = _gpu(tmp_path, drivers, icds)
    assert mod.vulkan_flag(drm, dirs) == (['--use-angle=vulkan'] if vulkan else [])


def test_vulkan_reaches_the_launch_unless_the_operator_says_no(desk, monkeypatch):
    monkeypatch.setattr(desk, 'vulkan_flag', lambda: ['--use-angle=vulkan'])
    assert desk.main(['h']) == 0 and desk.main(['h', '--no-vulkan']) == 0
    assert '--use-angle=vulkan' in desk.launches[0]
    assert '--use-angle=vulkan' not in desk.launches[1]


def test_two_windows_placed_at_once_both_end_full_screen_and_both_are_pressed(monkeypatch):
    """Hyprland's fullscreen, and a shortcut to an XWayland window, act on the
    focused window. Placed in parallel, one window's focus landed between the
    other's focus and fullscreen: one was toggled twice and the other never,
    and no press reached either page (seen on the Duo, 2026-11-24)."""
    import threading
    import time as _time
    spec = importlib.util.spec_from_file_location('_desk6', ROOT / 'tools' / 'pool' / 'batighor_desktop.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    real_sleep, guard = _time.sleep, threading.Lock()
    state = {'focus': None, 'fs': {'0xa': 0, '0xb': 0}, 'pressed': {'0xa': 0, '0xb': 0}}

    def hypr(cmd):
        with guard:
            if cmd == 'j/clients':
                return json.dumps([{'pid': 1, 'address': '0xa', 'fullscreen': state['fs']['0xa']},
                                   {'pid': 2, 'address': '0xb', 'fullscreen': state['fs']['0xb']}])
            if cmd == 'j/monitors':
                return json.dumps([{'name': 'eDP-1', 'activeWorkspace': {'id': 1}},
                                   {'name': 'DP-2', 'activeWorkspace': {'id': 2}}])
            if 'hl.dsp.focus' in cmd:
                state['focus'] = '0xa' if '0xa' in cmd else '0xb'
            elif 'window.fullscreen' in cmd:
                state['fs'][state['focus']] ^= 1
            elif 'Shift_L' in cmd:
                # Delivered only to the focused window, whatever it names.
                state['pressed'][state['focus']] += 1
        if 'hl.dsp.focus' in cmd:
            real_sleep(0.05)          # the other window's turn comes here
        return 'ok'
    monkeypatch.setattr(mod, 'hyprland', hypr)
    monkeypatch.setattr(mod.time, 'sleep', lambda s: real_sleep(0.001))
    with mod.concurrent.futures.ThreadPoolExecutor() as pool:
        assert all(pool.map(lambda w: mod.place(*w), [(1, 'eDP-1'), (2, 'DP-2')]))
    assert state['fs'] == {'0xa': 1, '0xb': 1}
    assert state['pressed'] == {'0xa': len(mod.PRESS_AT_S), '0xb': len(mod.PRESS_AT_S)}
