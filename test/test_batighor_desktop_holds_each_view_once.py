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
VIEWS = ['board_health', 'mongla_layout', 'thrust']


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
            return json.dumps({'data': [{'name': v} for v in VIEWS]}).encode()
        return json.dumps(mod.served[url.rsplit('/', 1)[-1].removesuffix('.json')]).encode()
    monkeypatch.setattr(mod, 'get', get)
    monkeypatch.setattr(mod, 'install', lambda base: [])
    monkeypatch.setattr(mod.shutil, 'which', lambda _: '/bin/true')
    monkeypatch.setattr(mod, 'running', lambda: False)
    monkeypatch.setattr(mod, 'vulkan_flag', lambda: [])  # this machine's GPU is not the test's
    monkeypatch.setenv('GDK_SCALE', '1')        # the desktop scales: no flag of ours
    mod.launches = []
    monkeypatch.setattr(mod.subprocess, 'Popen', lambda argv, **_: mod.launches.append(argv))
    return mod


def test_a_fresh_app_opens_on_the_cockpit(desk):
    assert desk.main(['h']) == 0
    argv = desk.launches[0]
    assert '--defaultLayout=mongla_layout' in argv
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


def test_a_dense_screen_starts_the_app_at_a_scale_it_can_be_read_at(desk):
    """A 4K 15-inch panel at device scale 1 drew 12 px type 1 mm tall; at 2
    the console reads comfortably on it (the desktop app's working scale)."""
    laptop = ('HDMI-A-1 connected 1920x1080+7680+0 (normal) 480mm x 260mm\n'
              'eDP-1 connected primary 3840x2160+0+0 (normal) 340mm x 190mm\n')
    assert desk.screen_scale(laptop) == 2.0         # 287 dpi against READ_DPI 140
    assert desk.screen_scale('DP-1 connected 1920x1080+0+0 (normal) 527mm x 296mm') == 1.0
    assert desk.screen_scale('eDP-1 connected 1920x1080+0+0 (normal) 0mm x 0mm') is None
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
