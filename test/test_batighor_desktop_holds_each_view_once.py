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
    assert 'WARNING: thrust changed' in out and 'delete thrust' in out
    assert json.loads((desk.LAYOUTS / 'thrust.json').read_text())['v'] == 2


def test_a_dense_screen_starts_the_app_at_a_scale_it_can_be_read_at(desk):
    """A 4K 15-inch panel at device scale 1 drew 12 px type 1 mm tall."""
    laptop = ('HDMI-A-1 connected 1920x1080+7680+0 (normal) 480mm x 260mm\n'
              'eDP-1 connected primary 3840x2160+0+0 (normal) 340mm x 190mm\n')
    assert desk.screen_scale(laptop) == 2.5
    assert desk.screen_scale('DP-1 connected 1920x1080+0+0 (normal) 527mm x 296mm') == 1.0
    assert desk.screen_scale('eDP-1 connected 1920x1080+0+0 (normal) 0mm x 0mm') is None
    assert desk.main(['h']) == 0
    assert not any(a.startswith('--force-device-scale-factor') for a in desk.launches[0])
    assert desk.main(['h', '--scale', '2']) == 0
    assert '--force-device-scale-factor=2' in desk.launches[1]
