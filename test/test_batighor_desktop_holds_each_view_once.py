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
