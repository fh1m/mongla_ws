"""Our panels reach every browser from Kutum, and say the helm's own words.

Lichtblick's side of each shape is its 1.30.0 source, `api/extensions/` and
`providers/ExtensionCatalogProvider/utils.ts`; the bundle switch is the line
read from the pinned release's `main.<hash>.js`.
"""
import io
import json
import re
import shutil
import subprocess
import threading
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

import pytest

from mongla_batighor import panels, seat, serve

PANELS = Path(__file__).resolve().parents[1] / 'panels'
HELM_JS = (PANELS / 'helm' / 'extension.js').read_text()
SWITCH = b'const r={apiUrl:void 0,version:"1.30.0",devWorkspace:""};'


def _web(tmp, main=b'x;' + SWITCH + b'y;'):
    web = tmp / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<script src="main.0123456789abcdef01.js"></script>')
    (web / 'main.0123456789abcdef01.js').write_bytes(main)
    return web


def _get(url, method='GET'):
    req = urllib.request.Request(url, method=method, data=b'{}' if method != 'GET' else None)
    try:
        r = urllib.request.urlopen(req)
        return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


@pytest.fixture
def kutum(tmp_path):
    layouts = tmp_path / 'layouts'
    layouts.mkdir()
    (layouts / 'mongla_layout.json').write_text('{}')
    srv = serve.make_server(0, _web(tmp_path), layouts, 8765, panels_dir=PANELS)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()


def test_the_app_is_served_with_the_api_on_under_a_new_name(kutum):
    _, _, html = _get(kutum + '/app/')
    assert b'main.0123456789abcdef01.api.js' in html
    status, headers, js = _get(kutum + '/app/main.0123456789abcdef01.api.js')
    assert status == 200 and b'apiUrl:"/api",version:"1.30.0"' in js
    assert b'apiUrl:void 0' not in js and 'immutable' in headers['Cache-Control']
    assert 'workspace=mongla' in serve.app_link('mongla.local:8000', 8765, 'x')


def test_a_release_that_moved_the_switch_is_refused_not_half_served(tmp_path):
    for main in (b'nothing here', SWITCH + SWITCH):
        web = tmp_path / str(len(main))
        web.mkdir()
        (web / 'index.html').write_text('<script src="main.0123456789abcdef01.js">')
        (web / 'main.0123456789abcdef01.js').write_bytes(main)
        with pytest.raises(ValueError, match='expected once'):
            panels.api_bundle(web)


def test_the_workspace_lists_the_helm_and_hands_over_its_foxe(kutum):
    _, _, body = _get(kutum + '/api/workspaces/mongla/extensions')
    listed = json.loads(body)['data']
    assert [e['id'] for e in listed] == ['mongla.batighor-calibrate', 'mongla.batighor-helm']
    e = listed[1]
    # Lichtblick caches an org extension under `${publisher}.${name}` and
    # compares that with the listed id: they must be the same string.
    assert e['extensionId'] == f"{e['publisher']}.{e['name']}" and e['scope'] == 'org'
    assert json.loads(_get(kutum + f"/api/extensions/{e['id']}")[2])['data'] == e
    status, _, foxe = _get(kutum + f"/api/extensions/{e['id']}/download")
    z = zipfile.ZipFile(io.BytesIO(foxe))
    pkg = json.loads(z.read('package.json'))
    assert pkg['version'] == e['version'] and pkg['name'] == e['name']
    code = z.read('dist/extension.js').decode()
    assert code.startswith('const VERBS = ') and 'registerPanel' in code
    assert 'const TUNABLE = {"conf": "float"' in code and 'const CAMERAS = ["forward"' in code
    org = json.loads(_get(kutum + '/api/workspaces/mongla/layouts')[2])['data']
    assert [(l['name'], l['permission'], l['data']) for l in org] == \
        [('mongla_layout', 'ORG_READ', {})]
    favs = json.loads(_get(kutum + '/api/profile/favorite-layouts')[2])['data']
    assert favs == {'favoriteLayouts': []}


def test_every_verb_reaches_the_panel_with_its_fields_and_types():
    from mongla_control.commands import COMMANDS
    v = panels.verbs()
    assert set(v) == set(COMMANDS)
    assert v['set_mode']['fields']['target_name'] == 'string'
    assert v['move_forward']['fields']['duration'] == 'float'


def test_a_changed_panel_is_a_new_version_so_a_browser_fetches_it(tmp_path):
    d = tmp_path / 'helm'
    d.mkdir()
    (d / 'extension.js').write_text('a')
    v1 = panels.foxe(d)[0]['version']
    (d / 'extension.js').write_text('b')
    assert panels.foxe(d)[0]['version'] != v1


def test_kutum_refuses_a_browser_writing_to_the_workspace(kutum):
    for m in ('POST', 'PUT', 'DELETE'):
        assert _get(kutum + '/api/workspaces/mongla/extension', m)[0] == 403


def test_the_panel_says_the_helms_own_words():
    """One truth: the panel's copies of the helm's names are the helm's."""
    from mongla_batighor import helm
    helm_src = Path(helm.__file__).read_text()
    for op in set(re.findall(r'call\("(\w+)"', HELM_JS)):
        assert f"op == '{op}'" in helm_src, op
    anyone = re.search(r'const ANYONE = \[([^\]]*)\]', HELM_JS).group(1)
    assert set(re.findall(r'"(\w+)"', anyone)) == seat.ANYONE
    assert set(re.findall(r'confirms\.push\("(\w+)"\)', HELM_JS)) == seat.ARM_CONFIRMS
    for topic in re.findall(r'"(/mongla/[\w/]+)"', HELM_JS):
        assert f"'{topic}'" in helm_src or topic == '/mongla/state', topic


def test_the_main_layout_carries_the_helm():
    layout = json.loads((PANELS.parent / 'layouts' / 'mongla_layout.json').read_text())
    assert 'mongla.batighor.Helm!helm' in layout['configById']
    assert 'mongla.batighor.Helm!helm' in json.dumps(layout['layout'])


CALIBRATE_JS = (PANELS / 'calibrate' / 'extension.js').read_text()


def test_the_calibrate_panel_says_the_calibration_nodes_own_words():
    """Its buttons are the node's ops, sent through the helm's one op for them,
    and it reads the topic the node publishes."""
    from mongla_batighor import helm
    from mongla_vision.calibration import calibrate_node
    sent = set(re.findall(r'call\("(\w+)"', CALIBRATE_JS))
    steps = re.search(r'const STEPS = \[([^\]]*)\]', CALIBRATE_JS).group(1)
    sent |= set(re.findall(r'"(\w+)"', steps))
    assert sent == set(calibrate_node.OPS)
    assert 'op: "calibrate"' in CALIBRATE_JS and "op == 'calibrate'" in Path(helm.__file__).read_text()
    assert f'"{calibrate_node.NS}/state"' in CALIBRATE_JS
    assert re.search(r'NAME_KEY = "([^"]+)"', CALIBRATE_JS).group(1) == \
        re.search(r'NAME_KEY = "([^"]+)"', HELM_JS).group(1)


def test_the_calibrate_layout_draws_the_overlay_on_the_calibration_camera():
    from mongla_vision.calibration import calibrate_node
    layout = json.loads((PANELS.parent / 'layouts' / 'calibrate.json').read_text())
    image = layout['configById']['Image!camera']['imageMode']
    assert image['imageTopic'] == f'/mongla/vision/{calibrate_node.CAMERA}/stream/compressed'
    assert f'{calibrate_node.NS}/annotations' in image['annotations']
    assert 'mongla.batighor.Calibrate!calibrate' in json.dumps(layout['layout'])


@pytest.mark.skipif(not shutil.which('node'), reason='node not installed (the Pi, CI)')
@pytest.mark.parametrize('name', sorted(p.name for p in PANELS.iterdir()
                                        if (p / 'extension.js').is_file()))
def test_every_served_panel_parses(tmp_path, name):
    """A syntax error anywhere makes the whole panel inert, and it looks like
    a panel that stopped updating (camera-and-calibration.md §10). The code
    is checked as served: preamble and all."""
    ext = panels.extensions(PANELS)[f'{panels.PUBLISHER}.batighor-{name}']
    code = zipfile.ZipFile(io.BytesIO(ext[1])).read('dist/extension.js')
    f = tmp_path / 'extension.js'
    f.write_bytes(code)
    r = subprocess.run(['node', '--check', str(f)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
