"""Our panels reach every browser from Kutum, and say the helm's own words.

Lichtblick's side of each shape is its 1.30.0 source, `api/extensions/` and
`providers/ExtensionCatalogProvider/utils.ts`; the bundle switch is the line
read from the pinned release's `main.<hash>.js`.
"""
import pathlib
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
HELM_JS = (PANELS / 'control' / 'extension.js').read_text()   # the Control Center
SWITCH = b'const r={apiUrl:void 0,version:"1.30.0",devWorkspace:""};'
# The panels' tests run their JavaScript in node; CI's Humble image has node 12,
# older than the syntax the panels use, so they run on the dev box.
NEEDS_NODE = pytest.mark.skipif(not shutil.which('node'), reason='node not installed (the Pi, CI)')


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
    assert 'workspace=mongla' in serve.app_link('kutum.local:8000', 8765, 'x')


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
    assert [e['id'] for e in listed] == ['mongla.batighor-calibrate', 'mongla.batighor-control',
                                         'mongla.batighor-instruments']
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


def test_the_main_layout_carries_the_control_center():
    layout = json.loads((PANELS.parent / 'layouts' / 'mongla_layout.json').read_text())
    assert 'mongla.batighor.ControlCenter!control' in layout['configById']
    assert 'mongla.batighor.ControlCenter!control' in json.dumps(layout['layout'])
    assert re.search(r'^  ControlCenter: \[', HELM_JS, re.M) and 'registerPanel({ name, initPanel' in HELM_JS
    assert 'ControlCenter: "Control Center"' in HELM_JS and 'setDefaultPanelTitle(TITLE[kind])' in HELM_JS


def test_stop_surface_and_disarm_are_first_and_never_scroll_away():
    """In a small window the helm card above them pushed STOP below the fold."""
    cards = re.findall(r'card\(box, "([^"]+)"(?:, [^,)]*)?(?:, "(\w+)")?\)', HELM_JS)
    assert cards[0] == ('anyone, one press', 'pinned'), cards
    css = (PANELS.parent / 'theme' / 'batighor.css').read_text()
    pinned = re.search(r'\.mb section\.pinned\{([^}]*)\}', css)
    # -10px: flush over .mb's own 10px padding, or the scrolled card shows above it.
    assert pinned and 'position:sticky' in pinned.group(1) and 'top:-10px' in pinned.group(1)


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
    assert 'op: "calibrate"' in CALIBRATE_JS
    assert "op == 'calibrate'" in Path(helm.__file__).read_text()
    assert f'"{calibrate_node.NS}/state"' in CALIBRATE_JS
    assert 'readName()' in CALIBRATE_JS and 'readName()' in HELM_JS   # one name per browser


def test_the_calibrate_layout_draws_the_overlay_on_the_calibration_camera():
    from mongla_vision.calibration import calibrate_node
    layout = json.loads((PANELS.parent / 'layouts' / 'calibrate.json').read_text())
    image = layout['configById']['Image!camera']['imageMode']
    assert image['imageTopic'] == f'/mongla/vision/{calibrate_node.CAMERA}/stream/compressed'
    assert f'{calibrate_node.NS}/annotations' in image['annotations']
    assert 'mongla.batighor.Calibrate!calibrate' in json.dumps(layout['layout'])


@NEEDS_NODE
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


INSTRUMENTS_JS = (PANELS / 'instruments' / 'extension.js').read_text()


def _instruments():
    """name -> its topics, as the table declares them (CAMERAS expanded)."""
    from mongla_batighor.seat import CAMERAS
    out = {}
    for name, body in re.findall(r'^  (\w+): \{\n(.*?)^  \},$', INSTRUMENTS_JS, re.M | re.S):
        topics = re.search(r'topics: (\[.*?\]|[A-Z_]+),\n', body, re.S).group(1)
        # A shared list (`topics: MAP_TOPICS`, or `...MAP_TOPICS` inside one)
        # is read from its own declaration: one copy, parsed as written.
        for const in re.findall(r'\b([A-Z][A-Z_]+)\b', topics):
            decl = re.search(rf'^const {const} = \[(.*?)\];', INSTRUMENTS_JS, re.M | re.S)
            if decl:
                topics += decl.group(1)
        names = set(re.findall(r'"(/[\w/]+)"', topics))
        for tmpl in re.findall(r'(lockTopic|seeingTopic)\(c\)', topics):
            names |= {f'/mongla/vision/{c}/{tmpl[:-5]}' for c in CAMERAS}
        out[name] = names
    return out


def test_every_instrument_reads_only_topics_that_exist_and_small_ones():
    """Every topic an instrument names is published by a node here, and none
    is an image or a detection stream: opening one must cost the vehicle
    almost nothing (the owner's rule: observation must not slow the vehicle)."""
    src = '\n'.join(p.read_text(errors='ignore') for p in PANELS.parents[1].rglob('*.py')
                    if '/test/' not in str(p))
    table = _instruments()
    assert set(table) >= {'Vehicle', 'Vision', 'Navigation', 'Thrust', 'Link', 'Health', 'Kutum'}
    for name, topics in table.items():
        assert topics, name
        for topic in topics:
            leaf = topic.rsplit('/', 1)[-1]
            # An action's feedback topic is published by its server, which is
            # named by the action, not the topic.
            action = topic.split('/_action/')[0] if '/_action/' in topic else None
            assert (f"'{topic}'" in src or topic == '/diagnostics'
                    or (action and f"'{action}'" in src)
                    or re.search(rf"f'[^']*{{ns(?:_out)?}}/{leaf}'", src)), (name, topic)
            for heavy in ('image', 'compressed', 'detections', 'candidates', 'flow_tracks'):
                assert heavy not in topic, (name, topic)


def test_every_instrument_the_views_place_exists():
    """views.py names an instrument by its registered name; a typo there is a
    panel Lichtblick cannot find, drawn as an empty box."""
    from mongla_batighor import views
    control = (PANELS / 'control' / 'extension.js').read_text()
    parts = re.search(r'const PARTS = \{(.*?)\n\};', control, re.S).group(1)
    ours = set(_instruments()) | set(re.findall(r'^  (\w+): \[', parts, re.M)) | {'Calibrate'}
    assert {'ControlCenter', 'Detectors', 'Keys'} <= ours
    for name in views.VIEWS:
        for pid in views.build(name)['configById']:
            if pid.startswith(views.PANEL):
                assert pid[len(views.PANEL):].split('!')[0] in ours, (name, pid)


def test_the_vision_instrument_names_the_ladder_as_the_lock_does():
    from mongla_vision.tracking.lock_state import Rung
    rungs = re.search(r'const RUNGS = \[([^\]]*)\]', INSTRUMENTS_JS).group(1)
    assert re.findall(r'"(\w+)"', rungs) == [r.value for r in Rung]


# -- the look -----------------------------------------------------------

THEME = PANELS.parent / 'theme' / 'batighor.css'
FONTS = PANELS.parents[2] / 'docs' / 'assets' / 'fonts'


@pytest.mark.parametrize('name', sorted(p.name for p in PANELS.iterdir()
                                        if (p / 'extension.js').is_file()))
def test_a_panel_carries_no_look_and_no_helper_of_its_own(name):
    """One stylesheet and one helper file: three panels each with their own
    CSS and their own `el` drifted apart in looks and in behaviour."""
    js = (PANELS / name / 'extension.js').read_text()
    for own in ('<style', 'el("style"', 'const CSS', 'function el(', 'NAME_KEY =',
                'const num =', 'const fin ='):
        assert own not in js, f'{name} defines {own!r}; it belongs in common.js or the theme'
    assert 'mount(context' in js
    served = panels.extensions(PANELS)[f'{panels.PUBLISHER}.batighor-{name}'][1]
    assert b'function el(' in zipfile.ZipFile(io.BytesIO(served)).read('dist/extension.js')


def test_the_theme_is_the_sites_palette_and_fonts():
    """The tokens are the site's own values, and every font it asks for is a
    file the site ships and setup.py installs."""
    site = (FONTS.parent / 'site.css').read_text()
    css = THEME.read_text()
    for token in ('d0', 'd1', 'd2', 'd3', 'rule', 'rule-lit', 'ink', 'ink-2', 'ink-3'):
        want = re.search(rf'--{token}:(#[0-9a-f]+)', site).group(1)
        assert f'--mb-{token}:{want};' in css, token
    assert re.search(r'--red-lit:(#[0-9a-f]+)', site).group(1) in css
    assert re.search(r'--sea-lit:(#[0-9a-f]+)', site).group(1) in css
    asked = set(re.findall(r'/theme/fonts/([\w-]+)\.woff2', css))
    setup = (PANELS.parent / 'setup.py').read_text()
    fonts = re.search(r'THEME_FONTS = \(([^)]*)\)', setup).group(1)
    installed = set(re.findall(r"'([\w-]+)'", fonts))
    assert asked == installed and asked
    for f in asked:
        assert (FONTS / f'{f}.woff2').is_file(), f


def test_the_app_page_links_the_theme_and_kutum_serves_it(tmp_path):
    theme = tmp_path / 'theme'
    (theme / 'fonts').mkdir(parents=True)
    (theme / 'batighor.css').write_text(THEME.read_text())
    (theme / 'fonts' / 'zed-mono-extended.woff2').write_bytes(b'wOF2')
    (tmp_path / 'secret.css').write_text('x')
    layouts = tmp_path / 'layouts'
    layouts.mkdir()
    web = _web(tmp_path)
    (web / 'index.html').write_text('<html><head><title>x</title></head>'
                                    '<script src="main.0123456789abcdef01.js"></script></html>')
    srv = serve.make_server(0, web, layouts, 8765, panels_dir=PANELS)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{srv.server_address[1]}'
    try:
        _, _, html = _get(base + '/app/')
        assert html.count(b'/theme/batighor.css') == 1
        assert html.index(b'/theme/batighor.css') < html.index(b'</head>')
        status, headers, css = _get(base + '/theme/batighor.css')
        assert status == 200 and b'--mb-ink' in css and headers['Cache-Control'] == 'no-cache'
        status, headers, _ = _get(base + '/theme/fonts/zed-mono-extended.woff2')
        assert status == 200 and 'immutable' in headers['Cache-Control']
        for bad in ('/theme/../secret.css', '/theme/%2e%2e/secret.css',
                    '/theme/x/../../secret.css'):
            assert _get(base + bad)[0] == 404, bad
    finally:
        srv.shutdown()


def test_the_installed_share_carries_everything_serve_reads():
    """The tests above read the source tree; Kutum serves the installed one.
    A file setup.py forgot (panels/common.js, 2026-11-12) killed the server at
    start on Kutum while every test here passed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('mongla_batighor'))
    except Exception:                                   # noqa: BLE001
        pytest.skip('mongla_batighor is not installed')
    for src in [*PANELS.glob('*.js'), *PANELS.glob('*/extension.js'), THEME,
                *(PANELS.parent / 'layouts').glob('*.json')]:
        rel = src.relative_to(PANELS.parent)
        assert (share / rel).is_file(), f'{rel} is not installed: add it to setup.py'
    css = THEME.read_text()
    for f in re.findall(r'/theme/(fonts/[\w-]+\.woff2)', css):
        assert (share / 'theme' / f).is_file(), f
    panels.extensions(share / 'panels')               # what serve.py does at start
    assert (share / 'model' / 'hull.mgla').is_file(), 'the hull: add it to setup.py'
    assert (share / 'model' / 'hull_geometry.yaml').is_file(), 'the thrusters: add it to setup.py'


def test_one_layout_opens_our_panels_in_the_web_and_the_desktop_app():
    """The web app names a panel `<qualifiedName>.<panel>` from the API entry,
    the desktop app (1.27) `<package.json displayName>.<panel>`; a layout
    names `mongla.batighor.<panel>`. Seen on the desktop 2026-11-12:
    "Unknown panel type: mongla.batighor.Navigation"."""
    for entry, data in panels.extensions(PANELS).values():
        pkg = json.loads(zipfile.ZipFile(io.BytesIO(data)).read('package.json'))
        assert entry['qualifiedName'] == pkg['displayName'] == panels.QUALIFIED
        # The description is the panel's own first line, not the preamble.
        assert not pkg['description'].startswith('const') and len(pkg['description']) < 200
    for f in (PANELS.parent / 'layouts').glob('*.json'):
        for pid in json.loads(f.read_text())['configById']:
            if pid.startswith('mongla.'):
                assert pid.startswith(panels.QUALIFIED + '.'), (f.name, pid)


def test_a_page_that_does_not_link_the_theme_gets_it_from_the_panel():
    """The desktop app's page is not Kutum's: the sheet travels in the panel."""
    for _, data in panels.extensions(PANELS).values():
        code = zipfile.ZipFile(io.BytesIO(data)).read('dist/extension.js').decode()
        assert f'const THEME_CSS = {json.dumps(THEME.read_text())};' in code
    common = (PANELS / 'common.js').read_text()
    assert re.search(r'function mount\([^)]*\) \{\s*theme\(\);', common)


def test_every_perception_stage_is_a_topic_a_node_publishes():
    """A stage reads `on` while its topic has a publisher; a renamed topic would
    read `off` forever while the stage ran."""
    import pathlib
    from mongla_batighor.overlay import STAGES
    src = pathlib.Path(__file__).resolve().parents[2]
    code = ''.join(p.read_text() for pkg in ('mongla_vision', 'mongla_batighor')
                   for p in (src / pkg / pkg).rglob('*.py'))
    for _, rel, _, _ in STAGES:
        assert re.search(r"\}/" + re.escape(rel) + r"[_'\"{]", code), rel


def test_a_stage_is_on_only_while_a_node_publishes_it():
    """The graph lists a topic a subscriber alone asked for: that is not a stage running."""
    from mongla_batighor.overlay import stages
    ns = '/mongla/vision/forward/'
    names = [ns + 'lock', ns + 'detections', ns + 'target_pose_near']
    pubs = {ns + 'detections': 1, ns + 'lock': 0, ns + 'target_pose_near': 1}
    st = {x['stage']: x for x in stages('forward', names, lambda t: pubs.get(t, 0))}
    assert st['detector']['on'] and st['pose (PnP)']['on']
    assert not st['lock ladder']['on'] and st['lock ladder']['switch'] == 'lock:=true'
    assert 'flow' not in st, 'flow runs on the downward camera only'
    dwn = {x['stage'] for x in stages('downward', [], lambda t: 0)}
    assert 'flow' in dwn


def test_every_stage_names_a_switch_bringup_takes():
    """An operator types the switch the panel shows; one bringup does not take
    is a dead end at the pool (`vision_stack` once stood for pose)."""
    import pathlib
    from mongla_batighor.overlay import STAGES
    bringup = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_manager' / 'launch'
               / 'bringup.launch.py').read_text()
    for stage, _, switch, _ in STAGES:
        name, _, value = switch.partition(':=')
        assert value == 'true' and f"'{name}'" in bringup, (stage, switch)


def test_the_vehicle_panel_calls_a_mission_running_as_the_helm_does():
    """One silence for "no mission", in the helm and on the operator's screen."""
    import pathlib
    from mongla_batighor import helm
    js = (pathlib.Path(__file__).resolve().parents[1] / 'panels' / 'instruments' / 'extension.js').read_text()
    (limit,) = re.findall(r'const MISSION_ALIVE_S = ([0-9.]+);', js)
    assert float(limit) == helm.MISSION_ALIVE_S
    # Every instrument that says a mission runs uses that one constant.
    uses = re.findall(r'ageS\(s\.aliveAt\) < ([\w.]+)', js)
    assert len(uses) >= 2 and set(uses) == {'MISSION_ALIVE_S'}, uses


def test_what_an_unset_setting_means_is_the_detectors_own_default():
    """The Control Center says what a default does (`UNSET`), and each value it
    calls the default is the one the detector declares."""
    import pathlib
    import re as _re
    from mongla_batighor.seat import TUNABLE
    root = pathlib.Path(__file__).resolve().parents[2]
    js = (root / 'mongla_batighor' / 'panels' / 'control' / 'extension.js').read_text()
    setup = (root / 'mongla_vision' / 'mongla_vision' / 'detector_setup.py').read_text()
    block = _re.search(r'const UNSET = \{(.*?)\n  \};', js, _re.S).group(1)
    rows = _re.findall(r'(\w+): \[("[^"]*"|[0-9.]+), "', block)
    assert len(rows) >= 3
    for name, unset in rows:
        assert name in TUNABLE, name
        (declared,) = _re.findall(rf"declare_parameter\('{name}',\s*([^)]+)\)", setup)
        assert float(declared) == float(unset) if TUNABLE[name] is float \
            else declared.strip().strip("'") == unset.strip('"'), (name, declared, unset)


@NEEDS_NODE
def test_the_desktop_app_takes_kutums_layout_unless_the_operator_edited_it():
    """layoutUpdates: a stale copy from one of Kutum's files is replaced, an
    edited one (`working`) is kept, a current one and one of the operator's
    own (no `from`) are untouched."""
    import json as _json
    import subprocess as _sp
    src = (PANELS / 'common.js').read_text() + INSTRUMENTS_JS
    rec = lambda name, data, working=None, frm=True: {  # noqa: E731
        'namespace': 'local', 'layout': {'id': name, 'name': name,
                                         **({'from': f'{name}.json'} if frm else {}),
                                         'baseline': {'data': data, 'savedAt': 'old'},
                                         'working': working}}
    console = lambda v, tab: {'v': v, 'configById': {  # noqa: E731
        'Tab!console': {'activeTabIdx': tab, 'tabs': [{}, {}, {}]}}}
    cam3d = lambda d, th: {'configById': {'3D!hull': {  # noqa: E731
        'followTf': 'mongla', 'cameraState': {'distance': d, 'thetaOffset': th}}}}
    records = [rec('map', {'v': 1}), rec('flow', {'v': 1}, working={'data': {}}),
               rec('vision', {'v': 2}), rec('mine', {'v': 1}, frm=False),
               # Only a tab chosen since: not an edit, and the choice survives.
               rec('batighor', console(1, 0), working={'data': console(1, 2)}),
               # The app's own defaults written in on mount: not an edit either,
               # and kept beneath Kutum's values.
               rec('cockpit', {'configById': {'Plot!a': {'paths': [1]}}},
                   working={'data': {'configById': {'Plot!a': {'paths': [1], 'showXAxisLabels': True}}}}),
               # The 3D panel writes its camera back as it renders (an angle
               # of 200 comes back as -160, a distance creeps): a view, not an
               # edit. Counted as one, it kept the console off Kutum's layout
               # for good (seen on the desktop app, 2026-11-22).
               rec('hull', cam3d(2.2, 200), working={'data': cam3d(2.316, -160)}),
               # A value the file set, changed: an edit, kept.
               rec('thrust', {'configById': {'Plot!a': {'paths': [1]}}},
                   working={'data': {'configById': {'Plot!a': {'paths': [2]}}}})]
    served = {'map': {'v': 2}, 'flow': {'v': 2}, 'vision': {'v': 2}, 'mine': {'v': 2},
              'batighor': console(2, 0), 'cockpit': {'configById': {'Plot!a': {'paths': [9]}}},
              'thrust': {'configById': {'Plot!a': {'paths': [9]}}},
              'hull': cam3d(1.3, 200)}
    js = (src + '\nprocess.stdout.write(JSON.stringify(module.exports.layoutUpdates('
          + _json.dumps(records) + ',' + _json.dumps(served) + ',"now")));')
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    out = _sp.run(['node', '-e', pre + js],
                  capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    got = _json.loads(out.stdout)
    assert [r['layout']['name'] for r in got['update']] == ['map', 'batighor', 'cockpit', 'hull']
    # Kutum's camera, not the one the panel drifted to: the file changed it.
    assert got['update'][3]['layout']['baseline']['data'] == cam3d(1.3, 200)
    assert got['update'][2]['layout']['baseline']['data'] == {
        'configById': {'Plot!a': {'paths': [9], 'showXAxisLabels': True}}}
    assert got['update'][0]['layout']['baseline'] == {'data': {'v': 2}, 'savedAt': 'now'}
    console_now = got['update'][1]['layout']
    assert console_now['baseline']['data'] == console(2, 2) and 'working' not in console_now
    assert got['edited'] == ['flow', 'thrust']


@NEEDS_NODE
def test_the_status_strip_says_what_it_sees_and_marks_the_tabs():
    """statusOf: a held lock names its rung, an unknown health row is not a
    fault, and an armed vehicle marks the cockpit live."""
    import json as _json
    import subprocess as _sp
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    lock = {'detections': [{'id': 'gate', 'results': [{'hypothesis': {'class_id': 'held', 'score': 0.8}}]}]}
    s = {'v': {'armed': True, 'mode': 'AUTO', 'depth_m': -1.2, 'yaw_deg': 90, 'battery_voltage': 14.8},
         'vAt': 1000, 'locks': {'forward': {'m': lock, 'at': 1000}, 'downward': {'m': {}, 'at': 1000}},
         'health': [{'name': 'baro', 'level': 3, 'at': 1000}, {'name': 'imu', 'level': 0, 'at': 1000}]}
    js = (pre + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS
          + f'\nprocess.stdout.write(JSON.stringify(module.exports.statusOf({_json.dumps(s)}, 1500)));')
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    got = _json.loads(out.stdout)
    cells = {k: (v, lvl) for k, v, lvl, *_ in got['cells']}
    assert cells['lock'] == ('forward held', 'live')
    assert cells['health'] == ('1 unknown', 'stale')
    assert cells['vehicle'] == ('ARMED', 'armed') and cells['depth'][0] == '1.20 m deep'
    assert got['tabs']['Cockpit'] == 'live' and got['tabs']['Vision'] == 'live'
    # A view whose feed has gone quiet (a starved link, 2026-11-23) must not
    # read as calm: the last word stays, with its age, as a warning.
    js_old = js.replace('statusOf(' + _json.dumps(s) + ', 1500)', 'statusOf(' + _json.dumps(s) + ', 4600)')
    old = _sp.run(['node', '-e', js_old], capture_output=True, text=True, timeout=30)
    assert old.returncode == 0, old.stderr
    cells = {k: (v, lvl) for k, v, lvl, *_ in _json.loads(old.stdout)['cells']}
    assert cells['vehicle'] == ('ARMED  3.6 s old', 'warn'), cells['vehicle']


@NEEDS_NODE
def test_the_mission_text_marks_the_line_that_sent_the_verb():
    """missionText: every line numbered, the running one marked, the mission
    named live while its runner beats and 'ended' after; nothing published is
    said as such, never as an empty page."""
    import json as _json
    import subprocess as _sp
    shim = ('const document={createElement:(t)=>({t,className:"",kids:[],attrs:{},'
            'append(...k){this.kids.push(...k)},setAttribute(k,v){this.attrs[k]=v},'
            'addEventListener(){}})};'
            'const text=(e)=>typeof e==="string"?e:(e.kids||[]).map(text).join("");'
            'const flat=(e)=>typeof e==="string"?[]:[e,...(e.kids||[]).flatMap(flat)];')
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    src = {'name': 'gate_run', 'file': '/x/missions/gate_run.py', 'first_line': 3,
           'text': 'import x\n\ndef run(m, log):\n    m.arm()\n    m.move_forward(duration=4)\n'}
    probe = ('\nconst I=module.exports.INSTRUMENTS.MissionText, s={};'
             'const out=[];'
             'out.push(I.draw(s).map(text).join("|"));'
             f'I.fold(s,"/mongla/mission/source",{{data:{_json.dumps(_json.dumps(src))}}});'
             'I.fold(s,"/mongla/mission_alive",{data:"gate_run"});'
             'I.fold(s,"/mongla/mission/line",{data:JSON.stringify({line:5,cmd:"move_forward"})});'
             'let d=I.draw(s);'
             'const rows=flat(d[1]).filter((e)=>/\\bln\\b/.test(e.className));'
             'out.push(rows.length, rows.filter((e)=>/now/.test(e.className)).map(text).join(""),'
             '  rows.filter((e)=>/entry/.test(e.className)).map(text).join(""), text(d[0]));'
             's.aliveAt=Date.now()-60000; out.push(text(I.draw(s)[0]));'
             'process.stdout.write(JSON.stringify(out));')
    js = pre + shim + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS + probe
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    empty, n, now, entry, head, ended = _json.loads(out.stdout)
    assert 'no /mongla/mission/source' in empty
    assert n == 6                                        # five lines and the trailing newline
    assert now == '5    m.move_forward(duration=4)'
    assert entry == '3def run(m, log):'
    assert 'gate_run' in head and 'line 5 sent move_forward' in head and 'ended' not in head
    assert 'gate_run.py, ended' in ended


@NEEDS_NODE
def test_a_panel_that_mounts_late_is_handed_what_came_before_at_its_own_age():
    """Lichtblick hands a panel only what arrives after it mounts, so the
    latched course reached the first Map and never a second one (seen in the
    desktop app: the Mission view's map waiting for a course the Map view
    showed). A later panel is handed the last message, stamped with when it
    came, so a beat a minute old does not read as a mission running."""
    import json as _json
    import subprocess as _sp
    shim = ('const document={createElement:(t)=>({t,className:"",kids:[],attrs:{},'
            'append(...k){this.kids.push(...k)},replaceChildren(...k){this.kids=k},'
            'setAttribute(k,v){this.attrs[k]=v},addEventListener(){},querySelector(){return null}}),'
            'querySelector:()=>true,head:{append(){}}};'
            'const requestAnimationFrame=(f)=>0,cancelAnimationFrame=()=>{};'
            'const intervals=[];globalThis.setInterval=(f)=>{intervals.push(f);return 0};'
            'globalThis.clearInterval=()=>{};')
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    probe = ('\nconst X=module.exports, spec=X.INSTRUMENTS.Mission;'
             'const ctx=()=>({panelElement:{append(){}},subscribe(){},watch(){}});'
             'const a=ctx(); X.makePanel(spec)(a);'
             'const doing={verb:"surface",fields:{},since:1};'
             'a.onRender({currentFrame:[{topic:"/mongla/doing",message:{data:JSON.stringify({now:doing,recent:[],stamp:2})}},'
             '{topic:"/mongla/mission_alive",message:{data:"gate_run"}}]},()=>{});'
             'const at=X.LAST.get("/mongla/mission_alive").at;'
             'X.LAST.get("/mongla/mission_alive").at=at-60000;'
             'const spy={}; const s2=Object.assign({},spec,{fold(s,t,m){spy.s=s;spec.fold(s,t,m)}});'
             'X.makePanel(s2)(ctx());'
             'process.stdout.write(JSON.stringify([spy.s.doing&&spy.s.doing.now.verb,'
             ' Math.round((Date.now()-spy.s.aliveAt)/1000)]));')
    js = pre + shim + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS + probe
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    verb, alive_age = _json.loads(out.stdout)
    assert verb == 'surface'                 # the second panel has /mongla/doing
    assert alive_age >= 59                   # and the old beat reads as old


@NEEDS_NODE
def test_the_pilot_keeps_the_helm_while_another_view_is_open():
    """Opening another view unmounts the Control Center. A beat kept in the
    panel stopped with it, the pilot lost the helm 3 s later and the helm
    stopped the verb they had sent (seen in the desktop app). The beat
    belongs to the page: it goes on after the panel is gone, and stops when
    the helm answers that this browser is no longer the pilot."""
    import json as _json
    import subprocess as _sp
    shim = ('function E(t){return{t,className:"",style:{},dataset:{},kids:[],value:t==="select"?"go":"",checked:false,'
            'append(...k){this.kids.push(...k)},replaceChildren(...k){this.kids=k},'
            'setAttribute(){},addEventListener(){},querySelectorAll(){return[]},querySelector(){return null},'
            'classList:{toggle(){}},options:[]}}'
            'const document={createElement:E,querySelector:()=>true,head:{append(){}}};'
            'const localStorage={getItem:()=>"ana",setItem(){}};'
            'const timers=[];globalThis.setInterval=(f)=>{timers.push(f);return timers.length};'
            'globalThis.clearInterval=(i)=>{timers[i-1]=()=>{}};')
    pre = ('const module={exports:{}};const CAMERAS=["forward"],'
           'VERBS={go:{help:"",fields:{},defaults:{}}},TUNABLE={},THEME_CSS="";')
    # Lichtblick's own words for a call from an unmounted panel (its
    # PanelExtensionAdapter); the Status strip stands for the panel that stays.
    probe = ('\nconst calls=[]; let answer="pilot", gone=false;'
             'const reply=(who)=>(s,r)=>{calls.push(who);return Promise.resolve({ok:true,message:answer})};'
             'const ctx={panelElement:{append(){}},subscribe(){},watch(){},'
             ' callService:(s,r)=>gone?Promise.reject(new Error("Service call after panel was unmounted"))'
             '  :reply("control")(s,r)};'
             'module.exports.activate({registerPanel:({initPanel})=>{'
             ' const off=initPanel(ctx); if(off) off();'
             ' ctx.onRender({currentFrame:[{topic:"/mongla/batighor/helm_state",'
             '  message:{data:JSON.stringify({pilot:"ana"})}}]},()=>{});'
             ' gone=true;}});'                              # another view opens
             'mount({panelElement:{append(){}},callService:reply("status")});'
             'const tick=async()=>{for(const f of timers)f(); await new Promise(r=>setTimeout(r,0));};'
             '(async()=>{await tick();await tick();const while_pilot=calls.length;'
             ' answer="watching"; await tick(); await tick(); await tick();'
             ' process.stdout.write(JSON.stringify([while_pilot, calls.length, calls]));})();')
    js = pre + shim + (PANELS / 'common.js').read_text() + HELM_JS + probe
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    while_pilot, total, who = _json.loads(out.stdout)
    assert while_pilot == 2, 'the beat goes on after the panel is gone'
    assert set(who) == {'status'}, 'through a panel that is still mounted'
    assert total == 3, 'and stops on the first answer that is not "pilot"'


@NEEDS_NODE
def test_a_calibration_button_outlives_the_states_that_follow_it():
    """The calibration state comes up to 4 times a second while a board is in
    view. Its library's apply buttons were built again on every one, and a
    button replaced between press and release loses the click."""
    import json as _json
    import subprocess as _sp
    shim = ('function E(t){return{t,className:"",style:{},dataset:{},kids:[],value:"",'
            'append(...k){this.kids.push(...k)},replaceChildren(...k){this.kids=k},'
            'setAttribute(){},addEventListener(){},querySelector(){return null}}}'
            'globalThis.Node=function(){};'
            'const document={createElement:E,querySelector:()=>true,head:{append(){}}};'
            'const localStorage={getItem:()=>"ana",setItem(){}};')
    pre = 'const module={exports:{}};const THEME_CSS="";'
    probe = ('\nconst ctx={panelElement:{append(){}},subscribe(){},watch(){},callService(){}};'
             'const lib=[{file:"pi_downward_water.yaml",medium:"water",width:640,height:480,'
             ' fx:500,max_ere_px:0.4}];'
             'const st=(hold)=>({topic:"x",message:{data:JSON.stringify({i:0,n:20,applies_to:"pi_downward",'
             ' medium:"water",grid:"8x6",square:0.025,cell:"centre",band:"near",hold,library:lib})}});'
             'const find=(e)=>e.t==="button"&&e.kids[0]==="apply"?e:(e.kids||[]).map(find).find(Boolean);'
             'module.exports.activate({registerPanel:({initPanel})=>{initPanel(ctx);'
             ' ctx.onRender({currentFrame:[st(0.1)]},()=>{}); const lb=ctx._lib;'
             ' const a=find(globalThis.ROOT); ctx.onRender({currentFrame:[st(0.6)]},()=>{});'
             ' const b=find(globalThis.ROOT); lib.push({file:"second.yaml"});'
             ' ctx.onRender({currentFrame:[st(0.7)]},()=>{}); const c=find(globalThis.ROOT);'
             ' process.stdout.write(JSON.stringify([Boolean(a), a===b, b===c, a.disabled]));}});')
    common = (PANELS / 'common.js').read_text().replace(
        'context.panelElement.append(box);', 'context.panelElement.append(box); globalThis.ROOT = box;')
    js = pre + shim + common + (PANELS / 'calibrate' / 'extension.js').read_text() + probe
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    found, kept, rebuilt_same, disabled = _json.loads(out.stdout)
    assert found and not disabled, 'a named operator sees an apply button'
    assert kept, 'a new hold value does not replace the button'
    assert not rebuilt_same, 'a changed library does'


def test_a_detector_setting_stays_on_one_row():
    """Detectors at 1600 px (Kutum, 2026-11-22): a flex row wrapped a long
    value ("model_path, as launched") onto its own line above its input, where
    it read as the row above's; and the model datalists sat in the label/value
    grid, so the odd/even label colours shifted after the first. The row is a
    fixed three-column grid and the datalists live in their own rows."""
    root = pathlib.Path(__file__).resolve().parents[1]
    js = (root / 'panels' / 'control' / 'extension.js').read_text()
    css = (root / 'theme' / 'batighor.css').read_text()
    assert 'tuneGrid.append(list)' not in js
    assert 'el("div", { class: "row tune" })' in js
    assert '.mb .row.tune{display:grid' in css
    assert 'text-overflow:ellipsis' in css.split('.mb .row.tune > :first-child', 1)[1].split('}', 1)[0]


@NEEDS_NODE
def test_the_log_folds_a_repeated_warning_into_one_line_with_a_count():
    """foldNodeLog: a node that says the same thing every second (the lock's
    'camera looks degraded', Kutum 2026-11-23) took the whole log card. A
    repeat, digits aside, replaces its earlier line with a count; a different
    message keeps its own line."""
    import json as _json
    import subprocess as _sp
    js = ((PANELS / 'control' / 'extension.js').read_text()
          .replace('module.exports = {', 'const _unused = {', 1)
          + '\nconst L=[];'
          + 'foldNodeLog(L,{t:1,key:"W lock: 7/8 collapsed",text:"a"});'
          + 'foldNodeLog(L,{t:2,key:"W loc: no aiding",text:"b"});'
          + 'foldNodeLog(L,{t:3,key:"W lock: 8/8 collapsed",text:"c"});'
          + 'foldNodeLog(L,{t:4,key:"E mgr: thruster 3 no rpm",text:"d"});'
          + 'foldNodeLog(L,{t:5,key:"E mgr: thruster 5 no rpm",text:"e"});'
          + 'process.stdout.write(JSON.stringify(L.map(e=>[e.t,e.n,e.text])));'
          # the heartbeat's setInterval would keep node running
          + 'process.exit(0);')
    pre = 'const module={exports:{}};const VERBS={},TUNABLE={},THEME_CSS="";'
    out = _sp.run(['node', '-e', pre + (PANELS / 'common.js').read_text() + js],
                  capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    got = _json.loads(out.stdout)
    # Two thrusters failing are two lines, not one with a count (audit 2026-11-24).
    assert got == [[2, 1, 'b'], [3, 2, 'c  (x2)'], [4, 1, 'd'], [5, 1, 'e']], got


@NEEDS_NODE
def test_the_strip_measures_lag_from_the_states_own_stamp():
    """noteLag + statusOf: a full bridge queue keeps delivering messages, each
    seconds old, so an arrival-gap check reads fresh while the operator looks
    at the past. The lag is arrival minus the state's own stamp, minus the
    least of that over the last minute, so the two clocks' offset cancels.
    Over 0.5 s it warns, over 2 s it is an error; the helm's video scale is
    shown beside it."""
    import json as _json
    import subprocess as _sp
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    js = (pre + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS + r'''
const X = module.exports, out = {};
const st = (sec) => ({ sec: Math.floor(sec), nanosec: Math.round((sec % 1) * 1e9) });
const s = { v: { armed: false, mode: "MANUAL" }, video: 0.5 };
// Kutum's clock is 40 s behind this one; the link adds 10 ms, then 2.5 s.
for (let i = 0; i < 20; i++) X.noteLag(s, 100000 + i * 50, st(60 + i * 0.05 - 0.01));
s.vAt = 101000;
out.calm = X.statusOf(s, 101000).cells.find((c) => c[0] === "to kutum");
X.noteLag(s, 101050, st(61.05 - 0.01 - 0.7));
out.slow = X.statusOf(s, 101050).cells.find((c) => c[0] === "to kutum");
X.noteLag(s, 101100, st(61.1 - 0.01 - 2.5));
out.late = X.statusOf(s, 101100).cells.find((c) => c[0] === "to kutum");
X.noteLag(s, 170000, st(130 - 0.01));
out.aged = s.lagWin.length;
// Lichtblick's receiveTime spells it nsec; 0.3 s late against the floor above.
X.noteLag(s, 170100, { sec: 130, nsec: 90e6 - 300e6 + 1e9 * 0 });
out.nsec = Math.round(s.lag * 1000);
// Reports: none in the first 15 s after mounting (a page that has just loaded
// lags while every panel mounts), then one every 2 s.
const r = { lag: 1.0, mountAt: 0 };
out.reports = [1000, 14000, 15500, 16000, 17600].map((t) => { r.vAt = t - 100; return X.shouldReport(r, t); });
// With the state silent the lag is frozen at its last arrival: never reported.
r.vAt = 17600; out.frozen = X.shouldReport(r, 17600 + 2000 + 2100);
out.words = [X.videoWords(1, ""), X.videoWords(0.5, "ab12", "ab12"), X.videoWords(0.25, "c3d4", "ab12"), X.videoWords(NaN, "")];
process.stdout.write(JSON.stringify(out));
process.exit(0);
''')
    run = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stderr
    got = _json.loads(run.stdout)
    assert got['calm'] == ['to kutum', 'lag 0.00 s  video x0.5', 'ok', 4], got['calm']
    assert got['slow'] == ['to kutum', 'lag 0.70 s  video x0.5', 'warn', 2], got['slow']
    assert got['late'] == ['to kutum', 'lag 2.50 s  video x0.5', 'err', 1], got['late']
    assert got['aged'] == 1, 'samples older than a minute are dropped'
    assert got['nsec'] == 300, got['nsec']
    assert got['reports'] == [False, False, True, False, True], got['reports']
    assert got['frozen'] is False
    assert got['words'] == ['video x1', 'video x0.5 set by this screen',
                            'video x0.25 set by screen c3d4', 'video --'], got['words']


@NEEDS_NODE
def test_nothing_is_shown_current_after_its_publisher_went_silent():
    """A dead manager's last green, its latched verb, a mission's latched task
    and the filter's last pose all stayed on screen as current (audit
    2026-11-24). Each now ages by its publisher's own rate."""
    import json as _json
    import subprocess as _sp
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    js = pre + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS + r"""
const X = module.exports, out = {};
const s = { health: [{ name: "mongla: leak", level: 0, message: "dry", at: 1000 }],
            doing: { now: { verb: "move_forward" } }, v: { armed: false }, vAt: 1000 };
out.freshHealth = X.statusOf(s, 2000).cells.find((c) => c[0] === "health");
out.deadHealth = X.statusOf(s, 1000 + 4000).cells.find((c) => c[0] === "health");
out.row = X.healthNow(s, 5000)[0];
out.freshVerb = X.statusOf(s, 2000).cells.find((c) => c[0] === "verb");
out.deadVerb = X.statusOf(s, 5000).cells.find((c) => c[0] === "verb");
process.stdout.write(JSON.stringify(out));
process.exit(0);
"""
    run = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stderr
    got = _json.loads(run.stdout)
    assert got['freshHealth'][2] == 'ok', got
    assert got['deadHealth'][2] == 'stale' and 'unknown' in got['deadHealth'][1], got
    assert got['row']['level'] == 3 and 'silent 4 s' in got['row']['message'], got
    assert got['freshVerb'] == ['verb', 'move_forward', 'live'], got
    assert got['deadVerb'][2] == 'warn' and 'manager silent' in got['deadVerb'][1], got


@NEEDS_NODE
def test_the_localization_card_says_why_flow_is_uncalibrated_in_full():
    """The flow view's state panel clipped the reason after a few words, and
    the card said only 'flow uncalibrated'. The card now carries the flow's
    own calibration state, whole, where it wraps."""
    import json as _json
    import subprocess as _sp
    shim = ('const document={createElement:(t)=>({t,className:"",kids:[],attrs:{},style:{},'
            'append(...k){this.kids.push(...k)},setAttribute(k,v){this.attrs[k]=v},'
            'addEventListener(){},querySelector(){return null}}),'
            'createElementNS:(n,t)=>document.createElement(t)};'
            'const text=(e)=>typeof e==="string"?e:(e.kids||[]).map(text).join("");')
    pre = ('const module={exports:{}};const CAMERAS=["forward","downward"],VERBS={},'
           'TUNABLE={},THEME_CSS="";')
    why = ("uncalibrated: 'calibration_downward_1280x720.json' was taken at 1280x720 "
           "and the frames are 640x480")
    probe = ('\nconst I=module.exports.INSTRUMENTS.LocalizationMap, s={};'
             'const out=[];'
             'I.fold(s,"/mongla/batighor/course",{data:JSON.stringify({course:"t",from:"deck",'
             'pool:{depth_m:2,tile_m:0.25,outline:[[0,0],[4,0],[4,9],[0,9]]},props:[]})});'
             f'I.fold(s,"/mongla/vision/downward/flow_calibration",{{data:{_json.dumps(why)}}});'
             'out.push(I.draw(s).map(text).join("|"));'
             'I.fold(s,"/mongla/vision/downward/flow_calibration",{data:"calibrated"});'
             'out.push(I.draw(s).map(text).join("|"));'
             'process.stdout.write(JSON.stringify(out));')
    js = pre + shim + (PANELS / 'common.js').read_text() + INSTRUMENTS_JS + probe
    out = _sp.run(['node', '-e', js], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    bad, good = _json.loads(out.stdout)
    assert 'flow scale' + why in bad, bad
    assert 'flow scalecalibrated' in good, good


@NEEDS_NODE
def test_a_failed_safety_verb_never_reads_like_one_that_took():
    """The log printed "disarm ended" for a disarm the manager failed, and
    dropped its reason (audit 2026-11-24)."""
    import json as _json
    import subprocess as _sp
    js = ((PANELS / 'control' / 'extension.js').read_text()
          .replace('module.exports = {', 'const _x = {', 1)
          + '\nconst f=(e)=>logLine(JSON.stringify({t:0,operator:"ana",...e})).text.slice(10);'
          + 'process.stdout.write(JSON.stringify([f({event:"disarm ended",ok:false,message:"no heartbeat"}),'
          + 'f({event:"disarm ended",ok:true,message:"disarmed"}),f({event:"pilot ana went silent"})]));'
          + 'process.exit(0);')
    pre = 'const module={exports:{}};const VERBS={},TUNABLE={},THEME_CSS="";'
    out = _sp.run(['node', '-e', pre + (PANELS / 'common.js').read_text() + js],
                  capture_output=True, text=True, timeout=30)
    assert out.returncode == 0, out.stderr
    bad, good, plain = _json.loads(out.stdout)
    assert 'FAILED: no heartbeat' in bad
    assert 'FAILED' not in good and good.endswith('disarm ended: disarmed')
    assert plain.endswith('pilot ana went silent')
