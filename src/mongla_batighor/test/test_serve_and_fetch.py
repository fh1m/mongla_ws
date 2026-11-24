"""Batighor's server and fetch, against a real socket and a real tarball."""
import gzip
import hashlib
import io
import json
import tarfile
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from mongla_batighor import fetch, serve
from mongla_batighor.pin import pin


def _tarball(tmp, files):
    p = tmp / 'web.tgz'
    with tarfile.open(p, 'w:gz') as t:
        for name, data in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            t.addfile(info, io.BytesIO(data))
    return p, hashlib.sha256(p.read_bytes()).hexdigest()


@pytest.fixture
def site(tmp_path):
    web = tmp_path / 'web'
    big = b'console.log(1);' * 200
    tgz, digest = _tarball(tmp_path, {'index.html': b'<html><head></head>app</html>',
                                      'main.0123456789abcdef01.js': big,
                                      'main.0123456789abcdef01.js.map': b'{}'})
    fetch.unpack(tgz, web, digest)
    layouts = tmp_path / 'layouts'
    layouts.mkdir()
    (layouts / 'mongla_layout.json').write_text('{"configById": {}}')
    (layouts / 'batighor.json').write_text('{"configById": {}}')
    srv = serve.make_server(0, web, layouts, bridge_port=8765)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}', big
    srv.shutdown()


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *a, **kw):
        return None


def _get(url, gzip_ok=False):
    req = urllib.request.Request(url, headers={'Accept-Encoding': 'gzip'} if gzip_ok else {})
    opener = urllib.request.build_opener(_NoRedirect)
    try:
        r = opener.open(req)
        return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


def test_root_opens_the_app_on_this_host_bridge_and_layout(site):
    base, _ = site
    status, headers, _ = _get(base + '/')
    assert status == 302
    loc = headers['Location']
    assert loc.startswith('/app/?ds=foxglove-websocket')
    assert 'ws%3A%2F%2F127.0.0.1%3A8765' in loc
    # Named, not sent: a layoutUrl raced the app's start-up (serve.app_link).
    # The console (status strip + a tab per view), not the bare cockpit: `/` opened
    # `mongla_layout` and an operator saw no tab bar at all (2026-11-22).
    assert loc.endswith('&workspace=mongla&layout=batighor') and 'layoutUrl' not in loc


def test_the_link_names_the_host_the_browser_used():
    link = serve.app_link('kutum.local:8000', 9000, 'x')
    assert 'ws%3A%2F%2Fkutum.local%3A9000' in link
    assert link.endswith('&layout=x')
    assert 'ws%3A%2F%2F192.168.0.103%3A8765' in serve.app_link('192.168.0.103', 8765, 'x')


def test_a_forged_host_is_refused_not_turned_into_a_bridge_address(site):
    """The Host header names the bridge the app connects to (CodeQL, #168)."""
    import http.client
    base, _ = site
    port = int(base.rsplit(':', 1)[1])
    for host in ('a b', 'x/y', 'x@y', 'h:80/\x00', 'x:8000:9'):
        c = http.client.HTTPConnection('127.0.0.1', port)
        c.putrequest('GET', '/', skip_host=True)
        c._output(f'Host: {host}'.encode())  # bypasses http.client's own check
        c.endheaders()
        r = c.getresponse()
        assert r.status == 400 and r.getheader('Location') is None, host
        c.close()
    for ok in ('kutum.local:8000', '192.168.0.103', '[::1]:8000'):
        assert serve._HOST.match(ok), ok


def test_an_unknown_layout_is_refused_not_served_empty(site):
    base, _ = site
    assert _get(base + '/?layout=nope')[0] == 404
    assert _get(base + '/?layout=../etc')[0] == 404


def test_hashed_assets_are_gzipped_and_cached_index_is_not(site):
    base, big = site
    status, headers, body = _get(base + '/app/main.0123456789abcdef01.js', gzip_ok=True)
    assert status == 200 and headers['Content-Encoding'] == 'gzip'
    assert gzip.decompress(body) == big and len(body) < len(big) / 4
    assert 'immutable' in headers['Cache-Control']
    status, headers, body = _get(base + '/app/')
    assert body == serve.themed(b'<html><head></head>app</html>')
    assert headers['Cache-Control'] == 'no-cache'


def test_nothing_outside_the_web_folder_is_served(site, tmp_path):
    import http.client
    base, _ = site
    (tmp_path / 'secret.txt').write_text('secret')
    host, port = base[len('http://'):].split(':')
    for raw in ('/app/../secret.txt', '/app/../../etc/passwd', '/etc/passwd'):
        c = http.client.HTTPConnection(host, int(port))
        c.putrequest('GET', raw, skip_accept_encoding=True)   # sent as written
        c.endheaders()
        r = c.getresponse()
        assert r.status == 404 and b'secret' not in r.read(), raw


def test_source_maps_are_left_out(site, tmp_path):
    assert not list((tmp_path / 'web').glob('*.map'))


def test_a_wrong_hash_writes_nothing(tmp_path):
    tgz, _ = _tarball(tmp_path, {'index.html': b'x'})
    with pytest.raises(ValueError, match='sha256'):
        fetch.unpack(tgz, tmp_path / 'web', '0' * 64)
    assert not (tmp_path / 'web').exists()


def test_an_unsafe_path_in_the_tarball_is_refused(tmp_path):
    tgz, digest = _tarball(tmp_path, {'../evil': b'x', 'index.html': b'x'})
    with pytest.raises(ValueError, match='unsafe'):
        fetch.unpack(tgz, tmp_path / 'web', digest)
    assert not (tmp_path / 'evil').exists() and not (tmp_path / 'web').exists()


def test_a_missing_build_says_how_to_install_it(tmp_path):
    srv = serve.make_server(0, tmp_path / 'absent', tmp_path, bridge_port=1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        status, _, body = _get(f'http://127.0.0.1:{srv.server_address[1]}/app/')
        assert status == 503 and b'ros2 run mongla_batighor fetch' in body
    finally:
        srv.shutdown()


def test_the_pin_is_a_release_asset_with_a_full_hash():
    p = pin(Path(__file__).resolve().parents[1])
    assert p['url'].endswith(f'/v{p["version"]}/lichtblick-web.tar.gz')
    assert len(p['sha256']) == 64 and int(p['sha256'], 16) >= 0


def test_the_3d_scene_gets_the_hull_and_nothing_else_under_model(tmp_path):
    """The URDF and every mesh it names come from memory (model.files); any
    other name, and any path, is not found."""
    import shutil
    import xml.etree.ElementTree as ET
    (tmp_path / 'layouts').mkdir()
    (tmp_path / 'model').mkdir()
    root = Path(__file__).resolve().parents[3]
    shutil.copy(root / 'docs' / 'assets' / 'cad' / 'hull.mgla', tmp_path / 'model' / 'hull.mgla')
    shutil.copy(root / 'src' / 'mongla_control' / 'config' / 'hull_geometry.yaml',
                tmp_path / 'model' / 'hull_geometry.yaml')
    srv = serve.make_server(0, tmp_path / 'web', tmp_path / 'layouts', bridge_port=1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{srv.server_address[1]}/model/'
    try:
        status, headers, body = _get(base + 'mongla.urdf')
        assert status == 200 and headers['Content-Type'] == 'application/xml'
        # The desktop app's page is a file: it reads these only with CORS.
        assert headers['Access-Control-Allow-Origin'] == '*'
        for m in ET.fromstring(body).iter('mesh'):
            status, headers, stl = _get(base + m.get('filename'))
            assert status == 200 and headers['Content-Type'] == 'model/stl'
            _, zh, z = _get(base + m.get('filename'), gzip_ok=True)
            assert zh['Content-Encoding'] == 'gzip' and gzip.decompress(z) == stl
            assert len(stl) > 84
        for bad in ('hull.mgla', '../layouts', 'nope.stl', ''):
            assert _get(base + bad)[0] == 404, bad
    finally:
        srv.shutdown()


def test_a_missing_hull_leaves_the_server_up(tmp_path, capsys):
    srv = serve.make_server(0, tmp_path / 'web', tmp_path, bridge_port=1)
    srv.server_close()
    assert 'no hull in the 3D scene' in capsys.readouterr().err


def test_a_3d_layer_url_reaches_the_app_absolute_on_the_host_it_used():
    """Lichtblick refuses a root-path URDF URL ("Invalid URDF URL", seen on
    Kutum 2026-11-12); the org layouts carry the browser's own host."""
    layout = {'configById': {'3D!a': {'layers': {'hull': {'url': '/model/mongla.urdf'},
                                                 'web': {'url': 'http://x/y.urdf'}}}}}
    out = serve.absolute(layout, 'http://kutum.local:8000')
    layers = out['configById']['3D!a']['layers']
    assert layers['hull']['url'] == 'http://kutum.local:8000/model/mongla.urdf'
    assert layers['web']['url'] == 'http://x/y.urdf'
    assert layout['configById']['3D!a']['layers']['hull']['url'] == '/model/mongla.urdf'


def test_the_org_layouts_carry_the_hosts_urdf_url(tmp_path):
    (tmp_path / 'layouts').mkdir()
    (tmp_path / 'layouts' / 'nav.json').write_text(
        json.dumps({'configById': {'3D!a': {'layers': {'h': {'url': '/model/mongla.urdf'}}}}}))
    srv = serve.make_server(0, tmp_path / 'web', tmp_path / 'layouts', bridge_port=1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    try:
        org = json.loads(_get(f'http://127.0.0.1:{port}/api/workspaces/mongla/layouts')[2])
        url = org['data'][0]['data']['configById']['3D!a']['layers']['h']['url']
        assert url == f'http://127.0.0.1:{port}/model/mongla.urdf'
        # Lichtblick refreshes its copy only when the date differs: another
        # host name must read as another date, the same one as the same.
        other = json.loads(_get(f'http://localhost:{port}/api/workspaces/mongla/layouts')[2])
        again = json.loads(_get(f'http://127.0.0.1:{port}/api/workspaces/mongla/layouts')[2])
        assert other['data'][0]['updatedAt'] != org['data'][0]['updatedAt']
        assert again['data'][0]['updatedAt'] == org['data'][0]['updatedAt']
    finally:
        srv.shutdown()


def test_a_layout_file_carries_the_hosts_urdf_url_for_the_desktop_app(tmp_path):
    """batighor_desktop.py copies `/layouts/<name>.json` into the desktop app's folder."""
    (tmp_path / 'layouts').mkdir()
    (tmp_path / 'layouts' / 'nav.json').write_text(
        json.dumps({'configById': {'3D!a': {'layers': {'h': {'url': '/model/mongla.urdf'}}}}}))
    srv = serve.make_server(0, tmp_path / 'web', tmp_path / 'layouts', bridge_port=1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    try:
        status, _, body = _get(f'http://127.0.0.1:{port}/layouts/nav.json')
        assert status == 200
        url = json.loads(body)['configById']['3D!a']['layers']['h']['url']
        assert url == f'http://127.0.0.1:{port}/model/mongla.urdf'
        assert _get(f'http://127.0.0.1:{port}/layouts/none.json')[0] == 404
    finally:
        srv.shutdown()


def test_a_path_that_leaves_the_web_root_is_refused_however_it_is_spelt(tmp_path):
    web = tmp_path / 'web'
    (web / 'a').mkdir(parents=True)
    (web / 'a' / 'x.js').write_text('x')
    (tmp_path / 'secret').write_text('s')
    (web / 'out').symlink_to(tmp_path / 'secret')
    files = serve.index(web)
    assert files == {'a/x.js': (web / 'a' / 'x.js').resolve()}
    for bad in ('', '.', '../secret', 'a/../../secret', str(tmp_path / 'secret'), 'out', '../web2/x'):
        assert bad not in files, bad


def test_a_symlink_install_serves_its_layouts_and_nothing_else(tmp_path):
    """Kutum builds with --symlink-install: every installed layout is a link
    into src/mongla_batighor/, and each font into docs/assets/fonts/.
    Excluding those served the vehicle no layout and no font (B-116). A link
    out of the workspace still stays out."""
    ws = tmp_path / 'ws'
    pkg, share = ws / 'src' / 'pkg', ws / 'install' / 'pkg' / 'share' / 'pkg'
    (pkg / 'layouts').mkdir(parents=True)
    (ws / 'docs' / 'fonts').mkdir(parents=True)
    (share / 'layouts').mkdir(parents=True)
    (share / 'theme' / 'fonts').mkdir(parents=True)
    (pkg / 'package.xml').write_text('<package/>')
    (pkg / 'layouts' / 'vision.json').write_text('{}')
    (ws / 'docs' / 'fonts' / 'a.woff2').write_text('f')
    (share / 'package.xml').symlink_to(pkg / 'package.xml')
    (share / 'layouts' / 'vision.json').symlink_to(pkg / 'layouts' / 'vision.json')
    (share / 'theme' / 'fonts' / 'a.woff2').symlink_to(ws / 'docs' / 'fonts' / 'a.woff2')
    (tmp_path / 'secret').write_text('s')
    (share / 'layouts' / 'out.json').symlink_to(tmp_path / 'secret')
    assert serve.index(share / 'layouts') == {
        'vision.json': (pkg / 'layouts' / 'vision.json').resolve()}
    assert serve.index(share / 'theme') == {
        'fonts/a.woff2': (ws / 'docs' / 'fonts' / 'a.woff2').resolve()}


def test_every_kind_of_file_lichtblick_ships_has_its_type():
    """The types come from a fixed table now. Lichtblick 1.30.0 ships these
    kinds (seen on Kutum); WebAssembly must be application/wasm or the
    browser refuses to stream-compile it."""
    for ext in ('.html', '.js', '.png', '.wasm', '.ttf', '.ico', '.json'):
        assert ext in serve.TYPES, ext
    assert serve.TYPES['.wasm'] == 'application/wasm'


def test_the_app_page_carries_the_first_visit_layout_fix_before_the_apps_scripts(site):
    """B-141: on a fresh profile the app matched `?layout=` before its sync had
    stored the organisation's layouts. The fix must be in the page the server
    sends, inside the head, so it sees the database as the app first found it."""
    base, _ = site
    _, _, body = _get(base + '/app/')
    assert serve.FIRST_VISIT_HEAD in body
    page = serve.themed(b'<html><head><script src="main.js"></script></head></html>')
    assert page.index(serve.FIRST_VISIT_HEAD) < page.index(b'</head>')


def test_the_first_visit_fix_never_creates_the_apps_database():
    """Opening `lichtblick-layouts` before the app does would create it empty
    at version 1, and the app's upgrade, which makes the `layouts` store, would
    never run: every later visit would hold no layouts at all. So the script
    asks `indexedDB.databases()` first and opens only a database that exists,
    and it reloads at most once per tab."""
    js = serve.FIRST_VISIT_HEAD.decode()
    assert js.index('indexedDB.databases()') < js.index('indexedDB.open(')
    assert 'indexedDB.open(DB)' in js, 'open with no version: never an upgrade of its own'
    assert 'objectStoreNames.contains("layouts")' in js
    assert 'sessionStorage.setItem(flag' in js and 'sessionStorage.getItem(flag)' in js
    assert js.count('location.reload()') == 1


def test_the_tab_and_the_menu_carry_mongla_not_lichtblick():
    """The page names and marks itself Batighor: Lichtblick's icons are gone,
    ours is linked, and the CSS's inline mark is the served file, byte for byte."""
    import pathlib
    import urllib.parse
    page = serve.themed(b'<html><head><title>Lichtblick</title>'
                        b'<link rel="icon" type="image/png" sizes="32x32" href="favicon-32x32.png"/>'
                        b'<link rel="apple-touch-icon" sizes="180x180" href="apple-touch-icon.png"/>'
                        b'</head></html>')
    assert b'favicon-32x32' not in page and b'apple-touch-icon' not in page
    assert b'<title>Batighor</title>' in page and b'href="/theme/mongla.svg"' in page
    assert serve._THEME_FILE.match('mongla.svg')
    theme = pathlib.Path(__file__).resolve().parents[1] / 'theme'
    svg = (theme / 'mongla.svg').read_text().strip()
    css = (theme / 'batighor.css').read_text()
    assert 'data:image/svg+xml,' + urllib.parse.quote(svg, safe='') in css


def test_the_desktop_app_can_list_and_read_the_layouts(site):
    """The instruments extension syncs the desktop app's copies from a page that
    is a file: the index names every layout and both answer with CORS."""
    base, _ = site
    status, headers, body = _get(base + '/layouts/index.json')
    assert status == 200 and json.loads(body) == ['batighor', 'mongla_layout']
    assert headers.get('Access-Control-Allow-Origin') == '*'
    status, headers, _ = _get(base + '/layouts/mongla_layout.json')
    assert status == 200 and headers.get('Access-Control-Allow-Origin') == '*'


def test_a_first_visit_opens_the_views_full_width():
    """Lichtblick opens its left sidebar by default, a fifth of the screen
    saying "Select a panel to edit its settings", and its first load writes
    that default to the workspace store. The page marks a profile with no
    store, and the first-visit script closes the sidebar in the store just
    before its one reload; a stored choice is never touched. Checked live in
    a fresh headless profile against Kutum. INJECTION: drop closeSidebar()
    from the reload, or rename the seed key on one side, and this fails."""
    from mongla_batighor import serve
    page = serve.themed(b'<html><head><title>Lichtblick</title></head><body></body></html>')
    assert b'localStorage.getItem("fox.workspace")===null' in page
    assert b'closeSidebar();location.reload()' in page
    assert b'open:false' in page
    # The key the head writes is the key closeSidebar reads; renaming one side
    # alone would leave every first visit with the sidebar open, silently.
    assert b'sessionStorage.setItem("batighor.sidebar-seed","1")' in page
    assert b'sessionStorage.getItem("batighor.sidebar-seed")' in page


def test_every_version_served_is_kept_and_served_back_newest_first(tmp_path):
    """A browser tells a working copy the app made of an old layout from an
    operator's edit by finding that old layout here (bead b0q)."""
    import json as _json
    import os as _os
    layouts, history = tmp_path / 'layouts', tmp_path / 'history'
    layouts.mkdir()
    f = layouts / 'batighor.json'
    srv = serve.make_server(0, tmp_path / 'web', layouts, bridge_port=1, history=history)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{srv.server_address[1]}'
    try:
        for v in (1, 2, 2, 3):
            f.write_text(_json.dumps({'configById': {}, 'v': v}))
            assert _get(f'{base}/api/workspaces/mongla/layouts')[0] == 200
            for p in (history / 'batighor').glob('*.json'):
                _os.utime(p, (p.stat().st_mtime - 1,) * 2)   # keep the order apart
        status, _, body = _get(f'{base}/layouts/history/batighor.json')
        assert status == 200
        assert [d['v'] for d in _json.loads(body)] == [3, 2, 1]
        assert _get(f'{base}/layouts/history/..%2Fx.json')[0] == 404
        assert _json.loads(_get(f'{base}/layouts/history/absent.json')[2]) == []
    finally:
        srv.shutdown()


def test_the_release_path_serve_seeds_from_is_the_one_release_sh_builds():
    import re as _re
    rel = (Path(__file__).resolve().parents[3] / 'kutum' / 'release.sh').read_text()
    base = _re.search(r'^BASE="\$HOME_DIR/([^"]+)"', rel, _re.M).group(1)
    releases = _re.search(r'^RELEASES="\$BASE/([^"]+)"', rel, _re.M).group(1)
    assert serve.RELEASE_LAYOUTS.startswith(f'{base}/{releases}/*/src/mongla_batighor/')
