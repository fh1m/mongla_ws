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
    tgz, digest = _tarball(tmp_path, {'index.html': b'<html>app</html>',
                                      'main.0123456789abcdef01.js': big,
                                      'main.0123456789abcdef01.js.map': b'{}'})
    fetch.unpack(tgz, web, digest)
    layouts = tmp_path / 'layouts'
    layouts.mkdir()
    (layouts / 'mongla_layout.json').write_text('{"configById": {}}')
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
    assert loc.endswith('&workspace=mongla&layout=mongla_layout') and 'layoutUrl' not in loc


def test_the_link_names_the_host_the_browser_used():
    link = serve.app_link('mongla.local:8000', 9000, 'x')
    assert 'ws%3A%2F%2Fmongla.local%3A9000' in link
    assert link.endswith('&layout=x')
    assert 'ws%3A%2F%2F192.168.0.103%3A8765' in serve.app_link('192.168.0.103', 8765, 'x')


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
    assert body == b'<html>app</html>' and headers['Cache-Control'] == 'no-cache'


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
    out = serve.absolute(layout, 'http://mongla.local:8000')
    layers = out['configById']['3D!a']['layers']
    assert layers['hull']['url'] == 'http://mongla.local:8000/model/mongla.urdf'
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
