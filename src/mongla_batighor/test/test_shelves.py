"""Batighor shows the missions and the deck, read only, and nothing beside them.

Built so the answers are fixed first: a mission in the shelf is listed and
read; `..`, a symlink out, a dotfile, a binary and an oversize file are each
refused with their reason; a write is refused by the server.
"""
import json
import threading
import urllib.error
import urllib.request

import pytest

from mongla_batighor import serve, shelves


@pytest.fixture()
def shelf(tmp_path):
    root = tmp_path / 'missions'
    (root / 'sauvc').mkdir(parents=True)
    (root / 'sauvc' / 'gate.py').write_text('mongla.arm()\n')
    (root / '__init__.py').write_text('')
    (root / '.secret.yaml').write_text('key: 1\n')
    (root / 'blob.bin').write_bytes(b'\0' * 8)
    (root / 'big.txt').write_text('x' * (shelves.MAX_BYTES + 1))
    (tmp_path / 'outside.yaml').write_text('no: 1\n')
    (root / 'link.yaml').symlink_to(tmp_path / 'outside.yaml')
    return {'missions': root, 'deck': tmp_path / 'not-here'}


def test_a_mission_is_listed_and_read_and_nothing_else_is(shelf):
    got = shelves.listing(shelf)
    # The dotfile, the binary, the link out and the package's __init__.py
    # (plumbing, not a mission: it was listed among them, Chrome 2026-11-25)
    # are not listed.
    assert sorted(f['path'] for f in got['missions']) == ['big.txt', 'sauvc/gate.py']
    assert got['deck'] == []
    assert shelves.read('missions', 'sauvc/gate.py', shelf) == (b'mongla.arm()\n', '')


@pytest.mark.parametrize('shelf_name, rel, why', [
    ('missions', '../outside.yaml', 'outside'),
    ('missions', 'link.yaml', 'outside'),
    ('missions', '.secret.yaml', 'not a text file'),
    ('missions', 'blob.bin', 'not a text file'),
    ('missions', 'big.txt', 'larger than'),
    ('missions', 'nope.py', 'no such file'),
    ('deck', 'bringup.yaml', 'not on this machine'),
    ('home', '.ssh/id_ed25519', 'no shelf'),
])
def test_each_refusal_says_why(shelf, shelf_name, rel, why):
    body, said = shelves.read(shelf_name, rel, shelf)
    assert body is None and why in said


def test_the_server_serves_the_shelf_and_refuses_a_write(shelf, tmp_path):
    web = tmp_path / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<html></html>')
    layouts = tmp_path / 'layouts'
    layouts.mkdir()
    srv = serve.make_server(0, web, layouts, 8765, shelves_at=shelf)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{srv.server_address[1]}'
    try:
        idx = json.loads(urllib.request.urlopen(f'{base}/files/index.json').read())
        assert 'sauvc/gate.py' in [f['path'] for f in idx['missions']]
        assert urllib.request.urlopen(f'{base}/files/missions/sauvc/gate.py').read() == b'mongla.arm()\n'
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(f'{base}/files/missions/%2e%2e/outside.yaml')
        assert e.value.code == 404 and b'outside' in e.value.read()
        req = urllib.request.Request(f'{base}/files/missions/sauvc/gate.py', data=b'x', method='PUT')
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(req)
        assert e.value.code == 403
    finally:
        srv.shutdown()


def test_the_missions_shelf_is_where_the_runner_loads_them():
    from mongla_planner.missions import _find_src_missions_dir
    import mongla_planner.missions as m
    want = _find_src_missions_dir() or __import__('pathlib').Path(m.__file__).parent
    assert shelves.SHELVES['missions'] == want
    assert any(p.suffix == '.py' and not p.name.startswith('_') for p in want.iterdir())
