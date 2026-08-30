"""The lab must never serve an index.html whose assets are not there.

This is a real failure that shipped: `colcon build` installs index.html as a
symlink back to source (so it tracks every frontend build), but setup.py
enumerates the hashed asset FILENAMES at colcon-build time. Rebuild the
frontend alone and the installed share holds a current index.html beside
assets from the previous build -- `GET /` answers 200 and every
`/assets/index-<hash>.js` it names answers 404. A blank page, no error
anywhere, and the served directory looks populated.

So the directory chooser tests consistency, not the mere presence of an
index.html.
"""
import pytest

server = pytest.importorskip('duburi_sim_web.server')


def _build(root, js='index-aaaa.js', css='index-bbbb.css', assets=True):
    (root / 'index.html').write_text(
        f'<!doctype html><script src="/assets/{js}"></script>'
        f'<link rel="stylesheet" href="/assets/{css}">')
    if assets:
        d = root / 'assets'
        d.mkdir()
        (d / js).write_text('//')
        (d / css).write_text('/**/')
    return root


def test_complete_build_accepted(tmp_path):
    assert server._static_complete(_build(tmp_path))


def test_missing_directory_rejected(tmp_path):
    assert not server._static_complete(tmp_path / 'nope')


def test_index_without_its_assets_rejected(tmp_path):
    """The exact shipped bug: index.html present, hashed assets stale."""
    assert not server._static_complete(_build(tmp_path, assets=False))


def test_stale_hash_rejected(tmp_path):
    """Assets present but under the PREVIOUS build's names."""
    _build(tmp_path, assets=False)
    d = tmp_path / 'assets'
    d.mkdir()
    (d / 'index-OLDHASH.js').write_text('//')
    (d / 'index-OLDHASH.css').write_text('/**/')
    assert not server._static_complete(tmp_path)


def test_index_with_no_asset_refs_accepted(tmp_path):
    """A hand-written single-file page is complete by definition."""
    (tmp_path / 'index.html').write_text('<!doctype html><p>hi')
    assert server._static_complete(tmp_path)
