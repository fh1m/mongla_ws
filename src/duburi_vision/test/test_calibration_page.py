"""The calibration page's JavaScript must PARSE.

⛔ THIS CAUGHT A SHIPPED DEFECT. Commit `1e27851` published a page whose
entire `<script>` block was a JavaScript SyntaxError, so **nothing on the
page worked** -- no status polling, no camera list, no buttons. It looked
like a page that had simply stopped updating, and every Python test passed,
because the bug is in a string.

The cause is a trap specific to writing JS inside a Python string: `PAGE` is
a plain triple-quoted string, so a lone `\\'` collapses to `'` and a lone
`\\n` becomes a real newline. Both produce broken JS silently --
`sw(\\''+c.device+'\\')` emitted `sw(''+c.device+'')`, which is two adjacent
string literals.

`node --check` is the cheapest thing that would have caught it. It is
skipped where node is absent (the Pi), so it is a dev-box guard -- which is
where the page is edited.
"""
import os
import re
import shutil
import subprocess

import pytest

pytest.importorskip('cv2')


def _script(page):
    return ''.join(re.findall(r'<script>(.*?)</script>', page, re.S))


def test_the_page_has_a_script_at_all():
    from duburi_vision.calibration import guide
    js = _script(guide.PAGE)
    assert len(js) > 1000
    # The functions the page cannot work without.
    for fn in ('function tick', 'function cams', 'function lib',
               'function ap', 'function sw', 'function setmed'):
        assert fn in js, fn


@pytest.mark.skipif(not shutil.which('node'), reason='node not installed')
def test_the_page_javascript_parses(tmp_path):
    from duburi_vision.calibration import guide
    p = tmp_path / 'page.js'
    p.write_text(_script(guide.PAGE))
    r = subprocess.run(['node', '--check', str(p)],
                       capture_output=True, text=True)
    assert r.returncode == 0, (
        'the calibration page ships broken JavaScript, so the WHOLE page is '
        'inert:\n' + r.stderr)


def test_every_route_the_page_calls_actually_exists():
    """A fetch to a route the server does not serve fails silently.

    The page's own 404 handling is a swallowed `catch(e){}` per poll, so a
    renamed route reads as "the panel stopped updating".
    """
    from duburi_vision.calibration import guide
    js = _script(guide.PAGE)
    called = set(re.findall(r"fetch\('(/[a-z._]+)", js))
    served = set(re.findall(r"self\.path(?:\.startswith\()?\s*==?=?\s*'(/[a-z._]*)'",
                            open(guide.__file__).read()))
    served |= set(re.findall(r"startswith\('(/[a-z._]+)'\)",
                             open(guide.__file__).read()))
    # '/' and '/video' are used as element src, not fetch.
    missing = called - served
    assert not missing, f'page fetches routes the server does not serve: {missing}'
