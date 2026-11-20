"""The vehicle receives every file a package's setup.py installs.

`kutum/deploy.sh` excludes generated and dev-box trees from the rsync, and
`docs/` among them -- while mongla_batighor installs its fonts and hull from
`docs/assets/`. The release then failed to build on the vehicle with "can't
copy ... zed-mono-extended.woff2" (2026-11-18), on the machine Batighor is
served from. This runs rsync with deploy.sh's own filters, dry, and checks
that each data file a setup.py names would arrive.
"""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def _filters():
    text = (ROOT / 'kutum/deploy.sh').read_text()
    block = text[text.index('rsync -az --delete'):]
    block = block[:block.index('./ "$TARGET')]
    return re.findall(r"(--(?:include|exclude)) '([^']+)'", block)


def _installed_sources():
    """Every literal or f-string path under the repo that a setup.py installs
    from outside its own package directory."""
    out = []
    for setup in ROOT.glob('src/*/setup.py'):
        for s in re.findall(r"""['"](\.\./\.\./[^'"{]+)(?:\{[^}]*\}([^'"]*))?['"]""",
                            setup.read_text()):
            path, tail = s
            base = (setup.parent / path).resolve()
            if tail:          # f-string with one field: check the folder
                base = base if base.is_dir() else base.parent
            out.append(base)
    return out


@pytest.mark.skipif(shutil.which('rsync') is None, reason='rsync not installed')
def test_every_file_a_package_installs_from_outside_it_is_deployed():
    args = []
    for flag, pat in _filters():
        args += [f'{flag}={pat}']
    listing = subprocess.run(
        ['rsync', '-n', '-a', '--out-format=%n', *args, './', '/nonexistent-dest/'],
        cwd=ROOT, capture_output=True, text=True)
    sent = set(listing.stdout.split())
    sources = _installed_sources()
    assert sources, 'no setup.py installs from outside its package -- update this test'
    missing = []
    for p in sources:
        rel = str(p.relative_to(ROOT))
        if p.is_dir():
            if not any(s.startswith(rel + '/') for s in sent):
                missing.append(rel + '/')
        elif rel not in sent:
            missing.append(rel)
    assert not missing, f'deploy.sh would not send: {missing}'
