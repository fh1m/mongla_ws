"""Our panels, served to Lichtblick as a workspace's extensions.

Lichtblick loads an extension from a server only when the build names an API
(`apiUrl`) and the link names a workspace. The release build names none, so
`api_bundle` sets it in the pinned bundle as it is served, and fails loudly,
not silently, when a new version no longer carries the one line it changes.

An extension is a `.foxe`: a zip of `package.json` and `dist/extension.js`.
Ours are plain JavaScript with no build step; `foxe` zips one directory of
`panels/` and puts the verb table above the code, read from
`mongla_control/commands.py` and the Move action, so a verb added there
reaches the browser with no edit here.

Lichtblick keeps an extension in the browser and downloads it again only when
its version changes; the version is the content's hash, so every change is
seen and nothing unchanged is downloaded twice.
"""
from __future__ import annotations

import hashlib
import io
import json
import re
import zipfile
from pathlib import Path

WORKSPACE = 'mongla'
PUBLISHER = 'mongla'
API = '/api'

# The one switch in the release bundle (lichtblick 1.30.0, main.<hash>.js):
# its build-time config object. `version` makes the match exact to a release.
_API_OFF = re.compile(rb'apiUrl:void 0,(version:"[0-9.]+")')
_MAIN = re.compile(r'main\.[0-9a-f]{16,}\.js')


def verbs() -> dict:
    """Every verb the helm accepts: help, fields with their types, defaults."""
    from mongla_control.commands import COMMANDS
    from mongla_interfaces.action import Move

    types = Move.Goal.get_fields_and_field_types()
    return {name: {'help': row['help'],
                   'fields': {f: types[f] for f in row['fields']},
                   'defaults': row.get('defaults', {})}
            for name, row in COMMANDS.items()}


def foxe(src: Path, preamble: str = '') -> tuple[dict, bytes]:
    """(the API's description of the extension, the .foxe bytes) for one panel dir."""
    code = preamble + (src / 'extension.js').read_text()
    name = f'batighor-{src.name}'
    digest = int(hashlib.sha256(code.encode()).hexdigest()[:8], 16)
    info = {'name': name, 'publisher': PUBLISHER, 'version': f'1.0.{digest}',
            'displayName': f'Batighor {src.name}', 'description': (code.splitlines() or [''])[0].lstrip('/ '),
            'license': 'see the mongla_ws repository', 'main': 'dist/extension.js'}
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('package.json', json.dumps(info, indent=2))
        z.writestr('dist/extension.js', code)
    ext_id = f'{PUBLISHER}.{name}'
    entry = {
        'id': ext_id, 'extensionId': ext_id, 'fileId': ext_id, 'scope': 'org',
        # Lichtblick names the panel type `<qualifiedName>.<panel name>`.
        'qualifiedName': f'{PUBLISHER}.batighor',
        'createdAt': '1970-01-01T00:00:00Z', 'updatedAt': '1970-01-01T00:00:00Z',
        **{k: info[k] for k in ('name', 'publisher', 'version', 'displayName',
                                'description', 'license')},
        'homepage': '', 'keywords': [], 'readme': '', 'changelog': '',
    }
    return entry, buf.getvalue()


def extensions(panels_dir: Path) -> dict:
    """extension id -> (entry, foxe bytes), for every panel directory."""
    pre = f'const VERBS = {json.dumps(verbs(), sort_keys=True)};\n'
    out = {}
    for d in sorted(p for p in panels_dir.iterdir() if (p / 'extension.js').is_file()):
        entry, data = foxe(d, pre)
        out[entry['id']] = (entry, data)
    return out


def api_bundle(web: Path) -> tuple[str, str, bytes] | None:
    """(the bundle's name, the name it is served as, its bytes with the API on).

    None when the build has no `main.<hash>.js`. ValueError when the switch
    is not there exactly once: a new Lichtblick that moved it must be read
    again, not served half-working.
    """
    index = web / 'index.html'
    if not index.is_file():
        return None
    m = _MAIN.search(index.read_text(errors='replace'))
    if not m:
        return None
    raw = (web / m.group(0)).read_bytes()
    patched, n = _API_OFF.subn(rb'apiUrl:"' + API.encode() + rb'",\1', raw)
    if n != 1:
        raise ValueError(f'{m.group(0)}: the apiUrl switch was found {n} times, '
                         'expected once; read the new release before serving it')
    # A new name: a browser keeps the old bundle for a year (it is immutable
    # by its hash), so the changed one must not answer to the same name.
    return m.group(0), m.group(0).replace('.js', '.api.js'), patched
