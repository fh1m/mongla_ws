"""Where the real competition footage lives on a development box.

The path is deliberately NOT written down here. It carries a retired project
name, and a contract test bans that name from live code: it is the one string
that would tie this repository to work it no longer belongs to. Hardcoding it
also assumes one machine, which is wrong the moment anyone else runs a tool.

Resolution order, first hit wins:

  1. $MONGLA_ARCHIVE
  2. the first line of ~/.mongla/archive_root

Both are operator state, outside the repository. A tool that needs footage
calls `archive_root()` and gets a clear refusal rather than a path that does
not exist on this machine -- which is the failure this replaces, because an
absent directory reads downstream as "no clips matched" and looks like a
finding about the footage instead of about the path.
"""
import os
import pathlib

ENV_VAR = 'MONGLA_ARCHIVE'
CONFIG = pathlib.Path.home() / '.mongla' / 'archive_root'


def archive_root(required: bool = True) -> str:
    """The footage root. Raises SystemExit with instructions when unset."""
    value = os.environ.get(ENV_VAR, '').strip()
    if not value and CONFIG.is_file():
        value = CONFIG.read_text().splitlines()[0].strip() if \
            CONFIG.read_text().strip() else ''
    if value and pathlib.Path(value).is_dir():
        return value
    if not required:
        return value
    raise SystemExit(
        f'no footage archive.\n'
        f'  export {ENV_VAR}=/path/to/raw_videos\n'
        f'  or:  mkdir -p {CONFIG.parent} && '
        f'echo /path/to/raw_videos > {CONFIG}\n'
        + (f'  (currently set to {value!r}, which is not a directory)'
           if value else ''))
