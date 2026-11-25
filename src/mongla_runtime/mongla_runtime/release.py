"""A Mongla process runs the active release or it does not run (B-153).

On Kutum the workspace is `~/mongla_ws`, a link to one folder under
`~/mongla/releases/`. A release's setup files record its real path, so a shell
opened before a deploy keeps launching the old one, and nothing about the
running stack says so. Checked when `mongla_runtime` is imported, which every
package that ships a command does. Anywhere else (a dev box, CI) no path names
a release and nothing happens.
"""
import os
import re
import sys

_RELEASE = re.compile(r'/mongla/releases/([^/]+)/')


def loaded() -> list:
    """Where the command and every loaded Mongla module really live."""
    files = [getattr(m, '__file__', None) for n, m in list(sys.modules.items())
             if n.startswith('mongla_')]
    return [os.path.realpath(f) for f in [*files, *sys.argv[:1]] if f]


def refuse_other_release(paths=None, live=None) -> None:
    """Exit, loudly, if any of `paths` is in a release other than the active one."""
    live = os.path.expanduser('~/mongla_ws') if live is None else str(live)
    found = {m.group(1) for m in map(_RELEASE.search, loaded() if paths is None else paths) if m}
    if os.path.islink(live):
        wrong = found - {os.path.basename(os.path.realpath(live))}
        active = os.path.basename(os.path.realpath(live))
    else:                          # no active release to compare with: no mixing
        wrong = found if len(found) > 1 else set()
        active = 'none'
    if wrong:
        raise SystemExit(
            f'ERROR: refusing to run old code. This process loads release '
            f'{", ".join(sorted(wrong))}; the active one is {active}. This shell '
            f'was opened before a deploy: open a NEW shell and start again.')
