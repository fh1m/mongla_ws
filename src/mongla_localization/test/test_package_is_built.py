"""The build script names packages explicitly, so a new one must be added to it.

⛔ WHAT THIS CATCHES. `build_mongla.sh` does not discover packages, it lists
them. A package added to `src/` and not to that list is never built, never
installed, and on the vehicle its nodes simply do not exist -- while every test
on the dev box passes, because pytest imports from source. The failure surfaces
as "executable not found" in a launch, minutes into a pool session.

Also pins the two places that must agree with each other: every package under
`src/` appears in the build list, and every entry in the build list exists.
"""
from __future__ import annotations

import pathlib
import re

_WS = pathlib.Path(__file__).resolve().parents[3]
_BUILD = _WS / 'build_mongla.sh'


def _src_packages() -> set:
    return {p.name for p in (_WS / 'src').iterdir()
            if (p / 'package.xml').is_file()}


def _built_packages() -> set:
    text = _BUILD.read_text()
    return set(re.findall(r'mongla_[a-z_]+', text)) & _src_packages()


def test_every_package_in_src_is_in_the_build_script():
    missing = _src_packages() - _built_packages()
    # mongla_interfaces is built by the interfaces-first step, which the script
    # runs separately; it still has to appear somewhere in the file.
    assert not missing, (
        f'these packages exist but are never built: {sorted(missing)}. '
        f'On the vehicle their nodes would not exist at all.')


def test_the_build_script_names_nothing_that_does_not_exist():
    stale = _built_packages() - _src_packages()
    assert not stale, f'build script names packages that are gone: {sorted(stale)}'


def test_mongla_localization_is_among_them():
    # Named explicitly: it is the newest, and a rename that dropped it from the
    # glob would make the test above vacuously pass.
    assert 'mongla_localization' in _built_packages()
